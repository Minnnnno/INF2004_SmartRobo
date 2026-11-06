#!/usr/bin/env python3
"""Barcode log analyser (standard library only).

Reads the serial logs the logger writes (default: Resources/logs_archive/run_*.txt, one file per
boot) and prints:
  1. one row per barcode pass: result, white level, peak, contrast, samples, elements;
  2. the failed passes sorted into groups (bad white reference, weak contrast, wrong element
     count, right count but a wrong character);
  3. for failed passes with a 29-element width list (bw), which character failed and which
     elements the decoder called wide/narrow, against the expected letter;
  4. for failed passes with distance/heading dumps (bp/bh), where the barcode sensor was
     sideways when each bar went by, to test whether the line follower's wiggle swings the
     sensor off the bars.

The firmware's own element widths (bw) are used; elements are not re-extracted here.
Usage:
  python tools/barcode_log.py                      all archived runs
  python tools/barcode_log.py FILE [FILE ...]      chosen logs
  python tools/barcode_log.py --expect Z --since 20261105-2100
  python tools/barcode_log.py --selftest
"""
import argparse
import glob
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_GLOB = os.path.join(ROOT, 'Resources', 'logs_archive', 'run_*.txt')

SENSOR_AHEAD_M = 0.18   # barcode sensor ahead of the wheel axle (15 cm to the line IRs + 3 cm)
WEAK_CONTRAST = 1500    # successful on-line reads peaked 2300-3300 above white
BAD_WHITE = 1000        # the floor reads about 140-250; a "white" above this was not floor

# Code 39, as in barcode.c: 9 elements (bar first), bit 8 = first element, 1 = wide.
SYMBOLS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. *$/+%"
PATTERNS = [0x034, 0x121, 0x061, 0x160, 0x031, 0x130, 0x070, 0x025, 0x124, 0x064,
            0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C, 0x01C,
            0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106, 0x046, 0x016,
            0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0,
            0x085, 0x184, 0x0C4, 0x094, 0x0A8, 0x0A2, 0x08A, 0x02A]
PATTERN_OF = dict(zip(SYMBOLS, PATTERNS))
SYMBOL_OF = dict(zip(PATTERNS, SYMBOLS))


# ---- port of the firmware's per-character classifier (barcode.c) ----
def widest(d, first, count):
    pattern = 0
    for _ in range(count):
        best = None
        for i in range(first, 9, 2):
            if not pattern & (1 << (8 - i)) and (best is None or d[i] > d[best]):
                best = i
        pattern |= 1 << (8 - best)
    return pattern


def nth(d, first, n):
    taken = widest(d, first, n - 1)
    return max([d[i] for i in range(first, 9, 2) if not taken & (1 << (8 - i))] or [0])


def char_pattern(d):
    def ratio(a, b):
        return a / b if b else float('inf')
    normal = ratio(nth(d, 0, 2), nth(d, 0, 3)) * ratio(nth(d, 1, 1), nth(d, 1, 2))
    spaces = ratio(nth(d, 1, 3), nth(d, 1, 4)) * ratio(nth(d, 0, 5), nth(d, 0, 1))
    if spaces > normal:
        return widest(d, 1, 3)
    return widest(d, 0, 2) | widest(d, 1, 1)


def decode(d):
    """Firmware rules: n = 10*chars - 1, starts and ends with '*', forward then reversed.
    Returns (text or None, per-direction list of (chars, patterns))."""
    tries = []
    for direction, seq in (('forward', d), ('reversed', d[::-1])):
        n = len(seq)
        if n % 10 != 9:
            tries.append((direction, None, []))
            continue
        pats = [char_pattern(seq[c * 10:c * 10 + 9]) for c in range(n // 10 + 1)]
        text = ''.join(SYMBOL_OF.get(p, '?') for p in pats)
        tries.append((direction, text, pats))
        if '?' not in text and len(text) >= 3 and text[0] == '*' and text[-1] == '*':
            return text, tries
    return None, tries


def selftest():
    ok = True
    for letter in 'ZAX':
        widths = []
        for c in '*' + letter + '*':
            p = PATTERN_OF[c]
            widths += [30 if p & (1 << (8 - i)) else 10 for i in range(9)] + [10]
        widths = widths[:-1]
        for seq, name in ((widths, 'forward'), (widths[::-1], 'reversed')):
            text, _ = decode(seq)
            good = text == '*' + letter + '*'
            ok &= good
            print(f"selftest {letter} {name}: {text} {'ok' if good else 'FAIL'}")
    return ok


# ---- log parsing ----
NUM_LINE = re.compile(r'^-?\d+( -?\d+)*$')
PREFIXED = re.compile(r'^(bv|bp|bh|bw)((?: -?\d+)+)$')
READ = re.compile(r'^barcode: \*([^*]*)\*$')
ARMED = re.compile(r'^barcode: armed, white=(\d+) peak=(\d+)')
NOREAD = re.compile(r'^barcode: no read, elements=(\d+) samples=(\d+) white=(\d+) min=(\d+) max=(\d+)')
STRETCH = re.compile(r'^barcode: stretch ended, no code \(samples=(\d+) elements=(\d+)'
                     r'(?: white=(\d+) min=(\d+) max=(\d+))?\)')
TRIP = re.compile(r'^shuttle: trip (\d+) (\w+)')


def parse(path):
    passes, context = [], ''
    lines = [l.rstrip('\r\n') for l in open(path, errors='ignore')]
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        m = TRIP.match(line)
        if m:
            context = f'trip {m[1]} {m[2]}'
        m = READ.match(line)
        if m:
            p = {'line': i + 1, 'result': 'read *%s*' % m[1], 'letter': m[1], 'ctx': context}
            if i + 1 < len(lines):
                a = ARMED.match(lines[i + 1].strip())
                if a:
                    p['white'], p['max'] = int(a[1]), int(a[2])
            passes.append(p)
        m = STRETCH.match(line)
        if m:
            p = {'line': i + 1, 'result': 'fragment', 'samples': int(m[1]), 'elements': int(m[2]), 'ctx': context}
            if m[3]:
                p['white'], p['min'], p['max'] = int(m[3]), int(m[4]), int(m[5])
            passes.append(p)
        m = NOREAD.match(line)
        if m:
            p = {'line': i + 1, 'result': 'NO READ', 'elements': int(m[1]), 'samples': int(m[2]),
                 'white': int(m[3]), 'min': int(m[4]), 'max': int(m[5]), 'ctx': context,
                 'bv': [], 'bp': [], 'bh': [], 'bw': [], 'notes': []}
            j = i + 1
            # The dump: prefixed lines (newer builds) or bare-number lines (older builds). Other tasks'
            # lines can land in between; skip them, stop at the next barcode pass.
            while j < len(lines):
                s = lines[j].strip()
                if s.startswith('barcode: reading') or READ.match(s) or NOREAD.match(s) or STRETCH.match(s):
                    break
                pm = PREFIXED.match(s)
                if pm:
                    p[pm[1]] += [int(x) for x in pm[2].split()]
                elif NUM_LINE.match(s) and not any(p[k] for k in ('bp', 'bh', 'bw')):
                    p['bv'] += [int(x) for x in s.split()]
                if s.startswith('barcode: no read while not driving'):
                    p['notes'].append('not counted (car not driving)')
                j += 1
            for key, want in (('bv', p['samples']), ('bp', p['samples']), ('bh', p['samples']),
                              ('bw', p['elements'])):
                if p[key] and len(p[key]) != want:
                    p['notes'].append(f'{key} has {len(p[key])} values, expected {want} (not realigned)')
            passes.append(p)
            i = j
            continue
        i += 1
    return passes


def group(p):
    if p['result'] != 'NO READ':
        return None
    if any(n.startswith('not counted') for n in p.get('notes', [])):
        return 'car not driving (ignored by firmware)'
    if p.get('white', 0) > BAD_WHITE:
        return 'bad white reference'
    if p.get('max', 0) - p.get('white', 0) < WEAK_CONTRAST:
        return 'weak contrast'
    if p['elements'] != 29:
        return 'wrong element count'
    return 'right count, wrong character'


def char_report(p, expect):
    d = p['bw']
    if len(d) != p['elements'] or len(d) % 10 != 9:
        return
    text, tries = decode(d)
    print(f"    decoded: {text or 'nothing'}  " +
          '  '.join(f"{dirn}={t}" for dirn, t, _ in tries if t))
    want = ('*' + expect + '*') if expect else None
    for direction, t, pats in tries:
        if not t:
            continue
        seq = d if direction == 'forward' else d[::-1]
        print(f"    {direction}:")
        for c, pat in enumerate(pats):
            el = seq[c * 10:c * 10 + 9]
            marks = ''.join('W' if pat & (1 << (8 - k)) else 'n' for k in range(9))
            line = f"      char {c} -> {SYMBOL_OF.get(pat, '?')}  widths(0.1mm) {el}  read {marks}"
            if want and c < len(want):
                ep = PATTERN_OF[want[c]]
                emarks = ''.join('W' if ep & (1 << (8 - k)) else 'n' for k in range(9))
                bad = [k for k in range(9) if marks[k] != emarks[k]]
                line += f"  expect {want[c]} {emarks}" + (f"  WRONG at element {bad}" if bad else '  ok')
            print(line)


def lateral_report(p):
    bv, bp, bh = p['bv'], p['bp'], p['bh']
    n = min(len(bv), len(bp), len(bh))
    if n < 60:
        return
    y, lat = 0.0, []
    for k in range(n):
        if k:
            y += math.sin(bh[k] / 1000.0) * (bp[k] - bp[k - 1])  # axle sideways drift, mm (+ = left)
        lat.append(y + SENSOR_AHEAD_M * 1000.0 * math.sin(bh[k] / 1000.0))
    white = sum(bv[:50]) / 50.0
    level = white + max(60.0, 0.15 * (max(bv[:n]) - white))
    runs, start = [], None
    for k in range(n):
        if bv[k] > level and start is None:
            start = k
        if (bv[k] <= level or k == n - 1) and start is not None:
            peak = max(range(start, k + 1), key=lambda q: bv[q])
            runs.append((start, k, peak))
            start = None
    print(f"    sensor sideways position at each bar (mm, + = left/away from the line? check sign on car); "
          f"heading range {min(bh[:n])}..{max(bh[:n])} mrad, distance {bp[n - 1]} mm")
    for a, b, q in runs:
        print(f"      at {bp[a]:4d}-{bp[b]:4d} mm  width {bp[b] - bp[a] + 1:3d} mm  peak {bv[q]:5d}  "
              f"sideways {lat[q]:+6.1f} mm")
    if len(runs) >= 3:
        xs = [lat[q] for _, _, q in runs]
        ys = [bv[q] for _, _, q in runs]
        mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
        sxy = sum((x - mx) * (y2 - my) for x, y2 in zip(xs, ys))
        sxx = sum((x - mx) ** 2 for x in xs)
        syy = sum((y2 - my) ** 2 for y2 in ys)
        if sxx > 0 and syy > 0:
            print(f"      correlation peak vs sideways position: {sxy / math.sqrt(sxx * syy):+.2f} "
                  f"(near +-1: bar strength follows the swing; near 0: it does not)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('files', nargs='*')
    ap.add_argument('--expect', help='letter on the sheet (e.g. Z), to mark misread elements')
    ap.add_argument('--since', help='only runs from this stamp on, e.g. 20261105-2100')
    ap.add_argument('--fragments', action='store_true', help='also list short no-code stretches')
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()
    if args.selftest:
        sys.exit(0 if selftest() else 1)
    files = args.files or sorted(glob.glob(DEFAULT_GLOB))
    if args.since:
        files = [f for f in files if os.path.basename(f)[4:] >= args.since]
    groups, totals = {}, {'read': 0, 'NO READ': 0, 'fragment': 0}
    failures = []
    for f in files:
        passes = parse(f)
        shown = [p for p in passes if p['result'] != 'fragment' or args.fragments]
        if not shown:
            continue
        print(f"== {os.path.basename(f)}")
        for p in passes:
            kind = 'read' if p['result'].startswith('read') else p['result']
            totals[kind] += 1
            if p['result'] == 'fragment' and not args.fragments:
                continue
            g = group(p)
            contrast = (p['max'] - p['white']) if 'max' in p and 'white' in p else None
            print(f"  line {p['line']:5d} {p['ctx']:18s} {p['result']:9s} white={p.get('white', '-')!s:>5} "
                  f"peak={p.get('max', '-')!s:>5} contrast={contrast if contrast is not None else '-'!s:>5} "
                  f"samples={p.get('samples', '-')!s:>4} elements={p.get('elements', '-')!s:>3}"
                  + (f"  [{g}]" if g else '') + ''.join(f"  ({n})" for n in p.get('notes', [])))
            if g:
                groups[g] = groups.get(g, 0) + 1
                failures.append((f, p, g))
    print(f"\nTotals: {totals['read']} reads, {totals['NO READ']} failed passes, {totals['fragment']} short fragments")
    for g, n in sorted(groups.items(), key=lambda kv: -kv[1]):
        print(f"  {g}: {n}")
    for f, p, g in failures:
        if not (p['bw'] or p['bh']):
            continue
        print(f"\n-- {os.path.basename(f)} line {p['line']} [{g}]")
        if p['bw']:
            char_report(p, args.expect)
        if p['bh']:
            lateral_report(p)


if __name__ == '__main__':
    main()
