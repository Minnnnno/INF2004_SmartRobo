#!/usr/bin/env python3
"""Check the team's C files against the enforceable BARR-C:2018 rules.

    python tools/barr_check.py            list violations, exit 1 if any
    python tools/barr_check.py --notes    also list the "preferred practice" ones

Text based, so it needs no compiler.  Layout is checked by tools/barr_format.py
--check.  Agreed deviations: docs/CODING_STANDARD.md.
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import barr_format as B  # noqa: E402

MAX_LINES = 125      # 6.2.a, project limit (docs/CODING_STANDARD.md)
LONG_OK = {'manual_console', 'line_drive'}
WIDE_OK = {'include/telemetry_frame.h'}      # parsed line by line by dashboard.py
CONTINUE_OK = {'manual_console', 'barcode_elements', 'route_pivot', 'line_loop', 'comms_task',
               'encoder_capture_events', 'barcode_best_fit'}

found = []
notes = []


def blank(text):
    """Comments and strings replaced by spaces, same length and lines."""
    def r(m):
        return re.sub(r'[^\n]', ' ', m.group())
    return re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])+\'', r, text, flags=re.S)


def line_of(text, pos):
    return text.count('\n', 0, pos) + 1


def add(rule, f, line, what):
    found.append((f, line, rule, what))


def each(rule, f, code, pattern, what, flags=0):
    for m in re.finditer(pattern, code, flags):
        add(rule, f, line_of(code, m.start()), what + ': ' + m.group().strip()[:50])


def check(f):
    raw = open(os.path.join(B.ROOT, f), 'rb').read()
    text = raw.decode('utf-8').replace('\r\n', '\n')
    code = re.sub(r'#include[^\n]*', lambda m: ' ' * len(m.group()), blank(text))
    lines = text.split('\n')
    for n, line in enumerate(lines, 1):
        if '\t' in line:
            add('3.5.a', f, n, 'tab')
        if len(line) > B.WIDTH and f not in WIDE_OK:
            add('1.2.a', f, n, '%d columns' % len(line))
    if B.END not in lines[-3:]:
        add('3.3.c', f, len(lines), 'no end-of-file comment')
    if not text.startswith('/** @file ' + os.path.basename(f)):
        add('2.2.h', f, 1, 'no @file comment')
    if f.endswith('.h') and not re.search(r'#ifndef (\w+)\n#define \1\b', text):
        add('4.2.b', f, 1, 'no include guard')
    each('1.7', f, code, r'\b(auto|register|goto)\b', 'keyword')
    each('5.2.b', f, code, r'\b(short|long)\b', 'keyword')
    each('5.4.b', f, code, r'\b(float|double)\b(?! +float\d\d_t;)(?!\s+float\d\d_t)', 'use float32_t/float64_t')
    each('8.5.b', f, code, r'\b(abort|exit|setjmp|longjmp)\s*\(', 'call')
    each('8.6.a', f, code, r'(?<=[(&|] )?\b[a-z][\w.>\-\[\]]* [=!]= (?:NULL|true|false|\d[\w.]*|[A-Z][A-Z0-9_]{2,})\b(?!\s*[(.])',
         'constant on the right')
    each('8.4.c', f, code, r'while \((?:1|true|TRUE)\)', 'use for (;;)')
    each('8.1.a', f, code, r'^[ ]+(?:static |const )*(?:unsigned|int|bool|char|float\d\d_t|u?int\d+_t) \**\w+(?:\[\w*\])?'
         r'(?: = [\w.]+)?, \**\w+ *[;=,\[]', 'two declarations', re.M)
    each('1.3.a', f, code, r'^ *(?:else )?(?:if|for|while) \([^\n{;]*\) *[^\n{ ;][^\n]*;$', 'no braces', re.M)
    each('1.3.b', f, code, r'^[^\n#=]*\) *\{$', 'brace not on its own line', re.M)
    each('3.3.a', f, code, r'^(?![^\n]*\bfor \()[^\n]*;[ ]*[^\s\n][^\n]*;', 'two statements', re.M)
    each('6.4.a', f, code, r'\(FP\)(?!\w+_task\b)(?=\w+,)\w+', 'task name')
    each('7.1.f', f, code, r'\b(?:unsigned|int|bool|float\d\d_t|u?int\d+_t) \**[a-z]\w*[A-Z]\w*\s*[=;,)]', 'upper case in a name')
    each('7.1.e', f, code, r'\b(?:unsigned|int|bool|float\d\d_t|u?int\d+_t|char) \**(?!(?:kp|ki|kd|id|up)\b)[a-z]\w?(?=\s*[=;,)\[])', 'short name')
    for m in re.finditer(r'/\*(?:[^*]|\*(?!/))*\*/', text):
        body = m.group()[2:-2]
        if '/*' in body:
            add('2.1.b', f, line_of(text, m.start()), 'nested comment')
    for m in re.finditer(r'^(\w+) \([^;{]*\)\n\{\n(.*?)^\}', text, re.S | re.M):
        name, n = m.group(1), m.group(2).count('\n') + 3
        at = line_of(text, m.start())
        if len(name) > 31:
            add('6.1.d', f, at, name)
        if n > MAX_LINES and name not in LONG_OK and not f.endswith('_test.c'):
            add('6.2.a', f, at, '%s: %d lines' % (name, n))
        body = blank(m.group(2))
        if re.search(r'\bcontinue;', body):
            (notes if name in CONTINUE_OK else found).append((f, at, '1.7.d', name + ': continue'))
        if len(re.findall(r'\breturn\b', body)) > 1:
            notes.append((f, at, '6.2.c', '%s: %d exits' % (name, len(re.findall(r'\breturn\b', body)))))
        for s in re.finditer(r'\bswitch \(', body):
            if 'default:' not in body[s.start():]:
                add('8.3.b', f, at, name + ': switch without default')


def main():
    for f in B.team_files():
        check(f)
    for f, n, rule, what in sorted(found):
        print('%s:%d: %s %s' % (f, n, rule, what))
    if '--notes' in sys.argv:
        for f, n, rule, what in sorted(notes):
            print('%s:%d: note %s %s' % (f, n, rule, what))
    print('%d violation(s), %d preferred-practice note(s) in %d files' %
          (len(found), len(notes), len(B.team_files())))
    return 1 if found else 0


if __name__ == '__main__':
    sys.exit(main())
