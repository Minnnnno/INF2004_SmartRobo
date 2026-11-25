#!/usr/bin/env python3
"""Lay out the team's C files as BARR-C:2018 asks (docs/CODING_STANDARD.md).

    python tools/barr_format.py            format every team file in place
    python tools/barr_format.py --check    only report files that would change
    python tools/barr_format.py FILE...    format these files

clang-format does most of it (app_program/.clang-format).  This script finds
clang-format, then adds what it cannot do:
  1.2/2.2  a trailing comment that does not fit in 80 columns moves above
  3.3.b    a blank line before and after each block
  3.3.c    an end-of-file comment
  3.5/3.6  no tabs, LF line endings
  8.3.a    "break" lined up with its "case"
Layout only: the compiled code does not change.
"""
import glob
import os
import re
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STYLE = os.path.join(ROOT, 'app_program', '.clang-format')
END = '/*** end of file ***/'
WIDTH = 80


def find_clang_format():
    places = [os.environ.get('CLANG_FORMAT'), shutil.which('clang-format')]
    places += glob.glob('C:/Program Files/Microsoft Visual Studio/*/*/VC/Tools/'
                        'Llvm/x64/bin/clang-format.exe')
    places += glob.glob(os.path.expanduser(
        '~/.vscode/extensions/ms-vscode.cpptools-*/LLVM/bin/clang-format*'))
    for place in places:
        if place and os.path.exists(place):
            return place
    sys.exit('clang-format not found: set CLANG_FORMAT (version 20 or later)')


def team_files():
    out = subprocess.run(['git', 'ls-files', 'app_program', 'include'], cwd=ROOT,
                         capture_output=True, text=True, check=True).stdout
    return [f for f in out.split('\n') if f.endswith(('.c', '.h')) and
            (f.startswith('app_program/') or f.count('/') == 1) and
            f != 'include/basic.h']


CLANG = None


def clang(text, name, limit=WIDTH):
    style = open(STYLE, encoding='utf-8').read()
    if limit != WIDTH:
        style = style.replace('ColumnLimit: %d' % WIDTH, 'ColumnLimit: %d' % limit)
    style = '\n'.join(l for l in style.split('\n') if not l.startswith('#'))
    tmp = os.path.join(os.environ.get('TEMP', '/tmp'), 'barr_style_%d_%d.txt' %
                       (os.getpid(), limit))
    with open(tmp, 'w', encoding='utf-8', newline='\n') as f:
        f.write(style)
    r = subprocess.run([CLANG, '--style=file:' + tmp, '--assume-filename=' + name],
                       input=text.encode('utf-8'), capture_output=True)
    os.remove(tmp)
    if r.returncode:
        sys.exit(name + ': ' + r.stderr.decode('utf-8', 'replace'))
    return r.stdout.decode('utf-8').replace('\r\n', '\n')


def split_trailing(line):
    """(code, comment) when a block or line comment follows code, else None."""
    i, n, quote = 0, len(line), None
    while i < n:
        c = line[i]
        if quote:
            if c == '\\':
                i += 1
            elif c == quote:
                quote = None
        elif c in '"\'':
            quote = c
        elif line.startswith('/*', i) or line.startswith('//', i):
            code = line[:i].rstrip()
            return (code, line[i:].rstrip()) if code.strip() else None
        i += 1
    return None


def lift_comments(text, macros_only=False):
    """Put trailing comments that do not fit on lines of their own, above."""
    lines = text.split('\n')
    out = []
    i = 0
    in_block = False
    while i < len(lines):
        line = lines[i]
        if in_block:
            out.append(line)
            in_block = '*/' not in line
            i += 1
            continue
        got = split_trailing(line)
        if got is None:
            stripped = line.strip()
            if stripped.startswith('/*') and '*/' not in stripped:
                in_block = True
            out.append(line)
            i += 1
            continue
        code, comment = got
        j = i
        parts = [comment]
        if comment.startswith('/*') and '*/' not in comment:
            while '*/' not in parts[-1] and j + 1 < len(lines):
                j += 1
                parts.append(lines[j].strip())
        closed = parts[-1].endswith('*/') or comment.startswith('//')
        macro = code.lstrip().startswith('#')
        continued = line.rstrip().endswith('\\') or (
            out and out[-1].rstrip().endswith('\\'))
        too_long = (j > i or len(line) > WIDTH) and (macro or not macros_only)
        if macro and re.match(r'\s*#\s*(endif|else)\b', code):
            too_long = False
        if too_long and closed and not continued:
            indent = re.match(r'\s*', code).group()
            words = ' '.join(re.sub(r'^\*\s+', '', p) for p in parts)
            # Keep "else" attached to its "if" when the comment is on it.
            out.append(indent + words)
            out.append(code)
        else:
            out.extend(lines[i:j + 1])
        i = j + 1
    return '\n'.join(out)


BLOCK = re.compile(r'\s*(if|for|while|do|switch)\b(?!.*;\s*$)')


def is_comment(line):
    s = line.strip()
    return s.startswith(('/*', '*', '//')) or s.endswith('*/') and '/*' not in s


def comment_map(lines):
    """True for lines that are wholly comment."""
    flags = []
    inside = False
    for line in lines:
        s = line.strip()
        if inside:
            flags.append(True)
            if '*/' in s:
                inside = not s.endswith('*/') and False
            continue
        if s.startswith('//') or (s.startswith('/*') and
                                  (s.endswith('*/') or '*/' not in s)):
            flags.append(True)
            inside = s.startswith('/*') and '*/' not in s
        else:
            flags.append(False)
            got = split_trailing(line)
            if got and got[1].startswith('/*') and '*/' not in got[1]:
                inside = True
    return flags


def align_break(lines):
    stack = []
    out = []
    flags = comment_map(lines)
    for k, line in enumerate(lines):
        s = line.strip()
        if flags[k] or s.startswith('#'):
            out.append(line)
            continue
        indent = len(line) - len(line.lstrip())
        if s == '{':
            head = ''
            for prev in reversed(out):
                if prev.strip() and len(prev) - len(prev.lstrip()) == indent:
                    head = prev.strip()
                    break
            stack.append((indent, head.startswith('switch')))
        elif s.startswith('}') and stack:
            stack.pop()
        if s.startswith('break;') and stack and stack[-1][1]:
            line = ' ' * (stack[-1][0] + 4) + s
        out.append(line)
    return out


def blank_lines(lines):
    flags = comment_map(lines)
    out = []
    n = len(lines)

    def is_open(s):
        return s == '{' or s.endswith(':') and re.match(r'(case\b.*|default|\w+):$', s)

    for k, line in enumerate(lines):
        s = line.strip()
        prev = next((p for p in reversed(out)), '')
        # Before a block (its own comment goes with it).
        start = k
        if flags[k] and (k == 0 or not flags[k - 1]):
            m = k
            while m < n and flags[m]:
                m += 1
            start = m
        if start < n and not (flags[k] and k and flags[k - 1]):
            target = lines[start]
            t = target.strip()
            block = BLOCK.match(target) and not t.startswith('while (') or (
                t.startswith(('while (', 'for (')) and not t.endswith(';'))
            p = prev.strip()
            if (block and p and not is_open(p) and not p.startswith('#') and
                    p != 'else' and not p.endswith('\\') and
                    not (p == '}' and t.startswith('while') and t.endswith(';')) and
                    not comment_map(out)[-1]):
                out.append('')
        out.append(line)
        # After a block.
        if s == '}' and k + 1 < n:
            nx = lines[k + 1].strip()
            if (nx and not nx.startswith(('}', 'else', '#', 'while (')) and
                    not nx.startswith('break;')):
                out.append('')
        # After the case's break.
        if s.startswith('break;') and k + 1 < n:
            nx = lines[k + 1].strip()
            if nx.startswith(('case ', 'default:', '/*')):
                out.append('')
    return out


DECL = re.compile(r'\s+(?:(?:static|const|volatile|unsigned)\s+)*'
                  r'(?:unsigned|int|bool|char|double|float|[a-z]\w*_t|[A-Z][A-Z0-9_]+)'
                  r'(?:\s+const)?\s+(?:\*+\s*(?:const\s+)?)?\w+(?:\[[^\]]*\])*\s*(?:=|;|$)')


def declaration_gaps(lines):
    """A blank line after each group of local declarations (3.3.b)."""
    flags = comment_map(lines)
    out = []
    k, n = 0, len(lines)
    while k < n:
        line = lines[k]
        if flags[k] or not DECL.match(line):
            out.append(line)
            k += 1
            continue
        # The whole declaration, however many lines it takes.
        while k < n:
            out.append(lines[k])
            code = (split_trailing(lines[k]) or (lines[k], ''))[0].rstrip()
            k += 1
            if code.endswith(';'):
                break
        m = k
        while m < n and flags[m]:
            m += 1
        if (m < n and lines[k].strip() and lines[m].strip() and
                not DECL.match(lines[m]) and
                not lines[m].strip().startswith(('}', '#'))):
            out.append('')
    return out


def type_gaps(lines):
    """A blank line around each type definition at file scope."""
    out = []
    flags = comment_map(lines)
    for k, line in enumerate(lines):
        starts = line.startswith('typedef') or (
            flags[k] and line.startswith('/*') and (k == 0 or not flags[k - 1]) and
            next((l for l, f in zip(lines[k:], flags[k:]) if not f), '')
            .startswith('typedef'))
        if (starts and out and out[-1].strip() and not flags[k - 1] and
                not out[-1].startswith(('#', 'typedef'))):
            out.append('')
        out.append(line)
        if re.match(r'} \w+;', line) and k + 1 < len(lines) and lines[k + 1].strip():
            out.append('')
    return out


def layout(lines):
    """Run the passes on the code, leaving multi-line macros exactly as they are."""
    kept = {}
    plain = []
    macro = []
    for line in lines:
        if macro or (line.lstrip().startswith('#') and line.endswith('\\')):
            macro.append(line)
            if not line.endswith('\\'):
                key = '#pragma BARR_MACRO_%d' % len(kept)
                kept[key] = macro
                plain.append(key)
                macro = []
            continue
        plain.append(line)
    plain = align_break(plain)
    plain = type_gaps(declaration_gaps(blank_lines(plain)))
    out = []
    for line in plain:
        out.extend(kept.get(line, [line]))
    return out


def finish(text):
    lines = [l.rstrip() for l in text.split('\n')]
    while lines and not lines[-1]:
        lines.pop()
    if lines and lines[-1].strip() == END:
        lines.pop()
        while lines and not lines[-1]:
            lines.pop()
    lines = layout(lines)
    out = []
    for line in lines:
        if not line and out and not out[-1]:
            continue
        out.append(line)
    return '\n'.join(out) + '\n\n' + END + '\n'


def fmt(text, name):
    text = text.replace('\r\n', '\n').expandtabs(4)
    text = lift_comments(text, macros_only=True)
    text = clang(text, name, 200)
    text = lift_comments(text)
    for _ in range(6):
        text = clang(text, name)
        lifted = lift_comments(text)
        if lifted == text:
            break
        text = lifted
    done = finish(text)
    for _ in range(4):
        again = finish(clang(done, name))
        if again == done:
            break
        done = again
    return done


def main():
    global CLANG
    CLANG = find_clang_format()
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    check = '--check' in sys.argv
    changed = 0
    for f in args or team_files():
        path = os.path.join(ROOT, f)
        raw = open(path, 'rb').read().decode('utf-8')
        new = fmt(raw, f)
        if new != raw.replace('\r\n', '\n'):
            changed += 1
            if check:
                print('would change', f)
            else:
                with open(path, 'w', encoding='utf-8', newline='\n') as out:
                    out.write(new)
    print('%d file(s) %s' % (changed, 'would change' if check else 'changed'))
    return 1 if check and changed else 0


if __name__ == '__main__':
    sys.exit(main())
