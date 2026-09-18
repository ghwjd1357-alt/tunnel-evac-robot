#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_wiring.py — 관제 화면의 배선 정합 검사 (2026-09-02)

브라우저 없이 잡을 수 있는 오류를 잡는다. 이 셋이 실제로 제일 자주 난다:
  ① JS 가 찾는 id 가 HTML 에 없다        → 화면이 조용히 안 갱신된다
  ② import 한 이름이 export 되지 않았다  → 모듈 전체가 안 뜬다
  ③ 정의 안 된 CSS 변수                  → 색이 통째로 빠진다
  ④ 화면 문자열에 이모지                 → 설계 규칙(ISA-101 · tokens.css) 위반
     🔴 주석의 이모지(🔴·⚠)는 저장소 문서 규약이므로 대상이 아니다.
        검사 대상은 **사람 눈에 보이는 문자열**뿐이다.

실행:  python3 console/test/check_wiring.py
"""
import re, sys, pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
html = (ROOT / 'index.html').read_text()
ids_html = set(re.findall(r'id="([^"]+)"', html))
js_files = sorted((ROOT / 'js').glob('*.js'))
js_all = {f.name: f.read_text() for f in js_files}

# ── ① id ────────────────────────────────────────────────────────────
missing = []
for name, src in js_all.items():
    for m in re.finditer(r"getElementById\(['\"]([^'\"]+)['\"]\)", src):
        if m.group(1) not in ids_html:
            missing.append((name, m.group(1)))
    # 뒤에 , 나 ) 가 와야 진짜 id 다. `set('n-' + k, ...)` 같은 결합은 동적이므로 건너뛴다.
    for m in re.finditer(r"\b(?:txt|set|stat)\(['\"]([a-z0-9-]+)['\"]\s*[,)]", src):
        if m.group(1) not in ids_html:
            missing.append((name, m.group(1)))

# ── ② import / export ───────────────────────────────────────────────
exp = {n: set(re.findall(r'export\s+(?:async\s+)?(?:function|const|let|class)\s+([A-Za-z0-9_]+)', s))
       for n, s in js_all.items()}
bad_imports = []
for name, src in js_all.items():
    for m in re.finditer(r"import\s*\{([^}]+)\}\s*from\s*['\"]\./([^'\"]+)['\"]", src):
        tgt = m.group(2)
        if tgt not in exp:
            bad_imports.append((name, tgt, '파일 없음')); continue
        for n in [x.strip().split(' as ')[0] for x in m.group(1).split(',') if x.strip()]:
            if n not in exp[tgt]:
                bad_imports.append((name, f'{tgt}:{n}', 'export 안 됨'))

# ── ③ CSS 변수 ──────────────────────────────────────────────────────
defined = set(re.findall(r'(--[a-z0-9-]+)\s*:', (ROOT / 'css' / 'tokens.css').read_text()))
used = set(re.findall(r'var\((--[a-z0-9-]+)\)', (ROOT / 'css' / 'console.css').read_text() + html))
undef = sorted(used - defined)

# ── ④ 이모지 — '화면에 나가는 문자열'만 본다 ────────────────────────
EMOJI = re.compile('[\U0001F300-\U0001FAFF☀-➿️]')

def strip_comments(src):
    src = re.sub(r'/\*.*?\*/', '', src, flags=re.S)      # 블록 주석
    return re.sub(r'(?m)//.*$', '', src)                  # 줄 주석

emoji_hits = []
for name, src in js_all.items():
    for m in re.finditer(r"""(['"`])((?:\\.|(?!\1).)*)\1""", strip_comments(src)):
        if EMOJI.search(m.group(2)):
            emoji_hits.append((name, m.group(2)[:60]))
# HTML 은 태그 밖 텍스트만
for chunk in re.split(r'<[^>]+>', re.sub(r'<!--.*?-->', '', html, flags=re.S)):
    if EMOJI.search(chunk):
        emoji_hits.append(('index.html', chunk.strip()[:60]))

def report(title, rows, fmt):
    print(f'{"🔴" if rows else "🟢"} {title}: {len(rows)}건')
    for r in rows[:12]:
        print('    ', fmt(r))

print(f'\n{"="*70}\n  관제 배선 검사\n{"="*70}')
report('HTML 에 없는 id 를 JS 가 찾는다', missing, lambda r: f'{r[0]}  →  #{r[1]}')
report('export 되지 않은 이름을 import',  bad_imports, lambda r: f'{r[0]}  →  {r[1]}  ({r[2]})')
report('정의되지 않은 CSS 변수',          undef, lambda r: r)
report('화면 문자열의 이모지',            emoji_hits, lambda r: f'{r[0]}  "{r[1]}"')

n = len(missing) + len(bad_imports) + len(undef) + len(emoji_hits)
print(f'{"="*70}\n{"🔴 총 %d건" % n if n else "🟢 전부 통과"}\n')
sys.exit(1 if n else 0)
