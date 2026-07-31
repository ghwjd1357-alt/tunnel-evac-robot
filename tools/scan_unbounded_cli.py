#!/usr/bin/env python3
"""게이트 셸 스크립트의 '상한 없는 외부 CLI 호출'을 전수 검출한다 (MASTER_PLAN §7 예약 17).

사용법:  python3 scan_unbounded_cli.py a.sh b.sh ...
출력  :  위반이 있으면 `파일:줄  명령` 을 한 줄씩, 없으면 아무것도 출력하지 않는다.
종료  :  항상 0 (판정은 호출자가 출력 유무로 한다 — 검사기 자체의 오류는 예외로 죽는다).

── 왜 이 파일이 따로 있나 (07-31 검토 §11.2 P1-①) ──────────────────────────────
구판 검사기는 `test_harness_guards.sh` 안의 heredoc 정규식이었고, **하위 명령
화이트리스트**(`daemon|topic|service|param|gz model`)로 판정했다. 그 결과
`regression_negative.sh:44` 의 `timeout "$3" ros2 action send_goal` 을 아예 보지
못한 채 19/19 PASS 했다 — **거짓 녹색**이다. `ros2 lifecycle`·`ros2 action info` 도
같은 사각에 있었다(당시 우연히 상한이 걸려 있어 증상이 없었을 뿐).

교훈: '아는 명령만 본다'는 목록은 새 명령이 늘 때마다 조용히 뚫린다.
→ **모든 foreground `ros2`·`gz` 호출**을 대상으로 뒤집고, 예외(백그라운드)만 명시한다.

또 하나: 구판은 "같은 줄 어딘가에 hard_timeout 문자열이 있으면 통과"였다. 그래서
`hard_timeout 5 ros2 daemon stop; ros2 topic echo ...` 같은 줄이 통과했다.
→ 이제 **명령 단위로 결합**한다. hard_timeout 의 피연산자인 호출만 상한이 있다고 본다.

── 파서가 하는 일 ──────────────────────────────────────────────────────────
1) 따옴표 상태를 문자 단위로 추적해 인용 안쪽을 `q` 로 가린다(길이·줄번호 보존).
   → `python3 -c '...'` 안의 파이썬 본문이나 `"{pose: ...}"` 가 셸 명령으로 오독되지 않는다.
2) 인용 밖 주석(`#`)과 줄 이어쓰기(`\` + 개행)를 처리한다.
3) 인용 밖 구분자(개행 `;` `|` `&` `(` `)` 백틱)로 **단순 명령** 단위로 쪼갠다.
   → `if`/`until`/명령치환 `$(`/프로세스치환 `<(`/파이프/백틱 뒤의 호출도 전부 잡힌다.
4) 각 단위의 앞쪽 토큰을 걸어가며 명령 위치의 `ros2`·`gz` 를 찾고, 그 앞에
   `hard_timeout` 이 붙어 있는지만 본다.
   wrapper(`env`·`xargs`·`time` 등)를 만난 뒤에는 옵션 문법을 추정하지 않고 단위 끝까지
   보수적으로 훑는다. wrapper 옵션은 종류마다 달라 "첫 미지 토큰=다른 명령"으로 끝내면
   `xargs -n1 ros2` 같은 실제 호출이 조용히 빠지기 때문이다.

── 알려진 한계 (숨기지 않는다) ─────────────────────────────────────────────
- `eval`·변수로 조립한 명령(`$CMD topic echo`)은 정적으로 못 본다.
- heredoc 본문을 셸로 계속 읽는다. 게이트 5파일엔 heredoc 이 없어 지금은 무해하지만,
  heredoc 이 생기면 그 안의 텍스트가 명령으로 오검출될 수 있다(거짓 양성 = 안전 방향).
- wrapper 뒤에 나온 unquoted `ros2`·`gz` 토큰은 실제 실행 위치가 모호해도 위반으로 본다.
  `xargs echo ros2` 같은 형상은 거짓 양성이지만, 게이트에서는 그런 간접 표기를 금지하고
  `hard_timeout N ...` 아래의 명시 호출만 허용한다(fail-closed).
- 백그라운드(`… &`) 호출은 블록하지 않으므로 제외한다 — 대신 그 프로세스의 수명은
  각 스크립트의 `cleanup` 이 책임진다.
"""
import os
import re
import sys

# 명령 앞에 붙어도 '명령이 아직 안 나왔다'로 보는 것들
KEYWORDS = {
    'if', 'then', 'else', 'elif', 'fi', 'while', 'until', 'do', 'done',
    'case', 'esac', 'for', 'select', 'function', 'in', '!', '{', '}', 'time',
}
PREFIX = {'nohup', 'env', 'command', 'exec', 'builtin', 'xargs', 'sudo', 'stdbuf'}
ASSIGN = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*=')
DURATION = re.compile(r'^[0-9q$]')          # timeout 의 예산 인자(따옴표는 q 로 가려져 있다)
EXTERNAL = {'ros2', 'gz'}
SEPARATORS = set('\n;|&()`')


def _mask(src):
    """인용 안쪽을 q 로 가린 문자열과, 각 문자의 줄번호 배열을 돌려준다."""
    out, lineno_of = [], []
    line = 1
    state = None            # None / "'" / '"'
    i, n = 0, len(src)
    while i < n:
        ch = src[i]
        if state is None:
            # 줄 이어쓰기: 인용 밖의 \ + 개행은 사라진다(한 논리행으로 이어짐)
            if ch == '\\' and i + 1 < n and src[i + 1] == '\n':
                out.append(' '); lineno_of.append(line)
                out.append(' '); lineno_of.append(line)
                line += 1; i += 2
                continue
            if ch == '\\' and i + 1 < n:
                out.append('q'); lineno_of.append(line)
                out.append('q'); lineno_of.append(line)
                i += 2
                continue
            if ch == '#' and (not out or out[-1] in ' \t\n;|&()`'):
                while i < n and src[i] != '\n':      # 주석은 줄 끝까지 지운다
                    out.append(' '); lineno_of.append(line); i += 1
                continue
            if ch in ("'", '"'):
                state = ch
                out.append('q'); lineno_of.append(line); i += 1
                continue
            out.append(ch); lineno_of.append(line)
            if ch == '\n':
                line += 1
            i += 1
            continue
        # 인용 안: 개행까지 포함해 전부 가린다(인용 안 개행은 명령을 끊지 않는다)
        if state == '"' and ch == '\\' and i + 1 < n:
            out.append('q'); lineno_of.append(line)
            out.append('q'); lineno_of.append(line)
            if src[i + 1] == '\n':
                line += 1
            i += 2
            continue
        if ch == state:
            state = None
        out.append('q'); lineno_of.append(line)
        if ch == '\n':
            line += 1
        i += 1
    return ''.join(out), lineno_of


def _units(text, lineno_of):
    """단순 명령 단위로 쪼갠다 → [(토큰들, 배경실행여부)]. 토큰 = (문자열, 줄번호)."""
    units, buf, i, n = [], [], 0, len(text)
    while i < n:
        ch = text[i]
        if ch not in SEPARATORS:
            buf.append(i); i += 1
            continue
        background = False
        if ch == '&':
            prev = next((text[j] for j in range(i - 1, -1, -1) if text[j] not in ' \t'), '')
            if i + 1 < n and text[i + 1] == '&':      # && 는 구분자일 뿐, 배경실행이 아니다
                i += 1
            elif prev == '>':                          # 2>&1 의 & 는 구분자가 아니다
                buf.append(i); i += 1
                continue
            else:
                background = True
        units.append((_tokens(text, lineno_of, buf), background))
        buf = []
        i += 1
    if buf:
        units.append((_tokens(text, lineno_of, buf), False))
    return units


def _tokens(text, lineno_of, idxs):
    toks, cur, start = [], [], None
    for k in idxs:
        if text[k] in ' \t':
            if cur:
                toks.append((''.join(cur), lineno_of[start])); cur, start = [], None
            continue
        if start is None:
            start = k
        cur.append(text[k])
    if cur:
        toks.append((''.join(cur), lineno_of[start]))
    return toks


def _check_unit(toks):
    """이 단순 명령이 상한 없는 외부 CLI 호출이면 (줄번호, 명령) 을 돌려준다."""
    guarded = False
    wrapped = False
    i = 0
    while i < len(toks):
        tok, ln = toks[i]
        if tok in KEYWORDS:
            # `time`만 뒤의 명령을 실행하는 wrapper다. if/until 같은 셸 키워드는
            # 명령 위치를 열어 줄 뿐이므로 wrapped로 보지 않는다.
            if tok == 'time':
                wrapped = True
            i += 1
            continue
        if tok in PREFIX:
            wrapped = True
            i += 1
            continue
        if ASSIGN.match(tok):
            i += 1
            continue
        if tok == 'hard_timeout':
            guarded = True
            i += 2                                  # 다음 토큰 = 예산
            continue
        if tok == 'timeout':                        # 일반 timeout 은 TERM 무시를 못 죽인다
            guarded = False
            i += 1
            while i < len(toks) and (toks[i][0].startswith('-') or DURATION.match(toks[i][0])):
                i += 1
            continue
        if tok in EXTERNAL:
            if guarded:
                return None
            rest = ' '.join(t for t, _ in toks[i:i + 3])
            return (ln, rest)
        if wrapped:
            # ★ §12 P1: wrapper의 옵션/옵션 인자를 "다른 명령"으로 오인해 성공 반환하지
            # 않는다. wrapper별 문법을 불완전하게 재구현하는 대신 단위 끝까지 보수적으로
            # 훑어, 뒤의 ros2/gz가 hard_timeout 밖이면 반드시 잡는다.
            i += 1
            continue
        return None                                 # 다른 명령 — 뒤의 ros2/gz 는 그 인자다
    return None


def scan(path):
    text, lineno_of = _mask(open(path, encoding='utf-8').read())
    hits = []
    for toks, background in _units(text, lineno_of):
        if background:                              # 백그라운드 기동은 블록하지 않는다
            continue
        hit = _check_unit(toks)
        if hit:
            hits.append(hit)
    return hits


def main(argv):
    bad = []
    for path in argv:
        for ln, cmd in scan(path):
            bad.append('%s:%d  %s' % (os.path.basename(path), ln, cmd))
    if bad:
        print('\n'.join(bad))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
