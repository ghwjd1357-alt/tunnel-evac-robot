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
1) 따옴표 상태를 문자 단위로 추적해 인용 안쪽을 `q` 로 가린다(출력 문자별 원본 줄번호 보존).
   → `python3 -c '...'` 안의 파이썬 본문이나 `"{pose: ...}"` 가 셸 명령으로 오독되지 않는다.
2) 인용 밖 주석(`#`)을 물리 개행까지 가리고, 셸·큰따옴표 문맥의 줄 이어쓰기(`\` + 개행)는 Bash처럼
   **논리 입력에서 제거**한다. 공백 치환하면 `<\`+개행+`&` 같은 한 연산자가 갈라진다.
3) 인용 밖 구분자(개행 `;` `|` `&` `(` `)` 백틱)로 **단순 명령** 단위로 쪼갠다.
   → `if`/`until`/명령치환 `$(`/프로세스치환 `<(`/파이프/백틱 뒤의 호출도 전부 잡힌다.
4) 각 단위를 **끝까지** 훑어 `ros2`·`gz` 토큰을 찾고, 그 앞에 `hard_timeout` 이 결합돼
   있는지만 본다. wrapper 목록도, 옵션 문법 추정도 하지 않는다 — 어느 쪽이든
   "여기서 끝내도 된다"는 판단이 필요한데, 그 판단이 틀리면 **조용히** 뚫리기 때문이다.

── 알려진 한계 (숨기지 않는다) ─────────────────────────────────────────────
- `eval`·변수로 조립한 명령(`$CMD topic echo`)은 정적으로 못 본다.
- heredoc 본문을 셸로 계속 읽는다. 게이트 5파일엔 heredoc 이 없어 지금은 무해하지만,
  heredoc 이 생기면 그 안의 텍스트가 명령으로 오검출될 수 있다(거짓 양성 = 안전 방향).
- 단위 안에 **인용되지 않은** `ros2`·`gz` 토큰이 있으면 실행 위치가 아니어도 위반으로 본다
  (`echo ros2`·`grep ros2 file`·`xargs echo ros2`). 거짓 양성이지만 fail-closed 이고,
  게이트에서는 인용하거나 쓰지 않으면 된다. 실측상 `tools/` 셸 도구 8개 전량에서 0건이다.
- 큰따옴표 안의 **변수 확장**(`"${BASH_SOURCE[0]}"`·`"$1"`)은 명령치환이 아니므로 가린다.
  여기를 열면 게이트 5파일이 즉시 거짓 양성으로 무너진다 — 의도적으로 열지 않았다.
- 백그라운드(`… &`) 호출은 블록하지 않으므로 제외한다 — 대신 그 프로세스의 수명은
  각 스크립트의 `cleanup` 이 책임진다.
  ⚠ 제외의 근거는 오직 **'블록하지 않는다'** 이다. 그래서 `&` 라는 글자만 보고 빼면 안 된다 —
    `&>`·`&>>` 는 리다이렉션이고 그 명령은 **foreground 로 블록한다**(§14.2 P1). 역할 판정은
    `_amp_role()` 한 곳에서만 한다.
"""
import os
import re
import sys

# ⚠ 예전에 있던 KEYWORDS/PREFIX(wrapper 화이트리스트)는 §13.3 P2 로 **삭제**했다.
#    목록이 있으면 목록 밖 wrapper 로 조용히 뚫린다 — `_check_unit()` 주석 참조.
DURATION = re.compile(r'^[0-9q$]')          # timeout 의 예산 인자(따옴표는 q 로 가려져 있다)
EXTERNAL = {'ros2', 'gz'}
SEPARATORS = set('\n;|&()`')


def _mask(src):
    """셸 문맥은 남기고 **인용 안쪽만** q 로 가린다(출력 문자별 원본 줄번호 보존).

    ★ 07-31 §13.2 P1 (검토): 구판은 큰따옴표 안쪽을 **통째로** 가렸다. 그런데 셸의
      큰따옴표는 명령치환을 막지 않는다 — 그래서 실제로 실행되는 `$( )`·백틱까지 함께
      지워졌고, `y="$(gz model …)"` 가 검사기에서 **사라졌다**. 따옴표 하나 차이로
      상한 없는 `gz model` 이 조용히 통과한 것이다(게이트 5파일은 `"$( )"` 를 이미
      10곳 넘게 쓴다).
      → 문맥을 **스택**으로 추적한다. 큰따옴표 안이라도 명령치환 안쪽은 셸 문맥으로
        되돌리고, 닫히면 다시 인용으로 돌아온다.
      ⚠ 작은따옴표 안은 셸 의미 그대로 **전부** 가린다(치환이 일어나지 않는다).
      ⚠ `${VAR}` 는 변수 확장이지 명령치환이 **아니다** — 계속 가린다. 이걸 같이 열면
        `"${BASH_SOURCE[0]}"` 류가 전부 거짓 양성이 되어 게이트가 즉시 무너진다.
    """
    out, lineno_of = [], []
    line = 1
    # 문맥 스택: 'sq'=작은따옴표 · 'dq'=큰따옴표 · 'sub'=$( ) · 'paren'=( ) · 'bq'=백틱
    # 스택 top 이 sq/dq 가 아니면 '셸 문맥'(명령이 해석되는 자리)이다.
    stack = []
    i, n = 0, len(src)

    def emit(c, masked=False):
        out.append('q' if masked else c)
        lineno_of.append(line)

    def after_continuations(j):
        """j부터 연속된 line continuation을 건너뛴 (인덱스, 개행 수)를 돌려준다."""
        skipped = 0
        while j + 1 < n and src[j] == '\\' and src[j + 1] == '\n':
            j += 2
            skipped += 1
        return j, skipped

    while i < n:
        ch = src[i]
        top = stack[-1] if stack else None

        if top == 'sq':                       # 작은따옴표: 전부 가린다
            if ch == "'":
                stack.pop()
            emit(ch, True)
            if ch == '\n':
                line += 1
            i += 1
            continue

        # ★ 08-01 §16.2 P1: Bash 는 셸·큰따옴표 문맥의 `\`+개행을 토큰화 **전에 제거**한다.
        # 공백 두 칸으로 바꾸면 source 의 물리적 거리가 논리행에 남아 `<&`·`>&`·`&>`·`&&`,
        # 심지어 `ro`+`s2`가 갈라졌다. 여기서는 아무 문자도 emit 하지 않되 `line`만 올려
        # 뒤쪽 토큰의 원본 줄번호를 보존한다. 작은따옴표 안에서는 두 문자 모두 리터럴이다.
        if ch == '\\' and i + 1 < n and src[i + 1] == '\n':
            line += 1
            i += 2
            continue

        if top == 'dq':                       # 큰따옴표: 명령치환만 살린다
            if ch == '\\' and i + 1 < n:
                emit(ch, True); emit(src[i + 1], True)
                if src[i + 1] == '\n':
                    line += 1
                i += 2
                continue
            if ch == '"':
                stack.pop(); emit(ch, True); i += 1
                continue
            j, skipped = after_continuations(i + 1) if ch == '$' else (i + 1, 0)
            if ch == '$' and j < n and src[j] == '(':
                stack.append('sub')
                emit('$')
                line += skipped
                emit('(')                     # 인용 밖과 똑같이 내보내 단위가 쪼개지게 한다
                i = j + 1
                continue
            if ch == '`':
                stack.append('bq'); emit('`'); i += 1
                continue
            emit(ch, True)
            if ch == '\n':
                line += 1
            i += 1
            continue

        # --- 셸 문맥 (최상위 · $( ) · ( ) · 백틱 안) ---
        if ch == '\\' and i + 1 < n:
            emit(ch, True); emit(src[i + 1], True); i += 2
            continue
        if ch == '#' and (not out or out[-1] in ' \t\n;|&()`'):
            while i < n and src[i] != '\n':                    # 주석은 줄 끝까지 지운다
                emit(' '); i += 1
            continue
        if ch == "'":
            stack.append('sq'); emit(ch, True); i += 1
            continue
        if ch == '"':
            stack.append('dq'); emit(ch, True); i += 1
            continue
        if ch == '`':
            stack.pop() if top == 'bq' else stack.append('bq')
            emit('`'); i += 1
            continue
        j, skipped = after_continuations(i + 1) if ch == '$' else (i + 1, 0)
        if ch == '$' and j < n and src[j] == '(':
            stack.append('sub'); emit('$'); line += skipped; emit('('); i = j + 1
            continue
        if ch == '(':
            stack.append('paren'); emit('('); i += 1
            continue
        if ch == ')':
            if top in ('sub', 'paren'):
                stack.pop()
            emit(')'); i += 1
            continue
        emit(ch)
        if ch == '\n':
            line += 1
        i += 1
    return ''.join(out), lineno_of


def _amp_role(text, i, n):
    """인용 밖 `&` **하나**의 역할을 정한다 — 배경 실행 예외의 유일한 책임 경계다.

    ★ 판정 기준은 '아는 표기 목록'이 아니라 **셸 문법 사실 하나**다:
      **리다이렉션 연산자는 하나의 토큰이라 내부에 공백이 낄 수 없다.**
      그래서 `&` 가 리다이렉션의 일부인지는 **바로 붙어 있는 한 글자**로 결정된다.
      전제: `_mask()` 가 셸·큰따옴표 문맥의 line continuation을 먼저 제거해, 여기의 `text`는
      물리 source가 아니라 Bash가 토큰화하는 **논리 입력**이다(08-01 §16.2 P1).
      이 사실을 쓰면 `&` 가 나타나는 자리가 아래 넷으로 **닫힌다**:

      | 인접 조건        | 표기                                    | 역할 |
      |------------------|-----------------------------------------|------|
      | 뒤가 `&`         | `&&`                                    | 목록 연산자 (구분자, 배경 아님) |
      | 뒤가 `>`         | `&>` `&>>`                              | 리다이렉션 → **foreground** |
      | 앞이 `<` 또는 `>`| `<&` `>&` `n<&m` `n>&m` `<&-` `>&-`     | 리다이렉션 → **foreground** |
      | 그 외            | `&`                                     | 제어 연산자 = **배경 실행** |

      단어 안의 리터럴 `&`(인용·이스케이프)는 `_mask()` 가 이미 가려서 여기까지 오지 않는다.

    ★ 07-31 §14.2 P1 (검토): 구판은 앞의 `>`(`2>&1`)만 예외로 뒀고 **뒤의 `>`** 는 안 봤다.
      그래서 `ros2 topic echo /x &>/dev/null` 이 배경으로 분류돼 단위 전체가 검사에서 빠졌다.
    ★ 08-01 §15.2 P1 (검토): 그 보완도 `>` 쪽만 봤다 — **입력 FD 복제 `<&` 를 빠뜨렸다.**
      `ros2 topic echo /cmd_vel <&0` · `gz … 0<&3` · `ros2 … <&-` 는 전부 유효한 foreground
      명령인데(`bash -n` 확인) 배경으로 분류돼 사라졌다. 같은 자리 **여섯 번째** 재개방이다.
      근인은 표기를 하나씩 덧대고 있었던 것이고, 더 정확히는 `prev` 를 찾을 때 **공백을
      건너뛰고 있었던 것**이다 — 리다이렉션 연산자에는 공백이 낄 수 없으므로 그 탐색 자체가
      문법에 어긋났고, 판정이 인접성이 아니라 '가까이 있는 글자'에 기대고 있었다.
      → **인접 한 글자만** 본다. 이제 열거가 아니라 위 표의 네 자리를 닫는 규칙이다.
    ★ 08-01 §16.2 P1 (검토): 그 규칙의 입력이 틀렸다. Bash는 line continuation을 제거하지만
      `_mask()`는 공백으로 바꿔, 논리적으로 인접한 네 연산자와 `ro`+`s2`를 다시 갈랐다.
      역할 표를 더 늘리지 않고 `_mask()`가 Bash 논리 입력을 만들도록 책임을 바로잡았다.

    ⚠ 배경 실행을 빼는 근거는 오직 **'블록하지 않는다'** 이다. 그래서 `&` 라는 글자를 보고
      빼면 안 된다 — 리다이렉션 안의 `&` 가 붙은 명령은 **foreground 로 블록한다.**
    ⚠ 배경 예외 자체를 없애는 안도 검토했으나 **채택하지 않았다**: 실측상 게이트 5파일에
      거짓 양성 **7건**(전부 `nohup ros2 launch … &` — 실제로 블록하지 않는 정상 배선)이
      생긴다. 예외는 정당하고 틀린 것은 판정 정확도였다.
    """
    nxt = text[i + 1] if i + 1 < n else ''
    prv = text[i - 1] if i > 0 else ''
    if nxt == '&':                                 # &&
        return 'andand'
    if nxt == '>':                                 # &>  &>>
        return 'redir'
    if prv in '<>':                                # <&  >&  (n<&m · n>&m · <&- · >&-)
        return 'redir'
    return 'bg'


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
            role = _amp_role(text, i, n)
            if role == 'andand':                   # && 는 구분자일 뿐, 배경실행이 아니다
                i += 1
            elif role == 'redir':                  # 리다이렉션의 & = 구분자가 아니다
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
    """이 단순 명령이 상한 없는 외부 CLI 호출이면 (줄번호, 명령) 을 돌려준다.

    ★ 07-31 §13.3 P2 (검토) — **wrapper 화이트리스트를 없앴다.**
      §12 보완은 `nohup/env/command/exec/builtin/xargs/sudo/stdbuf` 목록 뒤에서만 단위를
      끝까지 훑고, 목록 **밖** 명령을 만나면 즉시 성공 반환했다. 그래서
      `setsid`·`nice -n 5`·`ionice -c2`·`taskset -c 0`·`unbuffer` 뒤의 무상한 `ros2` 가
      전부 통과했다. §11.2 에서 배운 것이 정확히 *"'아는 것만 보는 목록'은 새 항목이 늘 때
      조용히 뚫린다"* 인데, 그 목록이 한 층 아래로 옮겨간 채 공개되지 않았다.
      → 목록을 지우고 **단위 전체를 끝까지 훑는다.** 상한 판정은 `hard_timeout` 결합 하나뿐이다.
      실측(`tools/` 셸 도구 8개 전량): 이 전환으로 늘어난 검출 **0건** — 오늘 비용은 0 이다.
      ⚠ 대가: `echo ros2` · `grep ros2 file` 처럼 **실행이 아닌 언급**도 위반이 된다.
        의도된 fail-closed 다(게이트에서는 인용하거나 쓰지 않는다). 조용히 통과하느니
        시끄럽게 틀리는 쪽을 고른다.
    """
    guarded = False
    i = 0
    while i < len(toks):
        tok, ln = toks[i]
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
            return (ln, ' '.join(t for t, _ in toks[i:i + 3]))
        i += 1
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
