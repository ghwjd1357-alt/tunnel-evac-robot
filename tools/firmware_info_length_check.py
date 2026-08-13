#!/usr/bin/env python3
"""`/firmware/info` 문자열이 버퍼를 넘는지 **재서** 판정한다 (검토 §65.5).

왜 이 도구가 있나
-----------------
`publishFirmwareInfo()` 는 `snprintf` 로 한 줄을 만든다. `snprintf` 는 버퍼를 넘으면
**조용히 자르고** 잘린 문자열을 성공한 것처럼 남긴다. 그러면 `/firmware/info` 를 읽는
모든 도구(`d0_check` 등)가 뒤가 없어진 줄을 정상 표본으로 받는다.

08-13 에 나는 "최악 845자" 라고 손으로 세어 기록했다. 검토 §65.5 가 38개 인자를 전수해
813/890/925 로 정정했다. **손으로 세면 틀린다** — 그래서 도구로 만든다.

왜 `.ino` 에서 뽑아 쓰나
-----------------------
format 문자열을 이 파일에 복사해 두면, `.ino` 가 바뀌었는데 여기가 안 바뀌는 순간부터
이 검사는 "옛 계약이 안전하다"만 되풀이한다. 검토 §65.3 이 `test_drive_checks` 를
"구 상수를 정답으로 고정한 자기확인" 이라고 부른 것과 같은 함정이다.
그래서 매번 `.ino` 를 파싱해 format 과 버퍼 크기를 **거기서** 가져온다.

하는 일
-------
1. `.ino` 에서 `char infoBuffer[N]` 의 N 과 뒤따르는 `snprintf` format 문자열을 뽑는다.
2. 변환지시자와 인자를 **순서대로 짝지어** 타입별 최악 인자를 만든다.
   - `%s`  : 그 자리에 실제로 들어가는 `.ino` 의 문자열 상수 길이. 못 풀면 보수값 32자.
             (모든 `%s` 에 최장 상수를 먹이면 실제보다 500자 부풀어 쓸모가 없다)
   - `%d`  : `INT_MIN` — 이건 **타입 경계**다. int 로 이보다 긴 출력은 없다.
   - `%lu` : 4294967295 — 이것도 타입 경계 (펌웨어 카운터는 전부 uint32).
   - `%f`  : 🔴 그 자리에 실제로 들어가는 `.ino` 의 **상수 값**을 그대로 넣는다.
             초판은 `-999999.999999` 를 넣고 결과를 "타입 이론 최악"이라 불렀다 —
             검토 §66.3 이 그건 타입 경계가 아니라 임의로 고른 시나리오라고 지적했다.
             지금은 상수를 못 풀 때만 그 시나리오로 물러서고, **그 사실을 표시한다.**
3. 그 인자로 host `gcc` 를 돌려 실제 길이를 잰다. 추정하지 않는다.
4. 최악 길이 >= N 이면 **FAIL**. 여유도 함께 찍는다.
5. 🔴 **변이 주입 자가검사** — format 을 늘려 버퍼를 넘기면 이 도구가 반드시 FAIL
   해야 한다. 안 그러면 "안 넘는다"는 판정에 근거가 없다 (검토 §66.3).

사용
----
    python3 tools/firmware_info_length_check.py
    echo $?      # 0 = 통과

정본 = docs/MASTER_PLAN.md §7 예약 32-d · 검토현황 §65.5.
"""

import os
import re
import subprocess
import sys
import tempfile

INO = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "firmware",
    "teensy_integrated_base_v1_4",
    "teensy_integrated_base_v1_4.ino",
)

# 한 줄 주석은 format 조각 사이에 끼어 있다. C 는 그걸 무시하고 인접 문자열만 잇는다.
COMMENT = re.compile(r"//[^\n]*")
# "..." 조각. 이스케이프는 이 format 에 안 쓰이지만 있어도 통째로 넘어간다.
PIECE = re.compile(r'"((?:[^"\\]|\\.)*)"')
# 변환지시자. 이 펌웨어가 쓰는 것만 받는다 — 새 종류가 생기면 여기서 걸려 멈춘다.
SPEC = re.compile(r"%(?:\.\d+)?(?:lu|d|s|f)")


def fail(message):
    print("  \033[31mNG\033[0m  " + message)
    sys.exit(1)


def read_ino():
    with open(INO, encoding="utf-8") as handle:
        return handle.read()


def split_top_level(text):
    """괄호·대괄호 안의 쉼표는 건너뛰고 최상위 쉼표로만 자른다."""
    parts = []
    depth = 0
    current = []
    for ch in text:
        if ch in "([<":
            depth += 1
        elif ch in ")]>":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append("".join(current).strip())
            current = []
            continue
        current.append(ch)
    tail = "".join(current).strip()
    if tail:
        parts.append(tail)
    return [p for p in parts if p]


# 컴파일러가 채우는 것들. 길이가 규격으로 고정돼 있다.
BUILTIN_LEN = {
    "__DATE__": 11,   # "Aug 13 2026"
    "__TIME__": 8,    # "18:42:07"
}
# STRINGIFY(ARDUINO) 같은 매크로 전개는 소스에서 못 읽는다. 넉넉히 잡는다.
UNRESOLVED_LEN = 32


def string_length_of(arg, source):
    """`%s` 인자 하나가 만들 최대 문자 수."""
    if arg in BUILTIN_LEN:
        return BUILTIN_LEN[arg], arg

    # `cond ? "PID" : "PI"` — 두 갈래 중 긴 쪽.
    literals = PIECE.findall(arg)
    if literals:
        return max(len(x) for x in literals), arg

    # `static const char NAME[] = "..." "...";` 를 소스에서 찾아 길이를 잰다.
    decl = re.search(
        r"static\s+const\s+char\s+" + re.escape(arg) +
        r"\s*\[\s*\]\s*=\s*((?:\s*\"(?:[^\"\\]|\\.)*\")+)\s*;",
        source)
    if decl:
        return sum(len(x) for x in PIECE.findall(decl.group(1))), arg

    return UNRESOLVED_LEN, arg + " (미해결 → %d자 가정)" % UNRESOLVED_LEN


def extract(source):
    """버퍼 크기 · format 문자열 · %s 에 들어갈 가장 긴 문자열을 뽑는다."""
    buffer_match = re.search(r"char\s+infoBuffer\[(\d+)\]", source)
    if not buffer_match:
        fail("`char infoBuffer[N]` 을 못 찾았다 — .ino 구조가 바뀌었다")
    buffer_size = int(buffer_match.group(1))

    # 버퍼 선언 뒤부터 snprintf 인자 끝(`FW_LIBRARY_LIST);`)까지가 검사 범위다.
    tail = source[buffer_match.end():]
    end = tail.find("FW_LIBRARY_LIST);")
    if end < 0:
        fail("snprintf 인자 끝을 못 찾았다 — .ino 구조가 바뀌었다")
    block = tail[:end]

    # snprintf 의 두 번째 인자(sizeof) 뒤부터가 format 이다.
    size_of = block.find("sizeof(infoBuffer)")
    if size_of < 0:
        fail("snprintf 가 sizeof(infoBuffer) 를 안 쓴다 — 상한이 버퍼와 안 묶였다")
    fmt_region = COMMENT.sub("", block[size_of:])

    # 🔴 format 은 **연속한** 문자열 조각까지다. 인자 목록 안에도 문자열 리터럴이 있어서
    #    (`cond ? "PID" : "PI"`) 그냥 전부 긁으면 인자가 format 으로 빨려 들어간다.
    #    조각 사이가 공백뿐일 동안만 잇는다 — C 의 인접 문자열 이어붙이기 규칙 그대로다.
    pieces = list(PIECE.finditer(fmt_region))
    if not pieces:
        fail("format 문자열을 못 찾았다 — .ino 구조가 바뀌었다")
    fmt = pieces[0].group(1)
    fmt_end = pieces[0].end()
    for piece in pieces[1:]:
        if fmt_region[fmt_end:piece.start()].strip():
            break
        fmt += piece.group(1)
        fmt_end = piece.end()

    if "version=%s" not in fmt or not fmt.endswith("libraries=%s"):
        fail("format 을 못 읽었다 — 조각 이어붙이기가 깨졌다")

    arg_region = fmt_region[fmt_end:].lstrip()
    if not arg_region.startswith(","):
        fail("format 뒤에 인자 목록이 안 온다 — .ino 구조가 바뀌었다")
    args = split_top_level(arg_region[1:]) + ["FW_LIBRARY_LIST"]

    return buffer_size, fmt, args


def build_args(fmt, ino_args, source):
    """변환지시자와 인자를 짝지어 최악 인자를 만든다."""
    specs = SPEC.findall(fmt)
    if len(specs) != len(ino_args):
        fail("변환지시자 %d 개 vs 인자 %d 개 — 짝이 안 맞는다 (그 자체가 버그다)"
             % (len(specs), len(ino_args)))

    built = []
    notes = []
    for spec, arg in zip(specs, ino_args):
        if spec.endswith("s"):
            length, note = string_length_of(arg, source)
            built.append('"%s"' % ("X" * length))
            notes.append("    %%s ← %-28s %3d 자" % (note, length))
        elif spec.endswith("lu"):
            built.append("4294967295UL")  # 펌웨어 카운터는 전부 uint32
        elif spec.endswith("d"):
            built.append("INT_MIN")       # 부호 포함 11자 = %d **타입 경계**
        elif spec.endswith("f"):
            value, note = double_value_of(arg, source)
            built.append(value)
            notes.append("    %-6s ← %-28s %s" % (spec, note, value))
        else:
            fail("모르는 변환지시자 %s — 도구를 먼저 고쳐라" % spec)
    return built, notes


#: `%f` 가 상수를 못 풀 때 물러설 자리. 🔴 타입 경계가 아니라 **검사 시나리오**다.
FALLBACK_DOUBLE = "-999999.999999"


def double_value_of(arg, source):
    """`%f` 자리에 실제로 들어가는 `.ino` 상수 값을 찾아 그대로 쓴다 (검토 §66.3).

    이 format 의 `%f` 는 전부 `static const double` 이다 — 반지름·윤거·게인. 값이
    소스에 적혀 있는데 `-999999.999999` 를 넣고 "타입 이론 최악"이라 부르면, 실제로는
    일어날 수 없는 길이를 근거로 버퍼를 판정하는 것이다. 값을 못 풀 때만 물러선다.
    """
    name = arg.strip()
    match = re.search(
        r"static\s+const\s+double\s+%s\s*=\s*([0-9.eE+-]+)\s*;" % re.escape(name),
        source)
    if match is not None:
        return match.group(1), "%s (소스 상수)" % name
    return FALLBACK_DOUBLE, "%s 🔴 상수 미해결 — 시나리오값" % name


def measure(fmt, args):
    program = (
        "#include <stdio.h>\n"
        "#include <limits.h>\n"
        "static char buf[65536];\n"
        "int main(void){\n"
        '  printf("%d\\n", snprintf(buf, sizeof(buf), "'
        + fmt.replace("\\", "\\\\").replace('"', '\\"')
        + '", '
        + ", ".join(args)
        + "));\n"
        "  return 0;\n"
        "}\n"
    )
    with tempfile.TemporaryDirectory() as work:
        csrc = os.path.join(work, "probe.c")
        cbin = os.path.join(work, "probe")
        with open(csrc, "w", encoding="utf-8") as handle:
            handle.write(program)
        build = subprocess.run(
            ["gcc", "-O0", "-w", "-o", cbin, csrc],
            capture_output=True, text=True)
        if build.returncode != 0:
            fail("host gcc 컴파일 실패:\n" + build.stderr)
        run = subprocess.run([cbin], capture_output=True, text=True)
        if run.returncode != 0:
            fail("측정 프로그램 실행 실패")
        return int(run.stdout.strip())


#: 여유가 이보다 얇으면 문자열 하나 늘리는 순간 넘는다. 넘기 전에 멈춘다.
MIN_HEADROOM = 128


def evaluate(source):
    """한 소스에 대한 판정을 dict 로 돌려준다. 🔴 인쇄와 판정을 나눠 둔 이유는
    변이 주입 자가검사가 **같은 판정 경로**를 다시 타야 하기 때문이다 (검토 §66.3).
    """
    buffer_size, fmt, ino_args = extract(source)
    built, notes = build_args(fmt, ino_args, source)
    worst = measure(fmt, built)
    payload_limit = buffer_size - 1  # NUL 한 칸
    headroom = payload_limit - worst
    return dict(buffer_size=buffer_size, payload_limit=payload_limit,
                specs=len(ino_args), notes=notes, worst=worst,
                headroom=headroom,
                verdict=("OVERFLOW" if headroom < 0
                         else "THIN" if headroom < MIN_HEADROOM
                         else "OK"))


def self_check(source, payload_limit):
    """🔴 길이 초과 변이가 반드시 FAIL 하는가 (검토 §66.3).

    "안 넘는다"는 판정은 "넘는 것을 넘는다고 부를 수 있다"가 전제다. 그 전제를
    확인하지 않으면 이 도구가 무엇을 재고 있는지 알 수 없다. format 을 여유보다
    길게 늘려 다시 판정한다 — 저장소 파일은 안 건드리고 문자열만 바꾼다.
    """
    padding = "P" * (payload_limit + 64)
    anchor = 'libraries=%s'
    if source.count(anchor) != 1:
        fail("변이 지점 %r 을 하나로 특정하지 못했다" % anchor)
    mutated = source.replace(anchor, padding + anchor)
    return evaluate(mutated)["verdict"] == "OVERFLOW"


def main():
    source = read_ino()
    result = evaluate(source)

    print("── /firmware/info 길이 검사 (검토 §65.5·§66.3) ────────────────")
    print("  버퍼            : %d 바이트 (payload 한계 %d)"
          % (result["buffer_size"], result["payload_limit"]))
    print("  변환지시자      : %d 개 (인자와 짝 맞음)" % result["specs"])
    for note in result["notes"]:
        print(note)
    # 🔴 명칭 정정 (검토 §66.3). %d·%lu 는 타입 경계지만 %s·%f 는 현재 소스의 값이다.
    #    전체를 "타입 이론 최악" 이라 부르면 실제보다 강한 주장이 된다.
    print("  현재 소스 상한  : %d 자  (%%d·%%lu = 타입 경계 · %%s·%%f = 소스 실값)"
          % result["worst"])
    print("  여유            : %d 자" % result["headroom"])

    if result["verdict"] == "OVERFLOW":
        fail("버퍼를 %d 자 넘는다 — snprintf 가 조용히 자른다" % (-result["headroom"]))
    if result["verdict"] == "THIN":
        print("  \033[33m경고\033[0m  여유 %d 자 — 버퍼를 키워라" % result["headroom"])
        sys.exit(1)

    if not self_check(source, result["payload_limit"]):
        fail("길이 초과 변이를 넣었는데 OVERFLOW 가 안 났다 — 이 도구의 판정은 근거가 없다")
    print("  \033[32mOK\033[0m  넘지 않는다 (초과 변이는 잡는 것을 확인함)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
