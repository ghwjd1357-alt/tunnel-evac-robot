#!/usr/bin/env python3
"""run_experiments.py — 시뮬 빌드 → 손계산 대조(3단계) → 그래프·수치 표(4단계, plot_paper.py).

사용 (sim 폴더에서):
    python3 run_experiments.py

결과:
    results/checks.md        손계산 ↔ 시뮬 대조표
    results/paper/           보고서판 그림 5개(PNG+PDF) + tables.md
    results/slides/          발표판 그림 5개(PNG)

이 스크립트는 대조·그래프만 한다. 제어 계산은 전부 C++ 시뮬(펌웨어 헤더 그대로)이 한다.
"""

import csv
import io
import os
import subprocess
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, "build")
RESULTS = os.path.join(HERE, "results")
SIM = os.path.join(BUILD, "sim")

# 한글 글꼴. Noto Sans CJK 의 .ttc 는 matplotlib 에 "JP" 이름으로 등록되지만 한글 글자도 들어 있다.
plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 110

def build():
    os.makedirs(BUILD, exist_ok=True)
    cmd = ["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
           "-I", os.path.join(HERE, "..", "firmware", "term_drive_v2"),
           os.path.join(HERE, "sim_main.cpp"), "-o", SIM]
    subprocess.run(cmd, check=True)


def run(**kwargs):
    """시뮬을 한 번 돌려 열 이름 → 숫자 리스트 사전으로 돌려준다."""
    args = [SIM] + [f"{k}={v}" for k, v in kwargs.items()]
    out = subprocess.run(args, check=True, capture_output=True, text=True).stdout
    rows = list(csv.DictReader(io.StringIO(out)))
    return {key: [float(r[key]) for r in rows] for key in rows[0]}


def at(data, t):
    """시각 t 에 가장 가까운 행 번호."""
    return min(range(len(data["t_s"])), key=lambda i: abs(data["t_s"][i] - t))


# ─────────────────────────────────────────────────────────────────────────
# 3단계 — 손계산 대조
# ─────────────────────────────────────────────────────────────────────────
def checks():
    rows = []

    def add(name, hand, sim, ok, where):
        rows.append((name, hand, sim, "✅" if ok else "❌", where))

    step = run(seconds=3)
    add("0.12 m/s 의 FF", "64 (63.5 반올림)", f"{step['ff_pwm'][0]:.0f}", step["ff_pwm"][0] == 64,
        "pid_controller.h:209")
    add("첫 계산 PWM (FF + P + I)", "64 + 7.2 + 0.012 = 71", f"{step['desired_pwm'][0]:.0f}",
        step["desired_pwm"][0] == 71, "pid_controller.h:232")
    boost = [step["applied_pwm"][i] for i in range(len(step["t_s"])) if step["t_s"][i] <= 0.08]
    add("출발 부스트", "75 를 80 ms 동안", "t=0.02~0.06 s 에 75 (3칸), 0.08 s 부터 램프",
        boost[1:4] == [75, 75, 75] and boost[4] != 75, "wheel.h:191")

    steady = [i for i in range(len(step["t_s"])) if step["t_s"][i] >= 1.0]
    measured = sorted(set(round(step["measured"][i], 4) for i in steady))
    add("정상상태 측정 속도 값", "0.1152 (17칸) / 0.1220 (18칸)", " / ".join(f"{m:.4f}" for m in measured),
        measured == [0.1152, 0.1220], "wheel.h:179")

    worst = 0.0
    for i in range(1, len(step["t_s"])):
        expect = step["filtered"][i - 1] + 0.10 * (step["measured"][i] - step["filtered"][i - 1])
        worst = max(worst, abs(expect - step["filtered"][i]))
    add("필터 식 f += 0.1 × (m − f)", "매 칸 성립", f"최대 차이 {worst:.1e} (CSV 소수 5자리 반올림 수준)",
        worst < 2e-5, "wheel.h:181")

    stall = run(target=0.12, seconds=90, block_from=0, block_to=90)
    i10 = at(stall, 10.0)
    add("0.12 막힘 10 s 뒤 PWM", "64 + 7.2 + 6 = 77", f"{stall['desired_pwm'][i10]:.0f}",
        stall["desired_pwm"][i10] == 77, "")
    reach = next(stall["t_s"][i] for i in range(len(stall["t_s"])) if stall["i_term"][i] >= 40.0 - 1e-6)
    add("0.12 막힘, I 항이 40 에 닿는 시각", "8 ÷ 0.12 = 66.7 s", f"{reach:.2f} s", abs(reach - 66.7) < 0.1,
        "pid_controller.h:157")

    stall30 = run(target=0.30, seconds=30, block_from=0, block_to=30)
    plateau = max(stall30["i_term"])
    add("0.30 막힘, I 항 최대 (조건부 적분)", "160 − 124 − 18 = 18 (상한 40 전에 멈춤)", f"{plateau:.2f}",
        abs(plateau - 18.0) < 0.1, "pid_controller.h:266")

    lines = ["# 손계산 ↔ 시뮬 대조 (3단계)", "",
             "시뮬 = 펌웨어 헤더(`firmware/term_drive_v2/`) 그대로 + 가짜 모터. `python3 run_experiments.py` 로 다시 만든다.",
             "", "| 항목 | 손계산 | 시뮬 | 판정 | 코드 |", "|---|---|---|---|---|"]
    lines += [f"| {a} | {b} | {c} | {d} | `{e}` |" if e else f"| {a} | {b} | {c} | {d} | |"
              for a, b, c, d, e in rows]
    passed = sum(1 for r in rows if r[3] == "✅")
    lines += ["", f"**{passed} / {len(rows)} 일치**", ""]
    with open(os.path.join(RESULTS, "checks.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("\n".join(lines))
    return passed == len(rows)


def main():
    os.makedirs(RESULTS, exist_ok=True)
    build()
    ok = checks()
    import plot_paper   # 그래프·수치 표 (논문 양식 · 보고서판 + 발표판)
    plot_paper.main()
    print("그래프 →", os.path.join(RESULTS, "paper"), "·", os.path.join(RESULTS, "slides"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
