#!/usr/bin/env python3
"""run_experiments.py — 시뮬 빌드 → 손계산 대조(3단계) → 실험 그래프 4종(4단계).

사용 (sim 폴더에서):
    python3 run_experiments.py

결과:
    results/checks.md            손계산 ↔ 시뮬 대조표
    results/1_step_response.png  스텝 응답 (0 → 0.12 m/s)
    results/2_kp_compare.png     Kp 30·적분상한 20 (08-22 이전) vs Kp 60·40 (현재)
    results/3_windup.png         바퀴 막힘 → 풀림 (적분 상한 · 조건부 적분)
    results/4_filter.png         속도 필터 α (잡음 vs 지연)

이 스크립트는 그래프·대조만 한다. 제어 계산은 전부 C++ 시뮬(펌웨어 헤더 그대로)이 한다.
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

MODEL_NOTE = "모터는 가정 모델 (지면 실측 직선 + 시정수 0.10 s 가정) — 제어 코드는 펌웨어 그대로"


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


def footer(fig, text=MODEL_NOTE):
    fig.text(0.5, 0.005, text, ha="center", fontsize=8, color="gray")


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


# ─────────────────────────────────────────────────────────────────────────
# 4단계 — 그래프
# ─────────────────────────────────────────────────────────────────────────
def plot_step():
    d = run(seconds=3)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    ax1.plot(d["t_s"], d["target"], "k--", label="목표 0.12 m/s")
    ax1.plot(d["t_s"], d["true_velocity"], label="실제 바퀴 속도 (모델)")
    ax1.step(d["t_s"], d["measured"], where="post", alpha=0.45, label="측정 속도 (엔코더, 20 ms)")
    ax1.plot(d["t_s"], d["filtered"], label="필터 속도 (제어기가 보는 값)")
    ax1.set_ylabel("속도 [m/s]")
    ax1.set_title("① 스텝 응답 — 정지 → 0.12 m/s")
    ax1.legend(loc="lower right", fontsize=9)
    ax1.grid(alpha=0.3)

    ax2.step(d["t_s"], d["applied_pwm"], where="post", label="실제 출력 PWM (부스트·램프 후)")
    ax2.step(d["t_s"], d["desired_pwm"], where="post", label="원하는 PWM (FF + P + I)")
    ax2.plot(d["t_s"], d["ff_pwm"], "k:", label="FF 64")
    ax2.annotate("출발 부스트 75", xy=(0.04, 75), xytext=(0.4, 76), arrowprops=dict(arrowstyle="->"))
    ax2.set_ylabel("PWM")
    ax2.set_xlabel("시간 [s]")
    ax2.set_ylim(55, 80)
    ax2.legend(loc="upper right", fontsize=9)
    ax2.grid(alpha=0.3)
    footer(fig)
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    fig.savefig(os.path.join(RESULTS, "1_step_response.png"))
    plt.close(fig)


def plot_kp():
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    nominal = {"Kp 30 · 적분상한 20 (08-22 이전)": dict(kp=30, ilimit=20),
               "Kp 60 · 적분상한 40 (현재)": dict(kp=60, ilimit=40)}
    for label, gains in nominal.items():
        d = run(seconds=3, **gains)
        axes[0].plot(d["t_s"], d["true_velocity"], label=label)
    axes[0].axhline(0.12, color="k", ls="--", lw=1)
    axes[0].set_title("(a) 평지 0.12 m/s — 거의 같다 (FF 가 대부분을 낸다)")
    axes[0].set_xlabel("시간 [s]")
    axes[0].set_ylabel("실제 바퀴 속도 [m/s]")
    axes[0].legend(fontsize=9)
    axes[0].grid(alpha=0.3)

    load = 50
    for label, gains in nominal.items():
        d = run(target=0.187, load=load, seconds=120, **gains)
        axes[1].plot(d["t_s"], d["true_velocity"], label=label)
    axes[1].axhline(0.187, color="k", ls="--", lw=1, label="목표 0.187 m/s (FF 86)")
    axes[1].set_title(f"(b) 부하 {load} PWM (예시) — 되돌릴 권한의 차이")
    axes[1].set_xlabel("시간 [s]")
    axes[1].legend(fontsize=9, loc="lower right")
    axes[1].grid(alpha=0.3)
    footer(fig, MODEL_NOTE + " · (b) 부하 값은 예시, 0.187 = .ino:337 주석의 08-22 오차")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(os.path.join(RESULTS, "2_kp_compare.png"))
    plt.close(fig)


def plot_windup():
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    cases = {"적분상한 40 (현재)": 40, "적분상한 20 (08-22 이전)": 20, "상한 없음 (비교용)": 1000}
    for label, limit in cases.items():
        d = run(target=0.12, seconds=200, block_from=0, block_to=80, ilimit=limit)
        axes[0][0].plot(d["t_s"], d["i_term"], label=label)
        axes[1][0].plot(d["t_s"], d["true_velocity"], label=label)
    axes[0][0].axvspan(0, 80, color="red", alpha=0.07)
    axes[0][0].set_title("(a) 0.12 m/s 명령 · 바퀴 0~80 s 막힘 → I 항")
    axes[0][0].set_ylabel("I 항 [PWM]")
    axes[0][0].legend(fontsize=9)
    axes[1][0].axhline(0.12, color="k", ls="--", lw=1)
    axes[1][0].set_xlim(75, 200)
    axes[1][0].set_title("(a) 80 s 에 풀린 뒤 — 쌓인 적분만큼 과속, 적분이 느려 오래 간다")
    axes[1][0].set_ylabel("실제 바퀴 속도 [m/s]")
    axes[1][0].set_xlabel("시간 [s]")
    axes[1][0].legend(fontsize=9)

    d = run(target=0.30, seconds=40, block_from=0, block_to=30)
    axes[0][1].plot(d["t_s"], d["i_term"], label="I 항")
    axes[0][1].axhline(40, color="gray", ls=":", label="적분 상한 40")
    axes[0][1].plot(d["t_s"], [p - 124 - 18 for p in d["desired_pwm"]], alpha=0)  # 축 맞춤용
    axes[0][1].axvspan(0, 30, color="red", alpha=0.07)
    axes[0][1].set_title("(b) 0.30 m/s 명령 · 0~30 s 막힘 → 조건부 적분")
    axes[0][1].set_ylabel("I 항 [PWM]")
    axes[0][1].set_ylim(-2, 45)
    axes[0][1].legend(fontsize=9)
    axes[1][1].step(d["t_s"], d["desired_pwm"], where="post", label="원하는 PWM")
    axes[1][1].axhline(160, color="red", ls=":", label="출력 상한 160")
    axes[1][1].axvspan(0, 30, color="red", alpha=0.07)
    axes[1][1].set_title("(b) 출력이 160 에 닿는 순간 적분이 멈춘다 (I = 18)")
    axes[1][1].set_ylabel("PWM")
    axes[1][1].set_xlabel("시간 [s]")
    axes[1][1].legend(fontsize=9)
    for row in axes:
        for ax in row:
            ax.grid(alpha=0.3)
    footer(fig, MODEL_NOTE + " · ⚠ (b) 풀린 뒤 속도는 직선 모델 맞춤 구간 밖이라 생략")
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(os.path.join(RESULTS, "3_windup.png"))
    plt.close(fig)


def plot_filter():
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    d = run(seconds=3)
    axes[0].step(d["t_s"], d["measured"], where="post", alpha=0.5, label="측정 (17/18칸 → 0.1152/0.1220)")
    axes[0].plot(d["t_s"], d["filtered"], lw=2, label="필터 α 0.10")
    axes[0].plot(d["t_s"], d["true_velocity"], "k--", lw=1, label="실제 (모델)")
    axes[0].set_xlim(1.0, 3.0)
    axes[0].set_ylim(0.112, 0.126)
    axes[0].set_title("(a) 정상상태 — 양자화 잡음을 필터가 고른다")
    axes[0].set_xlabel("시간 [s]")
    axes[0].set_ylabel("속도 [m/s]")
    axes[0].legend(fontsize=9)

    for alpha in (0.10, 0.30, 1.00):
        d = run(seconds=2, alpha=alpha)
        name = "필터 없음 (α 1.0)" if alpha == 1.0 else f"α {alpha:.2f}" + (" (현재)" if alpha == 0.10 else "")
        axes[1].plot(d["t_s"], d["true_velocity"], label=name)
    axes[1].axhline(0.12, color="k", ls="--", lw=1)
    axes[1].set_title("(b) 출발 — 필터가 느릴수록(α 작을수록) 지연 때문에 넘친다")
    axes[1].set_xlabel("시간 [s]")
    axes[1].set_ylabel("실제 바퀴 속도 [m/s]")
    axes[1].legend(fontsize=9)
    for ax in axes:
        ax.grid(alpha=0.3)
    footer(fig)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(os.path.join(RESULTS, "4_filter.png"))
    plt.close(fig)


def main():
    os.makedirs(RESULTS, exist_ok=True)
    build()
    ok = checks()
    plot_step()
    plot_kp()
    plot_windup()
    plot_filter()
    print("그래프 →", RESULTS)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
