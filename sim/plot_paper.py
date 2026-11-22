#!/usr/bin/env python3
"""plot_paper.py — 논문 양식 그래프 + 수치 표.

논문 그림의 관례를 따른다:
  · 제목 없음 — 설명은 보고서의 그림 캡션("그림 5-1. …")에 쓴다
  · 한 단 폭(3.5 inch) · 세리프 글꼴 · 작은 글자 (보고서판)
  · 색 + 선 종류(실선·쇄선·점선)를 같이 쓴다 — 흑백으로 인쇄해도 구분된다
    (색 = Okabe-Ito 팔레트의 파랑·주황·초록, 색각 이상에도 구분된다)
  · 눈금 안쪽, 네 변 모두 · 범례 테두리 없음 · 관련 그림은 (a)(b)
  · 벡터(PDF) + 300 dpi PNG
  · 수치는 그림이 아니라 표로 (tables.md)

같은 그림을 두 판으로 만든다:
  results/paper/   보고서판 (A4 한 단 폭, 7~8 pt)
  results/slides/  발표판   (글자·선을 키움, 16:9 화면용)

사용 (sim 폴더에서): python3 run_experiments.py   (빌드 → 대조 → 이 스크립트)
"""

import csv
import io
import os
import subprocess

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "build", "sim")

BLUE, ORANGE, GREEN = "#0072B2", "#D55E00", "#009E73"   # Okabe-Ito

BASE_STYLE = {
    "font.family": "Noto Serif CJK JP",   # 한글 세리프 (.ttc 가 JP 이름으로 등록됨, 한글 포함)
    "mathtext.fontset": "stix",           # 수식 글꼴 = Times 계열
    "axes.unicode_minus": False,
    "xtick.direction": "in",
    "ytick.direction": "in",
    "xtick.top": True,
    "ytick.right": True,
    "legend.frameon": False,
    "savefig.dpi": 300,
}

# 판마다 다른 것: 그림 크기 배율 · 글자 · 선 굵기
EDITIONS = {
    "paper": {"scale": 1.0, "font.size": 8, "legend.fontsize": 7, "tick": 7,
              "axes.linewidth": 0.6, "lines.linewidth": 1.0},
    "slides": {"scale": 1.9, "font.size": 15, "legend.fontsize": 13, "tick": 13,
               "axes.linewidth": 1.0, "lines.linewidth": 2.0},
}


def run(**kwargs):
    """시뮬을 한 번 돌려 열 이름 → 숫자 리스트 사전으로 돌려준다."""
    args = [SIM] + [f"{k}={v}" for k, v in kwargs.items()]
    out = subprocess.run(args, check=True, capture_output=True, text=True).stdout
    rows = list(csv.DictReader(io.StringIO(out)))
    return {key: [float(r[key]) for r in rows] for key in rows[0]}


def value_at(data, column, t):
    i = min(range(len(data["t_s"])), key=lambda k: abs(data["t_s"][k] - t))
    return data[column][i]


def step_metrics(t, y):
    """상승 시간(10→90 %) · 오버슈트 · 2 % 정착 시간. 기준 = 최종값."""
    final = y[-1]
    t10 = next(t[i] for i in range(len(y)) if y[i] >= 0.1 * final)
    t90 = next(t[i] for i in range(len(y)) if y[i] >= 0.9 * final)
    overshoot = (max(y) - final) / final * 100.0
    settle = 0.0
    for i in range(len(y)):
        if abs(y[i] - final) > 0.02 * final:
            settle = t[min(i + 1, len(t) - 1)]
    return final, t90 - t10, overshoot, settle


def label(ax, text, y=0.90):
    ax.text(0.02, y, text, transform=ax.transAxes)


# ─────────────────────────────────────────────────────────────────────────
# 데이터 (판과 무관하게 한 번만 돌린다)
# ─────────────────────────────────────────────────────────────────────────
def collect():
    data = {}
    data["step"] = run(seconds=1.5)
    data["kp_flat"] = {k: run(seconds=1.5, kp=kp, ilimit=il)
                       for k, (kp, il) in {"old": (30, 20), "new": (60, 40)}.items()}
    data["kp_load"] = {k: run(target=0.187, load=50, seconds=120, kp=kp, ilimit=il)
                       for k, (kp, il) in {"old": (30, 20), "new": (60, 40)}.items()}
    data["windup"] = {lim: run(target=0.12, seconds=200, block_from=0, block_to=80, ilimit=lim)
                      for lim in (40, 20, 1000)}
    data["cond"] = run(target=0.30, seconds=40, block_from=0, block_to=30)
    data["filter"] = {a: run(seconds=1.5, alpha=a) for a in (0.10, 0.30, 1.00)}
    data["filter_ss"] = run(seconds=3)
    return data


# ─────────────────────────────────────────────────────────────────────────
# 그림
# ─────────────────────────────────────────────────────────────────────────
def fig_step(d, s):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(3.5 * s, 3.6 * s), sharex=True)
    ax1.plot(d["t_s"], d["target"], color="black", ls=":", lw=0.8 * s, label="목표")
    ax1.plot(d["t_s"], d["true_velocity"], color=BLUE, ls="-", label="실제 속도")
    ax1.plot(d["t_s"], d["filtered"], color=ORANGE, ls="-.", label="필터 속도")
    ax1.set_ylabel("속도 $v$ (m/s)")
    ax1.set_ylim(0, 0.14)
    ax1.legend(loc="lower right")
    label(ax1, "(a)")
    ax2.step(d["t_s"], d["applied_pwm"], where="post", color=BLUE, ls="-", label="출력 PWM")
    ax2.step(d["t_s"], d["desired_pwm"], where="post", color=ORANGE, ls="--", label="계산 PWM")
    ax2.set_ylabel("PWM")
    ax2.set_xlabel("시간 $t$ (s)")
    ax2.set_xlim(0, 1.5)
    ax2.set_ylim(55, 80)
    ax2.legend(loc="upper right")
    label(ax2, "(b)", 0.86)
    return fig


def fig_kp(flat, load, s):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(3.5 * s, 3.8 * s))
    names = {"old": "$K_p$=30, $I_{max}$=20", "new": "$K_p$=60, $I_{max}$=40"}
    styles = {"old": (ORANGE, "--"), "new": (BLUE, "-")}
    for k in ("old", "new"):
        c, ls = styles[k]
        ax1.plot(flat[k]["t_s"], flat[k]["true_velocity"], color=c, ls=ls, label=names[k])
        ax2.plot(load[k]["t_s"], load[k]["true_velocity"], color=c, ls=ls, label=names[k])
    ax1.axhline(0.12, color="black", ls=":", lw=0.8 * s)
    ax1.set_xlim(0, 1.5)
    ax1.set_ylim(0, 0.14)
    ax1.set_xlabel("시간 $t$ (s)")
    ax1.set_ylabel("속도 $v$ (m/s)")
    ax1.legend(loc="lower right")
    label(ax1, "(a)")
    ax2.axhline(0.187, color="black", ls=":", lw=0.8 * s)
    ax2.set_xlim(0, 120)
    ax2.set_ylim(0, 0.22)
    ax2.set_xlabel("시간 $t$ (s)")
    ax2.set_ylabel("속도 $v$ (m/s)")
    ax2.legend(loc="lower right")
    label(ax2, "(b)")
    return fig


def fig_windup(runs, s):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(3.5 * s, 3.8 * s), sharex=True)
    names = {40: "$I_{max}$=40", 20: "$I_{max}$=20", 1000: "상한 없음"}
    styles = {40: (BLUE, "-"), 20: (ORANGE, "--"), 1000: (GREEN, "-.")}
    for lim in (40, 20, 1000):
        c, ls = styles[lim]
        ax1.plot(runs[lim]["t_s"], runs[lim]["i_term"], color=c, ls=ls, label=names[lim])
        ax2.plot(runs[lim]["t_s"], runs[lim]["true_velocity"], color=c, ls=ls, label=names[lim])
    for ax in (ax1, ax2):
        ax.axvspan(0, 80, color="0.9", lw=0)
    ax1.set_ylabel("I 항 (PWM)")
    ax1.set_ylim(0, 55)
    ax1.legend(loc="upper right")
    label(ax1, "(a)")
    ax2.axhline(0.12, color="black", ls=":", lw=0.8 * s)
    ax2.set_ylabel("속도 $v$ (m/s)")
    ax2.set_xlabel("시간 $t$ (s)")
    ax2.set_xlim(0, 200)
    ax2.set_ylim(0, 0.30)
    ax2.legend(loc="upper right")
    label(ax2, "(b)")
    return fig


def fig_cond(d, s):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(3.5 * s, 3.6 * s), sharex=True)
    for ax in (ax1, ax2):
        ax.axvspan(0, 30, color="0.9", lw=0)
    ax1.plot(d["t_s"], d["i_term"], color=BLUE, ls="-", label="I 항")
    ax1.axhline(40, color="black", ls=":", lw=0.8 * s, label="적분 상한 40")
    ax1.set_ylabel("I 항 (PWM)")
    ax1.set_ylim(0, 45)
    ax1.legend(loc="center right")
    label(ax1, "(a)")
    ax2.step(d["t_s"], d["desired_pwm"], where="post", color=BLUE, ls="-", label="계산 PWM")
    ax2.axhline(160, color="black", ls=":", lw=0.8 * s, label="출력 상한 160")
    ax2.set_ylabel("PWM")
    ax2.set_xlabel("시간 $t$ (s)")
    ax2.set_xlim(0, 40)
    ax2.set_ylim(130, 170)
    ax2.legend(loc="lower left")
    label(ax2, "(b)", 0.86)
    return fig


def fig_filter(ss, runs, s):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(3.5 * s, 3.8 * s))
    ax1.step(ss["t_s"], ss["measured"], where="post", color=ORANGE, ls="-", lw=0.7 * s,
             label="측정 속도")
    ax1.plot(ss["t_s"], ss["filtered"], color=BLUE, ls="-", label="필터 속도 ($\\alpha$=0.10)")
    ax1.set_xlim(1.0, 3.0)
    ax1.set_ylim(0.114, 0.124)
    ax1.set_xlabel("시간 $t$ (s)")
    ax1.set_ylabel("속도 $v$ (m/s)")
    ax1.legend(loc="lower right")
    label(ax1, "(a)")
    names = {0.10: "$\\alpha$=0.10", 0.30: "$\\alpha$=0.30", 1.00: "$\\alpha$=1.0 (필터 없음)"}
    styles = {0.10: (BLUE, "-"), 0.30: (ORANGE, "--"), 1.00: (GREEN, "-.")}
    for a in (0.10, 0.30, 1.00):
        c, ls = styles[a]
        ax2.plot(runs[a]["t_s"], runs[a]["true_velocity"], color=c, ls=ls, label=names[a])
    ax2.axhline(0.12, color="black", ls=":", lw=0.8 * s)
    ax2.set_xlim(0, 1.5)
    ax2.set_ylim(0, 0.14)
    ax2.set_xlabel("시간 $t$ (s)")
    ax2.set_ylabel("실제 속도 $v$ (m/s)")
    ax2.legend(loc="lower right")
    label(ax2, "(b)")
    return fig


# ─────────────────────────────────────────────────────────────────────────
# 표
# ─────────────────────────────────────────────────────────────────────────
def tables(data):
    out = ["# 수치 표", "",
           "모든 값 = 시뮬레이션. 모터는 가정 모델(지면 실측 직선 v = 0.003691 × (PWM − 30.99), 시정수 τ = 0.10 s 가정).",
           "오버슈트·정착 시간의 기준 = **최종값**(목표가 아님). `python3 run_experiments.py` 로 다시 만든다.", ""]

    d = data["step"]
    final, rise, over, settle = step_metrics(d["t_s"], d["true_velocity"])
    out += ["## 표 1. 스텝 응답 (목표 0.12 m/s) — fig_step", "",
            "| 지표 | 값 |", "|---|---|",
            f"| 최종 속도 | {final:.4f} m/s |",
            f"| 상승 시간 (10→90 %) | {rise:.2f} s |",
            f"| 오버슈트 | {over:.1f} % |",
            f"| 정착 시간 (±2 %) | {settle:.2f} s |",
            "| 출발 부스트 | 75 PWM, 0.02~0.06 s |",
            f"| 정상상태 PWM | {d['applied_pwm'][-1]:.0f} |", ""]

    out += ["## 표 2. 게인 비교 — fig_kp", "",
            "| 조건 | Kp, 적분 상한 | 오버슈트 | 정착 시간 | 최종/끝 속도 |", "|---|---|---|---|---|"]
    for k, name in (("old", "30, 20"), ("new", "60, 40")):
        f = data["kp_flat"][k]
        fin, _, ov, st = step_metrics(f["t_s"], f["true_velocity"])
        out.append(f"| (a) 평지 0.12 m/s | {name} | {ov:.1f} % | {st:.2f} s | {fin:.4f} m/s |")
    for k, name in (("old", "30, 20"), ("new", "60, 40")):
        f = data["kp_load"][k]
        out.append(f"| (b) 부하 50 PWM (예시), 목표 0.187 | {name} | – | – | "
                   f"30 s {value_at(f, 'true_velocity', 30):.3f} · 60 s {value_at(f, 'true_velocity', 60):.3f} · "
                   f"120 s {f['true_velocity'][-1]:.3f} m/s |")
    out.append("")

    out += ["## 표 3. 바퀴 막힘 0~80 s → 풀림 (목표 0.12 m/s) — fig_windup", "",
            "| 적분 상한 | 풀릴 때 I 항 | 풀린 뒤 최고 속도 | +20 s | +60 s | +120 s |", "|---|---|---|---|---|---|"]
    for lim, name in ((40, "40 (현재)"), (20, "20 (08-22 이전)"), (1000, "없음 (비교)")):
        w = data["windup"][lim]
        after = [w["true_velocity"][i] for i in range(len(w["t_s"])) if w["t_s"][i] >= 80]
        out.append(f"| {name} | {value_at(w, 'i_term', 79.98):.1f} | {max(after):.3f} m/s | "
                   f"{value_at(w, 'true_velocity', 100):.3f} | {value_at(w, 'true_velocity', 140):.3f} | "
                   f"{value_at(w, 'true_velocity', 200):.3f} |")
    out.append("")

    c = data["cond"]
    t160 = next(c["t_s"][i] for i in range(len(c["t_s"])) if c["desired_pwm"][i] >= 160)
    out += ["## 표 4. 바퀴 막힘 0~30 s (목표 0.30 m/s) — fig_cond", "",
            "| 지표 | 값 |", "|---|---|",
            f"| 첫 계산 PWM (FF 124 + P 18) | {c['desired_pwm'][0]:.0f} |",
            f"| 출력 상한 160 도달 | {t160:.2f} s |",
            f"| I 항 최대 (조건부 적분으로 멈춘 값) | {max(c['i_term']):.1f} PWM |",
            "| 적분 상한 40 | 이 조건에서는 닿지 않음 |",
            "| ⚠ 풀린 뒤 속도 | 직선 모델 맞춤 구간(PWM 55~67) 밖이라 보고하지 않음 |", ""]

    ss = data["filter_ss"]
    idx = [i for i in range(len(ss["t_s"])) if ss["t_s"][i] >= 1.0]

    def spread(col):
        vals = [ss[col][i] for i in idx]
        return max(vals) - min(vals)
    out += ["## 표 5. 속도 필터 — fig_filter", "",
            f"(a) 정상상태(1~3 s) 흔들림 폭: 측정 {spread('measured'):.4f} m/s → 필터 {spread('filtered'):.4f} m/s", "",
            "| α | 오버슈트 | 정착 시간 |", "|---|---|---|"]
    for a in (0.10, 0.30, 1.00):
        f = data["filter"][a]
        _, _, ov, st = step_metrics(f["t_s"], f["true_velocity"])
        out.append(f"| {a:.2f}{' (현재)' if a == 0.10 else ''} | {ov:.1f} % | {st:.2f} s |")
    out += ["", "⚠ 시뮬의 측정 잡음은 엔코더 양자화뿐이다(진동·떨림 없음) → 필터의 이득은 실제보다 작게 보인다.", ""]
    return "\n".join(out)


def main():
    data = collect()
    for edition, cfg in EDITIONS.items():
        out_dir = os.path.join(HERE, "results", edition)
        os.makedirs(out_dir, exist_ok=True)
        plt.rcParams.update(BASE_STYLE)
        plt.rcParams.update({
            "font.size": cfg["font.size"], "axes.labelsize": cfg["font.size"],
            "legend.fontsize": cfg["legend.fontsize"],
            "xtick.labelsize": cfg["tick"], "ytick.labelsize": cfg["tick"],
            "axes.linewidth": cfg["axes.linewidth"], "lines.linewidth": cfg["lines.linewidth"],
        })
        s = cfg["scale"]
        figures = {
            "fig_step": fig_step(data["step"], s),
            "fig_kp": fig_kp(data["kp_flat"], data["kp_load"], s),
            "fig_windup": fig_windup(data["windup"], s),
            "fig_cond": fig_cond(data["cond"], s),
            "fig_filter": fig_filter(data["filter_ss"], data["filter"], s),
        }
        for name, fig in figures.items():
            fig.tight_layout(h_pad=0.6)
            fig.savefig(os.path.join(out_dir, name + ".png"))
            if edition == "paper":
                fig.savefig(os.path.join(out_dir, name + ".pdf"))
            plt.close(fig)
    text = tables(data)
    with open(os.path.join(HERE, "results", "paper", "tables.md"), "w", encoding="utf-8") as f:
        f.write(text)
    print(text)


if __name__ == "__main__":
    main()
