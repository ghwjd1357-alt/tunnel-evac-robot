#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
accuracy_report.py — 벤치 CSV → 통계 + 그래프(PNG)
============================================================
[사용법]
  단일 런 분석:   python3 tools/accuracy_report.py trace.csv -o out.png
  전/후 비교:     python3 tools/accuracy_report.py before.csv after.csv \
                      --labels 조정전 조정후 -o compare.png

[통계 지표]
  mean : 궤적 전체 평균 오차 (ATE 평균)
  p95  : 95% 지점 오차 — "거의 항상 이 이내" (순간 스파이크에 덜 민감)
  max  : 최악 순간 오차
  final: 마지막 샘플 오차 (끝점 정확도)
"""

import argparse
import csv
import statistics

import matplotlib
matplotlib.use('Agg')            # 화면 없이 파일로만 (헤드리스)
import matplotlib.pyplot as plt

# 그래프 한글 깨짐 방지: 시스템에 나눔/노토 있으면 사용, 없으면 영문 표기 유지
for font in ('NanumGothic', 'Noto Sans CJK KR'):
    try:
        matplotlib.font_manager.findfont(font, fallback_to_default=False)
        plt.rcParams['font.family'] = font
        break
    except Exception:
        continue


def load(path):
    """CSV → (경과시간 리스트, 오차 리스트). 경과시간은 첫 샘플 기준 0초."""
    ts, errs = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            ts.append(float(row['t']))
            errs.append(float(row['err']))
    if not ts:
        raise SystemExit(f'{path}: 샘플 없음')
    t0 = ts[0]
    return [t - t0 for t in ts], errs


def stats(errs):
    s = sorted(errs)
    return {
        'n': len(errs),
        'mean': statistics.mean(errs),
        'p95': s[int(len(s) * 0.95) - 1] if len(s) > 1 else s[0],
        'max': max(errs),
        'final': errs[-1],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csvs', nargs='+', help='trace CSV (1개=단일, 2개=전/후 비교)')
    ap.add_argument('--labels', nargs='+', default=None)
    ap.add_argument('-o', '--out', default='accuracy.png')
    args = ap.parse_args()

    labels = args.labels or [f'run{i+1}' for i in range(len(args.csvs))]

    plt.figure(figsize=(9, 4.5))
    for path, label in zip(args.csvs, labels):
        ts, errs = load(path)
        st = stats(errs)
        print(f'[{label}] 샘플 {st["n"]}개 | 평균 {st["mean"]:.3f}m | '
              f'p95 {st["p95"]:.3f}m | 최대 {st["max"]:.3f}m | 최종 {st["final"]:.3f}m')
        plt.plot(ts, errs, label=f'{label} (mean {st["mean"]:.2f}m)')

    plt.xlabel('elapsed sim time (s)')
    plt.ylabel('position error (m)  |gt - slam|')
    plt.title('SLAM position error over trajectory')
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.out, dpi=130)
    print(f'그래프 저장: {args.out}')


if __name__ == '__main__':
    main()
