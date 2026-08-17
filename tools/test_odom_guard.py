#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""`odom_guard.py` 회귀 — 실측 표본을 고정값으로 박고 변이로 감도를 증명한다.

입력은 **지어낸 것이 아니다.** 2026-08-14 지도 세션 두 개의 bag 에서 실제로 나온
공분산 대각을 그대로 옮겨 왔다 (`MASTER_PLAN §7` 예약 41):

    map_0814_1350  (21분) — 12 건 · 첫 것이 +130.275s → 21ms 뒤 EKF NaN
    map_0814_1512  ( 9분) —  2 건 · 둘째가 +299.527s →  4ms 뒤 EKF NaN

🔴 **1512 의 두 건 중 하나는 EKF 를 죽였고 하나는 안 죽였다.** 그래서 이 시험은
"치명적인 것만 잡는가"가 아니라 **"정상 범위 밖을 전부 잡는가"** 를 본다.

rclpy 없이 돈다 — `check()` 가 순수 함수라 메시지 없이 판정할 수 있다.
"""
import sys

import odom_guard

# ── 실측 고정값 (twist covariance 의 대각 6개) ──────────────────────────────
NORMAL = [0.02, 0.02, 0.0, 0.0, 0.0, 0.1]

# map_0814_1350 — 12 건 중 서로 다른 모양 전부
CORRUPT_1350 = [
    [2.0e-2, 4.89397200e-295, 1.73795744e-060, 5.26499483e-315, 5.26499483e-315, 0.0],
    [2.0e-2, 4.89397200e-295, -7.29210256e-066, 5.26499483e-315, 5.26499483e-315, 0.0],
    [2.0e-2, 3.13850589e-05, 1.25524178e-01, 0.0, 0.0, 0.0],
    [2.0e-2, 4.89397200e-295, 6.06804230e-043, 5.26499483e-315, 5.26499483e-315, 0.0],
    [2.0e-2, 4.89397200e-295, -3.44559924e+178, 5.26499483e-315, 5.26499483e-315, 0.0],
    [4.89397200e-295, 3.65876093e+255, 5.26499483e-315, 5.26499483e-315, 0.0, 0.0],
    [2.0e-2, 4.89397200e-295, 1.19096212e+081, 5.26499483e-315, 5.26499483e-315, 0.0],
    [2.0e-2, 4.89397200e-295, 6.24362701e+177, 5.26499483e-315, 5.26499483e-315, 0.0],
    [2.0e-2, 4.89397200e-295, 8.96022702e+037, 5.26499483e-315, 5.26499483e-315, 0.0],
]
# map_0814_1512 — 🔴 첫째는 EKF 가 살아남았고 둘째가 죽였다. 둘 다 잡아야 한다.
CORRUPT_1512_SURVIVED = [4.8940e-295, 3.6925e+053, 5.2650e-315, 5.2650e-315, 0.0, 0.0]
CORRUPT_1512_KILLED = [2.0000e-002, 4.8940e-295, 9.4306e-153, 5.2650e-315, 5.2650e-315, 0.0]

# 실측된 쓰레기 vx (공분산은 멀쩡할 수도 있는 경로를 따로 잠근다)
CORRUPT_VX = (2.42337, 3.32595, 3.1118, 2.6326)


def flat(diag):
    """대각 6개를 6x6 행 우선 36칸으로 편다."""
    m = [0.0] * 36
    for i, v in enumerate(diag):
        m[i * 6 + i] = v
    return m


def ok(diag, vx=0.12, vy=0.0, wz=0.0):
    return odom_guard.check(flat(diag), vx, vy, wz)


def main():
    fails = []

    def want_pass(name, *a, **k):
        why = ok(*a, **k)
        if why is not None:
            fails.append(f'{name}: 정상인데 버렸다 — {why}')

    def want_drop(name, *a, **k):
        if ok(*a, **k) is None:
            fails.append(f'{name}: 깨졌는데 통과시켰다')

    # ── 1. 정상은 통과한다 (역회귀 앵커) ────────────────────────────────────
    want_pass('정상', NORMAL)
    want_pass('정상 · 순항 실측 0.128', NORMAL, vx=0.128)
    want_pass('정상 · 후진', NORMAL, vx=-0.12)
    want_pass('정상 · 회전 중', NORMAL, vx=0.0, wz=0.5)

    # ── 2. 실측 표본을 전부 잡는다 (bag 14건 중 서로 다른 모양 11종) ───────
    for i, d in enumerate(CORRUPT_1350, 1):
        want_drop(f'1350 실측 #{i}', d)
    want_drop('1512 실측 · EKF 가 살아남은 것', CORRUPT_1512_SURVIVED)
    want_drop('1512 실측 · EKF 를 죽인 것', CORRUPT_1512_KILLED)
    for v in CORRUPT_VX:
        want_drop(f'실측 쓰레기 vx={v}', NORMAL, vx=v)

    # ── 3. 경계는 양쪽을 잠근다 (AGENTS §3-10 ⑤) ───────────────────────────
    lo, hi = odom_guard.COV_MIN, odom_guard.COV_MAX
    for slot in (0, 1, 5):                       # vx, vy, vyaw 각각
        d = list(NORMAL)
        d[slot] = lo * 0.99
        want_drop(f'cov[{slot}] 하한 미만', d)
        d[slot] = lo * 1.01
        want_pass(f'cov[{slot}] 하한 초과', d)
        d[slot] = hi * 1.01
        want_drop(f'cov[{slot}] 상한 초과', d)
        d[slot] = hi * 0.99
        want_pass(f'cov[{slot}] 상한 미만', d)
        d[slot] = 0.0
        want_drop(f'cov[{slot}] = 0', d)
        d[slot] = -1.0
        want_drop(f'cov[{slot}] 음수', d)
        d[slot] = float('nan')
        want_drop(f'cov[{slot}] NaN', d)
        d[slot] = float('inf')
        want_drop(f'cov[{slot}] inf', d)

    vmax = odom_guard.VX_ABS_MAX
    want_pass('vx 상한 미만', NORMAL, vx=vmax * 0.99)
    want_drop('vx 상한 초과', NORMAL, vx=vmax * 1.01)
    want_drop('vx 음의 상한 초과', NORMAL, vx=-vmax * 1.01)
    want_drop('vx NaN', NORMAL, vx=float('nan'))
    want_drop('wz NaN', NORMAL, wz=float('nan'))

    # ── 4. 🔴 안 보는 자리는 안 본다 — 과잉 폐기를 막는다 ──────────────────
    #    EKF 는 vz·vroll·vpitch 를 융합하지 않는다(odom0_config). 정상 메시지도
    #    그 자리가 0 이므로, 거기까지 검사하면 **모든 메시지를 버린다.**
    for slot in (2, 3, 4):
        d = list(NORMAL)
        d[slot] = 0.0
        want_pass(f'안 쓰는 cov[{slot}] = 0 은 통과해야 한다', d)
        d[slot] = float('nan')
        want_pass(f'안 쓰는 cov[{slot}] = NaN 도 통과해야 한다', d)

    # ── 5. 🔴 관측된 표본은 전부 `≤ 0` 규칙 하나로 잡힌다 — 숨기지 않는다 ──
    #    실측 표본은 모두 vyaw 공분산이 정확히 0.0 이라, 하한(COV_MIN)·상한(COV_MAX)은
    #    **실측이 요구한 규칙이 아니라 방어적으로 더 둔 것**이다. 그러므로 그 둘은
    #    실측 고정값으로는 감도를 증명할 수 없다 — **각자 혼자 발화하는 합성 사례**로
    #    증명한다. 이 사실을 안 적고 "14건으로 검증했다"고 쓰면 거짓이 된다.
    only_zero = list(NORMAL);  only_zero[5] = 0.0            # ≤0 만 발화
    only_min = list(NORMAL);   only_min[1] = 1e-20           # 0 초과이나 하한 미만
    only_max = list(NORMAL);   only_max[1] = 1e10            # 상한 초과
    only_nonfin = list(NORMAL); only_nonfin[0] = float('nan')

    for nm, d in (('≤0', only_zero), ('하한', only_min),
                  ('상한', only_max), ('비유한', only_nonfin)):
        want_drop(f'단독 발화 · {nm}', d)

    n_zero_rule = sum(1 for d in CORRUPT_1350 + [CORRUPT_1512_SURVIVED,
                                                 CORRUPT_1512_KILLED]
                      if any(d[s] <= 0.0 for s in (0, 1, 5)))
    if n_zero_rule != len(CORRUPT_1350) + 2:
        fails.append(f'실측 표본 중 ≤0 규칙으로 잡히는 것이 {n_zero_rule} — '
                     '전부라고 적은 주석이 사실과 다르다')

    saved_min, saved_max, saved_vx = (odom_guard.COV_MIN, odom_guard.COV_MAX,
                                      odom_guard.VX_ABS_MAX)
    try:
        odom_guard.COV_MIN = 0.0                 # 변이: 하한 제거
        if ok(only_min) is not None:
            fails.append('변이(하한 제거)인데 여전히 잡는다 — 하한이 안 쓰이고 있다')
        odom_guard.COV_MIN = saved_min

        odom_guard.COV_MAX = float('inf')        # 변이: 상한 제거
        if ok(only_max) is not None:
            fails.append('변이(상한 제거)인데 여전히 잡는다 — 상한이 안 쓰이고 있다')
        odom_guard.COV_MAX = saved_max

        odom_guard.VX_ABS_MAX = 1e9              # 변이: vx 상한 제거
        if ok(NORMAL, vx=3.32595) is not None:
            fails.append('변이(vx 상한 제거)인데 여전히 잡는다 — vx 검사가 안 쓰이고 있다')
    finally:
        (odom_guard.COV_MIN, odom_guard.COV_MAX,
         odom_guard.VX_ABS_MAX) = saved_min, saved_max, saved_vx

    # 원복 확인 — 변이가 새어 나가면 다음 검사가 거짓으로 통과한다
    if ok(CORRUPT_1512_KILLED) is None:
        fails.append('변이 원복 실패 — 상수가 되돌아오지 않았다')

    if fails:
        print('\033[31mFAIL\033[0m  아래를 고칠 것:')
        for f in fails:
            print(f'   · {f}')
        return 1
    n_cov = len(CORRUPT_1350) + 2
    print(f'\033[32m전량 통과\033[0m — 실측 공분산 {n_cov} 종(bag 12+2 건 중 서로 다른 모양) '
          f'+ 실측 쓰레기 vx {len(CORRUPT_VX)} 종을 전부 잡고, 정상과 안 쓰는 자리는 '
          f'통과시키며, 느슨하게 바꾸면 놓치는 것을 확인함')
    return 0


if __name__ == '__main__':
    sys.path.insert(0, __file__.rsplit('/', 1)[0])
    sys.exit(main())
