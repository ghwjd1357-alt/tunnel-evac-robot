#!/usr/bin/env python3
"""SEARCH_BACK 진입 yaw 관문 — 사람을 멈추기 **직전에** 한 줄로 판정한다.

사용 (Jetson, 로봇이 아래 복도 직선 x≈7 에 있을 때):
    python3 tools/yaw_gate.py                # 기본 목표 π, 허용 ±0.3 rad
    python3 tools/yaw_gate.py --target 3.14159 --tol 0.3

왜 이 도구가 있나
-----------------
🔴 08-21 검토 §83.9 — **서쪽 탈출구는 180° 의 충분조건이 아니다.**
`SEARCH_BACK` 의 목표 yaw 는 `0.0` 하드코딩(예약 58)이고, 실제 회전량은
**진입 순간의 로봇 yaw** 가 정한다. 그래서 필요한 값은 *"테이크 시작 전"* 이
아니라 *"아래 복도 직선에서 사람을 멈추기 직전"* 의 yaw 다.

구판 런북은 이 확인을 테이크 절차 ⑥(주행 시작 **전**)에 뒀다. 그 뒤의 Nav2 주행과
코너 회전이 yaw 를 바꾸므로 **시작 전 값은 완료판정의 대리값이 아니다.**
그리고 사람이 `grep -A1` 출력을 눈으로 읽어 ±0.3 을 판단하게 돼 있었다 —
촬영 중에 암산할 일이 아니다.

🔴 **wrap 을 안 다루면 안 된다.** yaw 는 [-π, π] 로 감기므로 `+3.10` 과 `-3.10`
   은 0.08 rad 차이지 6.2 rad 차이가 아니다. 이 도구는 그 차이를 각도로 잰다.

판정:
    rc=0  🟢 진입해도 된다 — 약 180° 가 나온다
    rc=1  🔴 **유효한 값의 불통과** — 그 자리에서 사람을 멈추지 말 것
    rc=2  🔴 **판정 불능** — TF 를 못 읽었거나 인자가 비유한·범위 밖이다.
          통과로 읽지 않는다. (§84.7 — 구판은 잘못된 인자도 rc=1 로 섞었다)
"""

import argparse
import math
import sys


def wrap(a):
    """[-π, π] 로 정규화."""
    return math.atan2(math.sin(a), math.cos(a))


def angle_gap(yaw, target):
    """두 각의 최단 차이 [rad]. 🔴 wrap 을 반드시 통과시킨다."""
    return abs(wrap(yaw - target))


def check_args(target, tol, timeout):
    """🔴 §84.7 — 인자 자체가 판정 불능이면 rc=1 이 아니라 rc=2 다.

    구판은 `--tol nan` · `--target nan` · `--tol -0.3` 을 전부 *"여기서 멈추지
    말 것"*(rc=1)으로 끝냈다. rc=1 은 **유효한 값의 불통과**이고 rc=2 는
    **판정 불능**이라는 머리말 계약과 갈렸다 — 촬영 기록 해석이 틀어진다.
    반환: 사유 목록(빈 목록 = 통과)."""
    out = []
    if not math.isfinite(target):
        out.append(f'--target 이 유한값이 아니다 ({target!r})')
    if not math.isfinite(tol):
        out.append(f'--tol 이 유한값이 아니다 ({tol!r})')
    elif tol <= 0:
        out.append(f'--tol 은 양수여야 한다 ({tol!r})')
    elif tol > math.pi:
        out.append(f'--tol {tol!r} 은 π 를 넘어 어떤 각도든 통과시킨다 (무의미)')
    if not math.isfinite(timeout) or timeout <= 0:
        out.append(f'--timeout 은 양수 유한값이어야 한다 ({timeout!r})')
    return out


def verdict(yaw, target, tol):
    """(통과?, 차이[rad]). 순수 함수 — 이 자리가 단위시험 대상이다.

    🔴 §84.7 — 비유한 입력은 여기서도 방어한다. `check_args` 를 우회해 부르는
    호출자가 생겨도 NaN 이 조용히 '불통과' 로 둔갑하지 않게."""
    if not (math.isfinite(yaw) and math.isfinite(target) and math.isfinite(tol)):
        return None, float('nan')
    gap = angle_gap(yaw, target)
    return gap <= tol, gap


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def read_yaw(frame_from, frame_to, timeout):
    """TF 에서 현재 yaw 를 읽는다. 실패하면 None."""
    import rclpy
    from rclpy.node import Node
    import tf2_ros
    rclpy.init()
    node = Node('yaw_gate')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    out = None
    try:
        while node.get_clock().now().nanoseconds / 1e9 < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                tr = buf.lookup_transform(frame_from, frame_to, rclpy.time.Time())
            except Exception:
                continue
            q = tr.transform.rotation
            out = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--target', type=float, default=math.pi,
                    help='기대 yaw [rad] (기본 π = 서쪽을 보고 있다)')
    ap.add_argument('--tol', type=float, default=0.3, help='허용 오차 [rad]')
    ap.add_argument('--frame', default='map')
    ap.add_argument('--child', default='base_footprint')
    ap.add_argument('--timeout', type=float, default=5.0)
    ap.add_argument('--yaw', type=float,
                    help='TF 대신 이 값으로 판정 (시험·연습용)')
    a = ap.parse_args()

    bad = check_args(a.target, a.tol, a.timeout)
    if bad:
        for m in bad:
            print(f'🔴 인자 거부 — {m}')
        print('   판정 불능(rc=2). **통과로 읽지 말 것.**')
        return 2

    yaw = a.yaw if a.yaw is not None else read_yaw(a.frame, a.child, a.timeout)
    if yaw is None or not math.isfinite(yaw):
        print(f'🔴 TF 를 못 읽었다 ({a.frame}→{a.child}, {a.timeout}s). '
              f'판정 불능 — **통과로 읽지 말 것.** 위치추정이 살아 있는지 먼저 볼 것.')
        return 2

    ok, gap = verdict(yaw, a.target, a.tol)
    if ok is None:
        print('🔴 판정 불능 — 비유한 입력. **통과로 읽지 말 것.**')
        return 2
    deg = math.degrees(gap)
    print(f'현재 yaw = {yaw:+.3f} rad ({math.degrees(yaw):+.1f}°) · '
          f'목표 {a.target:+.3f} · 차이 {gap:.3f} rad ({deg:.1f}°) · 허용 {a.tol}')
    if ok:
        print('🟢 진입해도 된다 — 여기서 사람을 멈추면 약 180° 가 나온다.')
        return 0
    print('🔴 여기서 사람을 멈추지 말 것.')
    print('   목표 yaw 는 0.0 하드코딩이라(예약 58) 회전량은 **지금 yaw** 가 정한다.')
    print(f'   지금 멈추면 약 {math.degrees(abs(wrap(yaw))):.0f}° 회전이 나온다.')
    print('   → 직선 구간을 더 진행해 yaw 가 π 근처가 된 뒤 다시 실행할 것.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
