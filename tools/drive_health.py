#!/usr/bin/env python3
"""구동계 10초 점검 — 한쪽 엔코더가 덜 세고 있나.

사용 (🔴 무장돼 있어야 한다):
    python3 tools/drive_health.py              # 4초 회전하며 잰다
    python3 tools/drive_health.py --straight 8 # 🔵 8초 직진 — 가장 잘 갈리는 시험
    python3 tools/drive_health.py --sec 6      # 회전을 더 길게
    python3 tools/drive_health.py --watch      # 안 돌리고 지금 흐르는 값만 본다

왜 이 도구가 있나
-----------------
08-21 실차에서 회전이 하루 종일 느려졌고, 구현자는 그것을 **배터리 탓으로 오진**했다.
`/odom` 으로 쟀기 때문이다. 뒤늦게 `/imu/data` 와 대조해보니 진실이 반대였다:

    M2 14:00   odom 0.1283 ≈ IMU 0.1262   오차  1.6%   🟢 로봇 정상
    20:00      odom 0.0965  vs IMU 0.1456  오차 34%    🔴 **오도메트리가 고장**

즉 **로봇은 느려지지 않았고 오도메트리가 덜 세고 있었다.**

🔴 그리고 그날 밤 bag 을 되짚어 원인이 나왔다 — **미끄러짐이 아니었다.** 펌웨어는
한쪽당 엔코더 **2개를 평균**한다(`.ino` deltaRight = 0.5*(dFR+dRR)). 그래서 한쪽의
엔코더 하나가 0 이면 **그 쪽이 절반**으로 읽히고, 그 절반은 직진에도 회전에도
**같은 배율**로 나타난다. 21:32 리허설에서 오른쪽 계수가 **0.525** 로 나왔다.

이 도구의 첫 문구는 "엔코더·커플러·축이 미끄러진다" 였다. 그건 과잉 진단이었고,
**로봇을 뜯게 만드는 문구**였다. 먼저 볼 곳은 배선이다.

두 가지를 따로 잰다
-------------------
① **오도메트리 결손**  odom ω vs IMU ω. 자이로는 바퀴와 무관하니 자이로가 진짜다.
② **좌우 비대칭**      제자리 회전 명령인데 `odom linear.x` 가 0 이 아니면
                       좌우 바퀴 속도가 다르다. 부호로 어느 쪽이 약한지 갈린다.
   실측: 정상 -0.0002 m/s → 이상 -0.0160 m/s (예약 44 = 우전륜 유격과 같은 자리)

🔵 **`--straight` 가 가장 잘 가른다** — 지면 진실도 IMU 도 필요 없다. 직진 명령만
주면 좌우가 같아야 하므로, `/odom` 이 스스로 모순을 드러낸다:

    r  = odomω · BASE / (2 · odom속도)  =  (kR − kL)/(kR + kL)
    kL = 1 로 두면   kR = (1 − |r|)/(1 + |r|)

08-21 실측을 넣으면 r = −0.311 → **kR = 0.525**. 직진인데 odom 이 "휘는 중"이라고
말하면(그날 −0.058 rad/s) 그게 지문이다. 30초만 직진해도 없는 회전 100°를 지어낸다.

🔴 범인 바퀴 이름까지 가려면 `tools/drive_encoder_check.py --wheels=FR` 로 **한 바퀴씩**
굴린 bag 을 찍는다. 이 도구는 "어느 쪽이 얼마나" 까지다.

🔴 이 도구는 로봇을 움직인다. 반경 1 m(직진은 앞 2 m)를 비우고 E-stop 을 쥔다.
"""

import argparse
import math
import os
import statistics as st

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

# 🔴 08-21 — 구동부(micro-ROS)는 BEST_EFFORT 로 발행한다. RELIABLE 구독자는
#   **한 건도 못 받고**, 그게 에러가 아니라 경고 한 줄로 지나간다.
#   같은 함정 = `tools/wall_align.py` · 같은 패턴 = `tools/nav2_preflight.py:48`.
BEST_EFFORT = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

SELF = os.path.abspath(__file__)
TURN_RATE = 0.45        # rad/s — 회전 불감대(0.2329) 를 확실히 넘는 값
TRACK = 0.49            # m — URDF 의 바퀴 간격 (robot_real.urdf: y=±0.245)
# 🔴 odom 계열 상수다 (`.ino` ODOM_WHEEL_BASE). 명령 경로의 0.62 도, 물리 0.49 도 아니다.
#   회귀(`test_drive_health.py`)가 `.ino` 에서 읽어 대조한다 — 베껴 적고 잊는 것을 막는다.
# 🔴 2026-08-22 재교정 0.829 -> 0.859 (`.ino` 와 한 쌍. 회귀가 `.ino` 에서 읽어 대조한다).
#   근거 = 08-22 실측: 줄자 직진 2회가 반지름을 지지(0.9936·0.9919) · 제자리 회전
#   3회에서 odom/IMU = 1.0431. 정본 = docs/REAL_ROBOT_VALUES.md §1-c.
ODOM_WHEEL_BASE = 0.859

# 🔴 08-22 재교정 **이전**에 찍은 bag 은 이 값으로 풀어야 한다 — 그때 펌웨어가 쓰던
#   눈금이기 때문이다. 새 값(0.859)으로 옛 bag 을 풀면 kR 이 조용히 어긋난다
#   (08-21 21:32 리허설이 0.525 -> 0.507 로 읽힌다). `--pre-0822` 로 고른다.
WHEEL_BASE_PRE_0822 = 0.829
STRAIGHT_SPEED = 0.10   # m/s — M1 에서 안전이 증명된 속도 (guide_speed 와 같다)
MAX_STRAIGHT_SEC = 20.0
BAL_OK = 0.05           # |r| 이 이 아래면 좌우 균형 정상
MAX_SEC = 10.0

# 판정선 — 08-21 실측 기준
SLIP_OK = 10.0          # odom/IMU 오차 [%] 이 아래면 정상 (실측 정상 1.6%)
ASYM_OK = 0.005         # |odom linear.x| [m/s] (실측 정상 0.0002 · 이상 0.0160)


def verdict(slip_pct, asym, omega):
    print()
    ok = True
    if abs(slip_pct) <= SLIP_OK:
        print(f'  🟢 엔코더  오차 {slip_pct:+.1f}%  (정상 ±{SLIP_OK:.0f}% 이내)')
    else:
        ok = False
        which = '덜 센다' if slip_pct > 0 else '더 센다'
        print(f'  🔴 엔코더  오차 {slip_pct:+.1f}% — 오도메트리가 회전을 {which}')
        # 🔴 08-21 — 구판은 여기서 "미끄러진다, 조임 점검" 이라고 단정했다. 실제 원인은
        #   한쪽 엔코더가 0 을 내는 것이었고, 그건 조여서 고치는 물건이 아니었다.
        print('     → 한쪽 엔코더가 덜 세고 있을 수 있다. **--straight 로 어느 쪽인지 가른다.**')
    if abs(asym) <= ASYM_OK:
        print(f'  🟢 좌우    linear.x {asym:+.4f} m/s  (제자리 회전이면 0)')
    else:
        ok = False
        # vl = d - ωL/2 · vr = d + ωL/2 → d·ω 의 부호가 약한 쪽을 가른다
        weak = '오른쪽' if asym * omega < 0 else '왼쪽'
        print(f'  🔴 좌우    linear.x {asym:+.4f} m/s — **{weak}이 약하다**')
        print(f'     → 제자리로 안 돌고 호를 그린다. 예약 44(우전륜 유격) 참조.')
    print()
    print('  🟢 구동계 정상' if ok else '  🔴 조임·결합 점검이 필요하다')
    return 0 if ok else 1


def straight_verdict(lin, ow, iw, base=ODOM_WHEEL_BASE):
    """직진 명령만 준 구간에서 좌우 계수를 역산한다. `/odom` 하나면 된다.

    직진이면 좌우가 같아야 하므로, 한쪽이 덜 세면 `/odom` 이 **스스로 모순**을 낸다:
    속도는 낮아지고(둘의 평균) 동시에 없는 회전이 생긴다(둘의 차). 그 둘의 비가
    지면 진실 없이 계수를 준다.

        odom속도 = v(kL + kR)/2        odomω = v(kR − kL)/BASE
        r = odomω·BASE/(2·odom속도) = (kR − kL)/(kR + kL)      ← v 가 지워진다

    성한 쪽을 1 로 두면 약한 쪽 계수 = (1 − |r|)/(1 + |r|).
    🔴 **0.5 근처면 그 쪽 엔코더 2개 중 하나가 0 을 내고 있다** (펌웨어가 평균하므로).
    """
    print()
    if lin < 0.02:
        print(f'  🔴 판정 불가 — odom 직진 속도가 {lin:.4f} m/s 뿐이다.')
        print('     안 움직였거나 불감대 아래다. 무장·바닥·E-stop 을 본다.')
        return 1
    r = ow * base / (2.0 * lin)
    print(f'  odom 속도 {lin:.4f} m/s · odom ω {ow:+.4f} · IMU ω {iw:+.4f} rad/s')
    print(f'  좌우 불균형 r = {r:+.4f}   (직진이면 0 이어야 한다)')
    if abs(iw) > 0.05:
        print(f'  ⚠ IMU 도 {iw:+.4f} 라 실제로 휘었다 — 곧게 못 갔으면 다시 잰다.')
    if abs(r) <= BAL_OK:
        print(f'  🟢 좌우 균형 정상 (|r| ≤ {BAL_OK})')
        return 0
    weak = '오른쪽' if r < 0 else '왼쪽'
    k = (1.0 - abs(r)) / (1.0 + abs(r))
    print(f'  🔴 **{weak}이 {k:.3f} 배로 읽힌다** (성한 쪽을 1.0 으로 뒀을 때)')
    # 🔴 08-22 (§87.3) — 여기서 "엔코더 하나가 안 센다" 고 **단정하지 않는다.**
    #   이 값은 `/odom` 의 좌우 **합성 계측 비율**이고, 같은 비율을 내는 원인이
    #   여럿이다: 한 채널 dead(1.0, 0.0) · 두 채널이 같이 낮음(0.52, 0.52) ·
    #   부분/간헐 pulse loss · 그 쪽 공통 배선·기어·샘플링. 본문은 "어느 쪽이
    #   얼마나" 라고 말하면서 판정문만 단정해 서로 충돌했다.
    if abs(k - 0.5) < 0.12:
        print(f'     → 0.5 에 가깝다. **선두 가설 = {weak} 엔코더 2개 중 하나가 0.**')
        print(f'     🔴 확정은 아니다 — 같은 비율을 두 채널이 함께 낮아도 만든다.')
        print(f'     → `drive_encoder_check.py --wheels=FR` / `--wheels=RR` 로')
        print(f'        한 바퀴씩 갈라 **0 구간이 나오는 채널**을 본 뒤에 확정한다.')
    else:
        print('     → 절반과는 다르다. 부호 반전이나 부분 결손일 수 있다.')
    print('     🔵 어느 경우든 굽기 전에 배선부터: 커넥터·A/B·전원·GND, 그다음 커플러.')
    print(f'     🔵 범인 바퀴 이름은 한 바퀴씩 굴려 가른다:')
    print(f'        ros2 bag record /odom -o enc_FR_$(date +%m%d_%H%M)')
    print(f'        python3 tools/drive_encoder_check.py <bag> --wheels=FR')
    return 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sec', type=float, default=4.0, help='회전 시간 [s]')
    ap.add_argument('--watch', action='store_true', help='안 돌리고 관측만')
    ap.add_argument('--straight', type=float, metavar='SEC', default=0.0,
                    help=f'회전 대신 직진 N초 ({STRAIGHT_SPEED} m/s) — 좌우를 가른다')
    a = ap.parse_args()
    if a.sec > MAX_SEC:
        print(f'🔴 {a.sec}s 는 너무 길다 (상한 {MAX_SEC}s)')
        return 1
    if a.straight > MAX_STRAIGHT_SEC:
        print(f'🔴 직진 {a.straight}s 는 너무 길다 (상한 {MAX_STRAIGHT_SEC}s '
              f'= 약 {MAX_STRAIGHT_SEC * STRAIGHT_SPEED:.1f} m)')
        return 1

    rclpy.init()
    node = rclpy.create_node('drive_health')
    odo, imu, diag = [], [], []
    node.create_subscription(Odometry, '/odom',
                             lambda m: odo.append((m.twist.twist.linear.x,
                                                   m.twist.twist.angular.z)),
                             BEST_EFFORT)
    node.create_subscription(Imu, '/imu/data',
                             lambda m: imu.append(m.angular_velocity.z), BEST_EFFORT)
    node.create_subscription(Vector3, '/drive/diag',
                             lambda m: diag.append(m.z), BEST_EFFORT)

    def spin(sec):
        end = node.get_clock().now().nanoseconds + int(sec * 1e9)
        while node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        spin(2.0)
        if not odo or not imu:
            print(f'🔴 /odom {len(odo)}건 · /imu/data {len(imu)}건 — 스택 확인')
            return 1

        if a.watch:
            print('관측만 (로봇을 안 돌린다) — 지금 흐르는 값:')
            spin(3.0)
            ow = st.median([abs(w) for _, w in odo])
            iw = st.median([abs(w) for w in imu])
            print(f'  odom |ω| {ow:.4f}   IMU |ω| {iw:.4f}   linear.x '
                  f'{st.median([v for v, _ in odo]):+.4f}')
            print('  ⚠ 정지 중이면 둘 다 0 이라 판정이 안 된다. 회전이 필요하다.')
            return 0

        if not diag or abs(diag[-1] - 2.0) > 0.1:
            z = diag[-1] if diag else float('nan')
            print(f'🔴 무장 안 됨 (z={z:.1f}, 2.0 이어야 함). 회전하지 않는다.')
            print('   ros2 service call /drive/enable std_srvs/srv/SetBool "{data: true}"')
            return 1

        pub = node.create_publisher(Twist, '/cmd_vel', 10)

        if a.straight > 0.0:
            fwd = Twist()
            fwd.linear.x = STRAIGHT_SPEED
            print(f'🔴 직진 {a.straight:.1f}초 · {STRAIGHT_SPEED} m/s '
                  f'(약 {a.straight * STRAIGHT_SPEED:.1f} m) — 앞을 비웠는지 확인')
            odo.clear(); imu.clear()
            end = node.get_clock().now().nanoseconds + int(a.straight * 1e9)
            while node.get_clock().now().nanoseconds < end:
                pub.publish(fwd)
                rclpy.spin_once(node, timeout_sec=0.05)
            for _ in range(6):
                pub.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.05)
            # 🔴 가속 구간을 버린다 — 앞 1/3 은 아직 명령 속도가 아니다
            cut = max(1, len(odo) // 3)
            lin = st.median([r[0] for r in odo[cut:]]) if len(odo) > cut else 0.0
            ow = st.median([r[1] for r in odo[cut:]]) if len(odo) > cut else 0.0
            iw = st.median(imu[len(imu) // 3:]) if imu else 0.0
            print(f'\n  표본  odom {len(odo)}건 · IMU {len(imu)}건 (앞 1/3 버림)')
            return straight_verdict(lin, ow, iw)

        msg = Twist()
        msg.angular.z = TURN_RATE
        print(f'🔴 제자리 회전 {a.sec:.1f}초 · z=+{TURN_RATE} — 반경 1 m 를 비웠는지 확인')
        odo.clear(); imu.clear()
        end = node.get_clock().now().nanoseconds + int(a.sec * 1e9)
        while node.get_clock().now().nanoseconds < end:
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
        stop = Twist()
        for _ in range(6):
            pub.publish(stop)
            rclpy.spin_once(node, timeout_sec=0.05)

        # 🔴 가속 구간을 버린다 — 앞 30% 는 불감대를 넘느라 둘 다 0 에 가깝다
        cut = max(1, len(odo) // 3)
        ow = st.median([abs(w) for _, w in odo[cut:]]) if len(odo) > cut else 0.0
        iw = st.median([abs(w) for w in imu[len(imu)//3:]]) if imu else 0.0
        asym = st.median([v for v, _ in odo[cut:]]) if len(odo) > cut else 0.0
        print(f'\n  표본  odom {len(odo)}건 · IMU {len(imu)}건 (앞 1/3 버림)')
        print(f'  odom |ω| {ow:.4f} rad/s   IMU |ω| {iw:.4f} rad/s '
              f'= {math.degrees(iw):.2f} °/s')
        slip = 100.0 * (1 - ow / iw) if iw > 1e-6 else 0.0
        return verdict(slip, asym, TURN_RATE)
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
