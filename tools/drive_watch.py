#!/usr/bin/env python3
"""주행 중 감시 — **라이다 사망 + 구동계 재발**을 테이크 안에서 잡는다 (2026-08-22).

사용 (로봇을 움직이지 않는다 · 보기만 한다):
    python3 tools/drive_watch.py

★ **구동계 — 기준은 IMU 다** (08-22 §89.6 재설계)

🔴 첫 판은 `cmd_vel` 이 직진일 때 `odom ω` 가 0 인지를 봤다. **두 군데가 틀렸다:**
  ① 정상 곡선(`cmd_ω=0.019`)에서 **124회 오경보** — `odom ω` 를 0 과 비교했는데
     명령 곡률이 있으면 당연히 0 이 아니다. 멀쩡한 테이크를 버린다.
  ② 고치려고 `odom_ω ≈ (CMD_BASE/ODOM_BASE)·cmd_ω` 모델을 세웠는데, 실측 비율이
     **0.122 / 0.973** 로 이론값 0.748 과 전혀 안 맞았다 — 불감대·지연·슬립 때문에
     `cmd_ω → odom_ω` 는 깨끗한 상수가 아니다. **cmd_vel 을 기준 삼는 설계가 틀렸다.**
  ⚠ 그리고 `|cmd_ω| < 0.005` 로 조이는 길은 막혔다 — 실측에서 전진 명령의 **6.2%**
     뿐이다(Nav2 RPP 가 곡률 보정을 계속 낸다, 중앙값 0.0468).

🔵 **깨끗한 기준은 이미 있다 — 자이로다.** 08-21 진단을 그걸로 했다: 자이로는 바퀴와
물리적으로 무관하므로 `odom ω` 와 어긋나면 그 차이가 곧 오도메트리 오차다.

    |odom_ω − IMU_ω| > max(0.02, 0.15·|IMU_ω|)      ← 판정식

08-21 실측 네 경우가 모두 옳게 갈린다:

    정상 직진 odom +0.0031 / IMU +0.0031 → 차 0.000  🟢
    고장 직진 odom −0.0580 / IMU +0.0058 → 차 0.064  🔴
    정상 회전 odom +0.1283 / IMU +0.1262 → 차 0.002  🟢
    고장 회전 odom +0.0965 / IMU +0.1456 → 차 0.049  🔴

🔵 `cmd_vel` 을 안 보므로 **직진·곡선·회전 어디서나** 돈다 — 테이크 커버리지가 넓다.

★ **라이다 사망도 같이 본다** (지금 실차의 주 증상)

`PITFALLS §17-①` — 라이다는 **프로세스가 살아 있는 채로 데이터만 끊는다.** 로그에
에러가 0줄이라 사람이 알아채는 것은 *"로봇이 그냥 섰다"* 뿐이다. `/scan` 이 끊기면
**그 시각을 찍어서** 소리친다 — 시각이 있어야 `dmesg` 와 대조해 USB 재열거인지 가른다.

⚠ **이 도구는 원인을 못 가른다.** 시각과 "지금 이상하다" 를 주는 것뿐이다.

정본 = `REAL_ROBOT_VALUES §1-m-11` · `PITFALLS §17-①` ·
짝 도구 = `drive_health.py`(전 점검) · `bag_drive_report.py`(사후 판독).
"""
import argparse
import os
import statistics as st

import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

# 🔴 구동부(micro-ROS)는 BEST_EFFORT 로 발행한다. RELIABLE 구독자는 한 건도 못 받고
#   그것이 에러가 아니라 경고 한 줄로 지나간다 (`PITFALLS §17`).
BEST_EFFORT = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
SELF = os.path.abspath(__file__)
# 판정선 — 08-21 실측 네 경우에서 뽑았다(위 표).
DIFF_FLOOR = 0.02      # rad/s — 정지·직진에서의 절대 문턱
DIFF_FRAC = 0.15       # 회전 중에는 |IMU ω| 의 비율로
# 🔴 창을 채우려면 최소 이 정도는 와야 한다. `/odom` 실측은 **47.3 Hz** 다
#   (3.8 Hz 는 `/detections` 이고 이 도구와 무관하다 — 내가 검토 프롬프트에 그것을
#   잘못 적어 검토자가 3.8 로 시험했다). 여유를 크게 두되, **rate 가 모자라면
#   조용히 침묵하지 말고 그 사실을 말한다** — 필요할 때 침묵하는 것이 제일 나쁘다.
MIN_HZ = 5.0
# 🆕 89.6 — 한 사건에 계속 소리치지 않는다. 복구를 보면 다시 무장한다.
LATCH = True
# 🆕 89.6 — 이 시간 동안 판정이 한 번도 성립하지 않으면 "못 보고 있다" 고 알린다.
BLIND_WARN_SEC = 20.0
# 🔴 `/scan` 이 이 시간 동안 안 오면 죽은 것으로 본다. 정상 10 Hz(0.1s) 대비 10배
#   여유다 — 더 짧게 잡으면 한 번 걸러진 프레임에 소리친다.
SCAN_TIMEOUT_DEFAULT = 1.0


DIFF_FLOOR_DOC = '정지·직진 절대 문턱 [rad/s]'


def discrepancy(odom_w, imu_w):
    """|odom ω − IMU ω| 와 그때의 문턱. 반환 (diff, threshold, bad)."""
    diff = abs(odom_w - imu_w)
    th = max(DIFF_FLOOR, DIFF_FRAC * abs(imu_w))
    return diff, th, diff > th


class Watch:
    """순수 상태기계 — ROS 없이 회귀한다.

    🔵 `cmd_vel` 을 **안 본다.** 자이로가 기준이므로 직진·곡선·회전 어디서나 판정한다.
    """

    def __init__(self, window_sec=2.0, min_hz=MIN_HZ):
        self.window = window_sec
        self.min_n = max(4, int(window_sec * min_hz))
        self.buf = []            # (t, odom_w, imu_w, odom_lin)
        self.alerts = 0
        self.latched = False     # 같은 사건에 계속 소리치지 않는다
        self.last_verdict_t = None
        self.first_t = None      # 🔴 창에 묶이지 않는 기산점

    def feed(self, t, odom_w, imu_w, odom_lin=0.0):
        """반환: 경고/복구 문자열 또는 None."""
        if self.first_t is None:
            self.first_t = t
        self.buf.append((t, odom_w, imu_w, odom_lin))
        self.buf = [b for b in self.buf if t - b[0] <= self.window]
        if len(self.buf) < self.min_n or t - self.buf[0][0] < self.window * 0.8:
            return None
        self.last_verdict_t = t
        ow = st.median([b[1] for b in self.buf])
        iw = st.median([b[2] for b in self.buf])
        lin = st.median([b[3] for b in self.buf])
        diff, th, bad = discrepancy(ow, iw)
        if not bad:
            if self.latched and LATCH:
                self.latched = False
                return f'🟢 구동계 복구 — odom ω {ow:+.3f} ≈ IMU {iw:+.3f}'
            return None
        if self.latched and LATCH:
            return None
        self.latched = True
        self.alerts += 1
        # 🔵 직진 지문일 때만 쪽을 말한다 — 회전 중에는 이 부호가 쪽을 안 가른다.
        side = ''
        if abs(iw) < DIFF_FLOOR and lin > 0.02:
            side = f' · {"오른쪽" if ow < 0 else "왼쪽"}이 덜 읽힌다'
        return (f'🔴 구동계 이상 — odom ω {ow:+.3f} vs IMU {iw:+.3f} '
                f'(차 {diff:.3f} > 문턱 {th:.3f}){side}')

    def blind_for(self, t):
        """판정이 한 번도 성립하지 않은 시간. 표본이 모자라면 조용한 게 아니라 눈먼 것이다."""
        # 🔴 창에서 재면 눈먼 시간이 **창 크기를 못 넘는다** — 낮은 발행률에서
        #   영원히 눈멀어도 "2초째" 라고 말한다. 첫 표본부터 잰다.
        if self.last_verdict_t is None:
            return t - self.first_t if self.first_t is not None else 0.0
        return t - self.last_verdict_t


class ScanWatch:
    """`/scan` 침묵 감시 — 순수 상태기계(ROS 없이 회귀한다).

    🔴 라이다는 **에러 없이** 죽는다. 그래서 "안 온다" 를 세는 것 말고 방법이 없다.
    """

    def __init__(self, timeout=SCAN_TIMEOUT_DEFAULT):
        self.timeout = timeout
        self.last = None
        self.dead_since = None
        self.deaths = 0

    def on_scan(self, t):
        msg = None
        if self.dead_since is not None:
            msg = (f'🟢 /scan 복구 — {t - self.dead_since:.1f}초 끊겼다 '
                   f'(끊긴 시각 {self.dead_since:.1f})')
            self.dead_since = None
        self.last = t
        return msg

    def check(self, t):
        """주기 점검. 반환: 경고 또는 None. 🔵 같은 사망에 한 번만 소리친다."""
        if self.last is None or self.dead_since is not None:
            return None
        if t - self.last > self.timeout:
            self.dead_since = self.last
            self.deaths += 1
            return (f'🔴 /scan 끊김 — 마지막 수신 {self.last:.1f} '
                    f'({t - self.last:.1f}초째 없음)')
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--window', type=float, default=2.0, help='판정 창 [s]')
    ap.add_argument('--scan-timeout', type=float, default=SCAN_TIMEOUT_DEFAULT,
                    help='/scan 이 이 시간 동안 없으면 죽은 것으로 본다 [s]')
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('drive_watch')
    w = Watch(a.window)
    sw = ScanWatch(a.scan_timeout)
    imu = {'w': 0.0, 'seen': False}

    def now():
        return node.get_clock().now().nanoseconds / 1e9

    def bar(lines):
        print('\n' + '=' * 62)
        for ln in lines:
            print('  ' + ln)
        print('=' * 62, flush=True)

    def on_imu(m):
        imu['w'] = m.angular_velocity.z
        imu['seen'] = True

    def on_odom(m):
        if not imu['seen']:
            return              # 🔴 자이로가 기준이다 — 없으면 판정하지 않는다
        msg = w.feed(now(), m.twist.twist.angular.z, imu['w'],
                     m.twist.twist.linear.x)
        if msg and msg.startswith('🔴'):
            bar([msg,
                 '🔴 이 테이크는 다시 찍는다. 편집에서 알면 늦다.',
                 '→ 멈추고: python3 tools/drive_health.py --straight 8',
                 '→ 🔵 재발한 지금이 채널을 잡을 기회다:',
                 '   drive_encoder_check.py <bag> --wheels=FR / --wheels=RR'])
        elif msg:
            print('\n' + msg, flush=True)

    def on_scan(_m):
        m = sw.on_scan(now())
        if m:
            print('\n' + m, flush=True)

    def tick():
        m = sw.check(now())
        if m:
            bar([m,
                 '🔴 라이다는 **에러 없이** 죽는다 — 로그는 깨끗하다.',
                 '→ 스택 재기동 말고는 복구가 없다. 이 테이크는 버린다.',
                 '→ 끊긴 시각을 적어두고 `sudo dmesg | tail -40` 로 대조한다.'])
        # 🔴 표본이 모자라 판정이 안 서는 것을 **조용함으로 착각하면 안 된다.**
        if not imu['seen']:
            node.get_logger().warn('⚠ /imu/data 가 없다 — 구동계 판정 불가',
                                   throttle_duration_sec=10.0)
        elif w.blind_for(now()) > BLIND_WARN_SEC:
            node.get_logger().warn(
                f'⚠ 구동계 판정이 {BLIND_WARN_SEC:.0f}초째 안 선다 — '
                f'/odom 표본 부족(최소 {MIN_HZ:.0f} Hz 필요)',
                throttle_duration_sec=10.0)

    node.create_subscription(Imu, '/imu/data', on_imu, BEST_EFFORT)
    node.create_subscription(Odometry, '/odom', on_odom, BEST_EFFORT)
    node.create_subscription(LaserScan, '/scan', on_scan, BEST_EFFORT)
    node.create_timer(0.2, tick)

    print(f'주행 감시 시작 — /scan 침묵({a.scan_timeout:.1f}s) + '
          f'odom ω vs IMU ω ({a.window:.1f}초 창).')
    print('🔵 로봇을 움직이지 않는다. 보기만 한다. (Ctrl+C 로 종료)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f'\n종료 — 구동계 경고 {w.alerts}회 · /scan 끊김 {sw.deaths}회')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
