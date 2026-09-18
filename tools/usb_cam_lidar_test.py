#!/usr/bin/env python3
"""예약 66 판정기 — 카메라를 꽂은 채 주행해도 라이다·카메라가 버티는가 (2026-09-18).

사용 (젯슨에서, 로봇 스택 + 카메라 드라이버가 떠 있는 동안):
    python3 tools/usb_cam_lidar_test.py                 # 200초 감시 후 판정
    python3 tools/usb_cam_lidar_test.py --duration 240

완료판정 (MASTER_PLAN §7 예약 66 그대로):
    "카메라를 꽂은 채 3분 이상 연속 주행해도 /scan 이 끊기지 않고,
     카메라 자신도 usb disconnect 없이 버틴다."
    + 필수 부정 회귀: **모터를 돌리면서** 재라 — 정지 상태 통과는 증거가 아니다.

그래서 넷을 같이 본다 (verdict() 순수 함수 — test_usb_cam_lidar_test.py 가 잠근다):
    ① 관측 시간  ≥ 180 s
    ② /scan 최대 공백 < 1.0 s      (drive_watch.py 의 SCAN_TIMEOUT 과 같은 값)
    ③ 커널 로그에 usb disconnect / urb -32 / reset / over-current 0 건 (감시 시작 이후)
    ④ 움직인 시간 ≥ 60 %            (/odom |v| > 0.02 m/s 또는 |ω| > 0.05 rad/s 인 초)

🔴 ③ 은 `journalctl -k -f` 를 읽는다 — adm 그룹이면 sudo 없이 된다(젯슨 hanhan 은 된다).
🔴 카메라가 `usb 2-2` 처럼 재열거되면 `disconnect` 한 줄이 남는다. 그것 하나로 FAIL 이다.
"""
import argparse, re, subprocess, sys, threading, time

SCAN_GAP_LIMIT = 1.0
MIN_DURATION = 180.0
MIN_MOVING_RATIO = 0.60
USB_BAD = re.compile(r'usb .*(disconnect|urb stopped|reset (high|full|low)-speed|over-current)|cp210x.*-32',
                     re.I)


def verdict(duration, scan_max_gap, usb_events, moving_ratio, scan_count):
    """넷을 대조해 (pass, 사유 목록) 을 낸다. 사유는 비어 있으면 통과."""
    why = []
    if scan_count == 0:
        why.append('/scan 을 한 건도 못 받았다 — 라이다가 안 떠 있다')
    if duration < MIN_DURATION:
        why.append(f'관측 {duration:.0f}s < {MIN_DURATION:.0f}s')
    if scan_max_gap >= SCAN_GAP_LIMIT:
        why.append(f'/scan 최대 공백 {scan_max_gap:.2f}s ≥ {SCAN_GAP_LIMIT}s')
    if usb_events:
        why.append(f'USB 이상 {len(usb_events)}건: ' + ' | '.join(usb_events[:3]))
    if moving_ratio < MIN_MOVING_RATIO:
        why.append(f'움직인 시간 {moving_ratio*100:.0f}% < {MIN_MOVING_RATIO*100:.0f}% — 정지 시험은 증거가 아니다')
    return (not why), why


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--duration', type=float, default=200.0, help='감시 시간 [s]')
    a = ap.parse_args()

    import rclpy                                            # noqa: PLC0415
    from rclpy.qos import qos_profile_sensor_data             # noqa: PLC0415
    from sensor_msgs.msg import LaserScan                     # noqa: PLC0415
    from nav_msgs.msg import Odometry                         # noqa: PLC0415

    st = {'scan_last': None, 'scan_gap': 0.0, 'scan_n': 0, 'move_s': set()}
    usb_events = []

    def on_scan(_m):
        t = time.monotonic()
        if st['scan_last'] is not None:
            st['scan_gap'] = max(st['scan_gap'], t - st['scan_last'])
        st['scan_last'] = t; st['scan_n'] += 1

    def on_odom(m):
        v, w = m.twist.twist.linear.x, m.twist.twist.angular.z
        if abs(v) > 0.02 or abs(w) > 0.05:
            st['move_s'].add(int(time.monotonic()))

    def tail_kernel():
        p = subprocess.Popen(['journalctl', '-k', '-f', '-n', '0', '--no-pager'],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        for line in p.stdout:
            if USB_BAD.search(line):
                usb_events.append(line.strip()[-90:])
                print(f'  🔴 USB: {line.strip()[-100:]}', flush=True)

    threading.Thread(target=tail_kernel, daemon=True).start()

    rclpy.init()
    node = rclpy.create_node('usb_cam_lidar_test')
    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', on_odom, 10)
    t0 = time.monotonic()
    print(f'예약 66 감시 시작 — {a.duration:.0f}s · 로봇을 계속 움직여라 (정지 시험은 무효)', flush=True)
    next_report = 0.0
    try:
        while time.monotonic() - t0 < a.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
            el = time.monotonic() - t0
            if el >= next_report:                    # 30초마다 한 줄 (09-18 실차: 같은 초에 여러 줄 찍혔다)
                next_report += 30.0
                # 마지막 수신 뒤 지금까지의 침묵도 공백에 넣는다 — 끊긴 채 끝나는 경우
                gap_now = (time.monotonic() - st['scan_last']) if st['scan_last'] else 0.0
                print(f'  {el:5.0f}s  scan {st["scan_n"]:5d}건 · 최대공백 {max(st["scan_gap"], gap_now):.2f}s · '
                      f'움직임 {len(st["move_s"])}s · USB이상 {len(usb_events)}', flush=True)
    except KeyboardInterrupt:
        pass
    dur = time.monotonic() - t0
    if st['scan_last'] is not None:
        st['scan_gap'] = max(st['scan_gap'], time.monotonic() - st['scan_last'])
    moving = len(st['move_s']) / max(dur, 1.0)
    ok, why = verdict(dur, st['scan_gap'], usb_events, moving, st['scan_n'])
    print('\n== 예약 66 판정 ==')
    print(f'관측 {dur:.0f}s · /scan {st["scan_n"]}건 · 최대 공백 {st["scan_gap"]:.2f}s · '
          f'움직임 {moving*100:.0f}% · USB 이상 {len(usb_events)}건')
    print('🟢 PASS — 완료판정 충족' if ok else '🔴 FAIL — ' + ' / '.join(why))
    # Ctrl+C 로 끝내면 rclpy 가 이미 종료돼 있다 — 두 번 부르면 RCLError (09-18 실차)
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
