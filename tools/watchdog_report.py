#!/usr/bin/env python3
"""R0 watchdog(`TODO(D+0) #11`) 판정용 — bag 에서 정지 시각을 기계로 재계산한다.

정본 = `docs/JETSON_SETUP.md §7-c-0`.

★ 이 도구는 §7-c-0 의 **1차 증거(영상)를 대체하지 않는다.** `/odom.pose` 는 펌웨어가
엔코더를 적분해 만든 값이라, 지금 시험 대상인 그 펌웨어와 독립적인 관측이 아니다.
영상이 없을 때 **교차 증거를 기계 타임스탬프로 재현**하는 용도이며, 결과를 쓸 때는
"영상 미분석"을 함께 적는다.

사용법:
    python3 tools/watchdog_report.py <bag 디렉터리> [...]
    python3 tools/watchdog_report.py ~/Desktop/d0_evidence/d0_watchdog_*

2026-08-07 실측(3회) = 519.9 / 532.0 / 537.1 ms. 판정은 §7-c-0 참조 —
펌웨어 `WATCHDOG_TIMEOUT_MS = 500` 이 500ms 에 *발동*하므로 총 정지 시각은 항상
500ms 보다 크다. 기준 자체의 결함이며 재정의는 사용자 결정 + 검토자 확인 사항이다.
"""
import glob
import math
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# 엔코더 양자화 잡음과 실이동을 가르는 임계. 바퀴 반경 0.053 m 기준 충분히 작다.
MOVE_EPS_MM = 0.5
WATCHDOG_CONTRACT_MS = 500


def load(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cmds, odoms = [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/cmd_vel':
            m = deserialize_message(data, get_message(tmap[topic]))
            cmds.append((t, m.linear.x, m.angular.z))
        elif topic == '/odom':
            m = deserialize_message(data, get_message(tmap[topic]))
            p = m.pose.pose.position
            odoms.append((t, p.x, p.y, m.twist.twist.linear.x))
    return cmds, odoms


def report(bag, verbose=True):
    print('=' * 78)
    print('BAG:', os.path.basename(bag.rstrip('/')))
    cmds, odoms = load(bag)
    print(f'  /cmd_vel {len(cmds)}건 · /odom {len(odoms)}건')
    if not cmds or not odoms:
        print('  판정 불가 — 필요한 토픽이 비었다')
        return None

    dur = (odoms[-1][0] - odoms[0][0]) / 1e9
    print(f'  녹화 {dur:.1f}s · /odom 평균 {len(odoms) / dur:.1f} Hz')

    nonzero = [c for c in cmds if abs(c[1]) > 1e-9 or abs(c[2]) > 1e-9]
    if not nonzero:
        print('  판정 불가 — 비영 명령이 없다')
        return None
    zero_after = [z[0] for z in cmds
                  if abs(z[1]) <= 1e-9 and abs(z[2]) <= 1e-9 and z[0] > nonzero[-1][0]]
    if zero_after:
        gap = (min(zero_after) - nonzero[-1][0]) / 1e6
        print(f'  ⚠ 마지막 비영 명령 뒤 {gap:.0f} ms 에 zero 가 들어왔다 '
              f'— 이 창 안에서만 watchdog 을 판정할 수 있다')

    last_cmd = nonzero[-1][0]
    prev, last_move, tail = None, None, 0.0
    if verbose:
        print('\n   t_rel(ms)      x(m)      y(m)   |d|(mm)   twist.x')
    for (t, x, y, vx) in odoms:
        rel = (t - last_cmd) / 1e6
        if rel < -150 or rel > 2600:
            prev = (x, y)
            continue
        d = 0.0 if prev is None else math.hypot(x - prev[0], y - prev[1]) * 1000
        mark = ''
        if d > MOVE_EPS_MM:
            last_move = rel
            mark = '  <-- 이동'
        if verbose:
            print(f'  {rel:9.1f}  {x:8.4f}  {y:8.4f}  {d:8.3f}  {vx:8.4f}{mark}')
        prev, tail = (x, y), rel

    print()
    if last_move is None:
        print('  >>> 마지막 명령 이후 pose 변화 없음 (임계 0.5mm)')
        return None
    over = last_move - WATCHDOG_CONTRACT_MS
    print(f'  >>> 마지막 pose 이동 = +{last_move:.1f} ms '
          f'(계약 {WATCHDOG_CONTRACT_MS}ms 대비 {over:+.1f} ms)')
    print(f'  >>> 정지 후 관찰 창 = {tail - last_move:.0f} ms (§7-c-0 조건 2 는 2000ms 이상)')
    return last_move


def main(argv):
    targets = argv[1:]
    if not targets:
        targets = sorted(glob.glob(os.path.expanduser(
            '~/Desktop/d0_evidence/d0_watchdog_*')))
    if not targets:
        print('bag 디렉터리를 인자로 주거나 ~/Desktop/d0_evidence/ 에 두어라', file=sys.stderr)
        return 2
    for bag in targets:
        report(bag)
        print()
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
