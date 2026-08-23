#!/usr/bin/env python3
"""미션 상태를 촬영용으로 크게 띄운다 — `ros2 topic echo` 대신.

사용:
    python3 tools/mission_hud.py

왜 이 도구가 있나
-----------------
`ros2 topic echo /mission_state` 는 YAML 이 줄줄 흘러서 **화면에 찍히면 안 읽힌다.**
08-22~23 촬영에서 노트북 화면을 같이 찍기로 했으므로, 상태 하나를 크게 보여주고
바뀔 때만 갱신한다. 로그가 아니라 **계기판**이다.

🔴 로봇을 움직이지 않는다. 구독만 한다.
"""

import sys
import time

import rclpy
from std_msgs.msg import Bool, String

# 상태별 한 줄 설명 — 화면을 보는 사람이 무슨 일인지 알게
MEAN = {
    'PATROL':      '평시 순찰 중',
    'APPROACH':    '화재 감지 — 집결지로 출동',
    'SCAN_AREA':   '집결지 360° 훑기 — 대피자 탐색',
    'HOLD':        '추종자 놓침 — 제자리에서 재수집',
    'GATHER':      '집결 대기 — 대피자를 모으는 중',
    'GUIDE':       '저속 선행 유도 — 후방 감시 중',
    'SEARCH_BACK': '추종자 놓침 — 역행 재탐색',
    'ESCAPED':     '탈출 완료',
    'FAULT':       '주행 실패 — 재시도 중',
    'BLOCKED':     '안전한 집결지 없음 — 사람 판단 대기',
}
# 촬영에서 눈에 띄어야 하는 상태
ALERT = {'SEARCH_BACK', 'FAULT', 'BLOCKED'}

BIG = {
    'A': ['█▀█', '█▀█', '▀ ▀'], 'B': ['█▀▄', '█▀▄', '▀▀ '],
    'C': ['█▀▀', '█  ', '▀▀▀'], 'D': ['█▀▄', '█ █', '▀▀ '],
    'E': ['█▀▀', '█▀▀', '▀▀▀'], 'F': ['█▀▀', '█▀▀', '▀  '],
    'G': ['█▀▀', '█ █', '▀▀▀'], 'H': ['█ █', '█▀█', '▀ ▀'],
    'I': ['█', '█', '▀'],       'K': ['█ █', '█▀▄', '▀ ▀'],
    'L': ['█  ', '█  ', '▀▀▀'], 'N': ['█▄█', '█▀█', '▀ ▀'],
    'O': ['█▀█', '█ █', '▀▀▀'], 'P': ['█▀█', '█▀▀', '▀  '],
    'R': ['█▀█', '█▀▄', '▀ ▀'],
    'S': ['█▀▀', '▀▀█', '▀▀▀'], 'T': ['▀█▀', ' █ ', ' ▀ '],
    'U': ['█ █', '█ █', '▀▀▀'], '_': ['   ', '   ', '▀▀▀'],
}


def render(state, since, siren):
    rows = ['', '', '']
    for ch in state:
        g = BIG.get(ch, ['?', '?', '?'])
        for i in range(3):
            rows[i] += g[i] + ' '
    w = max(len(r) for r in rows) + 4
    w = max(w, 44)
    bar = '═' * w
    mark = '🔴' if state in ALERT else '🔵'
    print('\033[2J\033[H', end='')          # 화면 지우고 커서 맨 위로
    print(f'╔{bar}╗')
    print(f'║{" " * w}║')
    for r in rows:
        print(f'║  {r.ljust(w - 2)}║')
    print(f'║{" " * w}║')
    print(f'╚{bar}╝')
    print()
    print(f'  {mark}  {MEAN.get(state, "알 수 없는 상태")}')
    print()
    m, s = divmod(int(since), 60)
    print(f'      이 상태 유지 {m:02d}:{s:02d}      '
          f'사이렌 {"🔊 ON " if siren else "   OFF"}')
    sys.stdout.flush()


def main():
    rclpy.init()
    node = rclpy.create_node('mission_hud')
    box = {'state': '(대기)', 'since': time.time(), 'siren': False, 'dirty': True}

    def on_state(msg):
        if msg.data != box['state']:
            box['state'] = msg.data
            box['since'] = time.time()
            box['dirty'] = True

    def on_siren(msg):
        if msg.data != box['siren']:
            box['siren'] = msg.data
            box['dirty'] = True

    node.create_subscription(String, '/mission_state', on_state, 10)
    node.create_subscription(Bool, '/siren', on_siren, 10)

    last = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.time()
            # 초 표시가 흐르도록 1초마다, 상태가 바뀌면 즉시 다시 그린다
            if box['dirty'] or now - last >= 1.0:
                render(box['state'], now - box['since'], box['siren'])
                box['dirty'] = False
                last = now
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
