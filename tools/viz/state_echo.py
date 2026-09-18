#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""④-b  `ros2 topic echo /mission_state` 재생 — 아래로 흐른다.

실제 bag 의 `/mission_state` **전량**(realtake6 = 636건 · 2.0 Hz)을 그대로 흘린다.
진짜 echo 처럼 **화면을 지우지 않고 덧붙인다** — 터미널이 스스로 스크롤한다.

  python3 ~/ros2_ws/tools/viz/state_echo.py --list
  python3 ~/ros2_ws/tools/viz/state_echo.py --at 239     # 그 시점까지 찍고 멈춤
  python3 ~/ros2_ws/tools/viz/state_echo.py --speed 4
  python3 ~/ros2_ws/tools/viz/state_echo.py --plain      # 색 없이 (진짜 echo 와 동일)

🔴 화면 녹화가 아니라 재생이다.
"""
import sys, os, bisect, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _replay_common as C

COL = {'HOLD': C.YEL, 'SEARCH_BACK': C.RED, 'FAULT': C.RED, 'BLOCKED': C.RED}

def emit(val, changed, plain):
    if plain:
        print(f'data: {val}'); print('---')
    else:
        c = COL.get(val, C.GRN) if changed else ''
        print(f'{c}data: {val}{C.R}')
        print(f'{C.GRAY}---{C.R}')

def main():
    ap = C.cli('ros2 topic echo /mission_state 재생')
    ap.add_argument('--plain', action='store_true', help='색 없이 — 진짜 echo 와 같은 출력')
    ap.add_argument('--tail', type=int, default=24, help='--at 일 때 보여줄 건수')
    a = ap.parse_args()
    if a.list: C.show_list(); return 0

    print(f'{"" if a.plain else C.DIM}$ ros2 topic echo /mission_state{"" if a.plain else C.R}')
    if a.at is not None:
        i = bisect.bisect_right(C.RW_T, a.at + C.MT0)
        seg = C.RAW[max(0, i - a.tail):i]
        prev = C.RAW[max(0, i - a.tail) - 1][1] if max(0, i - a.tail) > 0 else None
        for _, v in seg:
            emit(v, v != prev, a.plain); prev = v
        return 0

    t0, t1 = C.span(a)
    i = bisect.bisect_right(C.RW_T, t0)
    prev = C.RAW[i-1][1] if i > 0 else None
    t = t0
    try:
        while t <= t1:
            j = bisect.bisect_right(C.RW_T, t)
            for _, v in C.RAW[i:j]:
                emit(v, v != prev, a.plain); prev = v
            i = j
            time.sleep(1.0 / a.fps); t += a.speed / a.fps
    except KeyboardInterrupt: pass
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
