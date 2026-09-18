#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""④-c  mission_manager 미션 로그 재생 — 아래로 흐른다.

  python3 ~/ros2_ws/tools/viz/mission_log.py --at 239
  python3 ~/ros2_ws/tools/viz/mission_log.py --speed 4

🔴 화면 녹화가 아니라 재생이다.
"""
import sys, os, bisect, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _replay_common as C

def colour(msg):
    if msg.startswith('★'):  return C.YEL
    if msg.startswith('🔥'): return C.ORA
    if msg.startswith('🔵'): return C.BLU
    if any(k in msg for k in ('실패','오류')): return C.RED
    return ''

def emit(lt, msg):
    v = lt - C.MT0
    ts = f'{0 if -1 < v < 1 else v:6.1f}'
    print(f'{C.GRAY}{ts}{C.R}  {colour(msg)}{msg}{C.R}')

def main():
    ap = C.cli('mission_manager 로그 재생')
    ap.add_argument('--tail', type=int, default=28, help='--at 일 때 보여줄 건수')
    a = ap.parse_args()
    if a.list: C.show_list(); return 0
    if a.at is not None:
        i = bisect.bisect_right(C.LG_T, a.at + C.MT0)
        for lt, m in C.LOG[max(0, i - a.tail):i]: emit(lt, m)
        return 0
    t0, t1 = C.span(a)
    i = bisect.bisect_right(C.LG_T, t0); t = t0
    try:
        while t <= t1:
            j = bisect.bisect_right(C.LG_T, t)
            for lt, m in C.LOG[i:j]: emit(lt, m)
            i = j
            time.sleep(1.0 / a.fps); t += a.speed / a.fps
    except KeyboardInterrupt: pass
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
