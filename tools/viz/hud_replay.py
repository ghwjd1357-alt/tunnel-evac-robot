#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""④-a  HUD 계기판만 — 진짜 터미널에 재생. 스샷은 사람이 찍는다.

`tools/mission_hud.py` 의 render() 를 그대로 import 한다 (촬영용 계기판 원본).
제자리에서 갱신되는 계기판이라 화면을 지우고 다시 그린다.

  python3 ~/ros2_ws/tools/viz/hud_replay.py --list
  python3 ~/ros2_ws/tools/viz/hud_replay.py --at 239
  python3 ~/ros2_ws/tools/viz/hud_replay.py --speed 4

🔴 화면 녹화가 아니라 재생이다. 데이터는 08-23 실차 기록 그대로지만
   그날 터미널을 캡처한 화면은 아니다.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _replay_common as C
sys.path.insert(0, os.path.join(C.REPO, 'tools'))
import mission_hud

def draw(t):
    name, since = C.state_at(t)
    mission_hud.render(name, since, C.siren_at(t))
    sys.stdout.flush()

def main():
    a = C.cli('HUD 계기판 재생').parse_args()
    if a.list: C.show_list(); return 0
    return C.loop(a, draw)

if __name__ == '__main__':
    raise SystemExit(main())
