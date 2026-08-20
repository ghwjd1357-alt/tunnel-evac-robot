#!/usr/bin/env python3
"""예약 55 — H자 **밖**을 통행금지로 칠한 keepout 마스크를 만든다.

사용:
    python3 tools/make_keepout_mask.py --out ~/Desktop/keepout_0820

왜 이 도구가 있나
-----------------
실제 터널은 H자보다 길고 **우리는 그 일부만 쓴다.** 라이다가 열린 복도 끝에서
멀리 쏜 광선이 지도에 **부채꼴 자유공간**으로 남는데, Nav2 는 그 구역을
"갈 수 있는 곳" 으로 읽는다. 우리가 밟아 본 적이 없는데도.

🔴 **`.pgm` 을 편집해서 막는 것이 아니다.** `real_bringup` 은 `map_server` 를
안 띄우고 `slam_toolbox` 가 `.posegraph` 에서 `/map` 을 만든다 — `.pgm` 은
표시용이라 항법에 영향이 0 이다. 막으려면 **costmap 필터**를 써야 한다.

이 도구는 그 필터가 먹을 마스크를 **원본 지도를 안 건드리고 새로 그린다.**
좌표·해상도·원점을 지도와 똑같이 맞추므로 겹쳐 놓으면 정확히 포개진다.

실측 기하 정본 = `docs/REAL_ROBOT_VALUES.md §1-l-4`.
"""

import argparse
import os
import sys

try:
    from PIL import Image, ImageDraw
except ImportError:
    print('🔴 Pillow 가 필요하다:  pip3 install --user Pillow')
    sys.exit(2)

# ── 지도 제원 (corridor_0820) — 마스크가 이것과 정확히 같아야 포개진다 ──
W, H = 689, 382
RES = 0.05
OX, OY = -10.2, -12.4

# ── 복도 실측 (§1-l-4) — 중심선과 폭 ──
UP_Y, LOW_Y, PASS_X = -0.08, -10.65, 8.95
CORR_W, PASS_W = 2.35, 1.60

# 🔴 밟은 범위는 x −0.19 ~ +12.99 뿐이다. 벽은 x +18 까지 그려졌지만 그건 5 m
#   넘게 떨어져서 라이다로 본 것이라 신뢰도가 다르다. 자유공간을 **밟은 범위 +
#   여유**로 자른다 — 안 밟아 본 곳으로 경로가 나가지 않게.
FREE_X_MIN, FREE_X_MAX = -1.0, 14.0
# 통로는 y 로 자른다 (§1-l-4: y −9.50 ~ −1.25)
PASS_Y_MIN, PASS_Y_MAX = -9.80, -1.00


def to_px(x, y):
    """map 좌표 → 픽셀 (col, row). 이미지는 위가 +y 라 row 를 뒤집는다."""
    return (x - OX) / RES, (H - 1) - (y - OY) / RES


def box(d, x0, y0, x1, y1, color):
    c0, r0 = to_px(x0, y1)      # y1 이 위 → row 작음
    c1, r1 = to_px(x1, y0)
    d.rectangle([c0, r0, c1, r1], fill=color)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.expanduser('~/Desktop/keepout_0820'),
                    help='확장자 없는 출력 경로 (.pgm 과 .yaml 이 생긴다)')
    ap.add_argument('--margin', type=float, default=0.10,
                    help='복도 폭에 더할 여유 [m] — 벽 근처 셀이 통행금지가 '
                         '되면 inflation 과 겹쳐 경로가 아예 안 난다')
    a = ap.parse_args()

    # 0 = 통행금지(검정), 254 = 통행가능(흰색). trinary 해석 기준.
    img = Image.new('L', (W, H), 0)
    d = ImageDraw.Draw(img)

    m = a.margin
    hw = CORR_W / 2 + m
    ph = PASS_W / 2 + m
    box(d, FREE_X_MIN, UP_Y - hw,  FREE_X_MAX, UP_Y + hw,  254)   # 위 복도
    box(d, FREE_X_MIN, LOW_Y - hw, FREE_X_MAX, LOW_Y + hw, 254)   # 아래 복도
    box(d, PASS_X - ph, PASS_Y_MIN, PASS_X + ph, PASS_Y_MAX, 254) # 연결통로

    pgm = a.out + '.pgm'
    img.save(pgm)

    free = sum(1 for p in img.getdata() if p > 127)
    print(f'마스크 {W}x{H} · res {RES} · origin ({OX}, {OY})')
    print(f'  통행가능 {free} px = {free * RES * RES:.1f} m²  '
          f'({free / (W * H) * 100:.1f}%)')
    print(f'  → {pgm}')

    yml = a.out + '.yaml'
    with open(yml, 'w', encoding='utf-8') as f:
        f.write(f"""# keepout_0820.yaml — 예약 55 통행금지 마스크
# 🔴 이 파일은 **지도가 아니다.** costmap KeepoutFilter 가 먹는 마스크다.
#    항법 지도는 여전히 slam_toolbox 가 .posegraph 에서 만든다.
# 생성 = tools/make_keepout_mask.py (실측 기하 = REAL_ROBOT_VALUES §1-l-4)
image: {os.path.basename(pgm)}
resolution: {RES}
origin: [{OX}, {OY}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
mode: trinary
""")
    print(f'  → {yml}')
    print()
    print('🔴 이걸로 끝이 아니다 — Nav2 에 필터를 붙여야 실제로 막힌다:')
    print('   ① costmap_filter_info_server + map_server(마스크) 노드 기동')
    print('   ② global/local costmap plugin 목록에 keepout_filter 추가')
    print('   전문 = docs/MASTER_PLAN.md §7 예약 55')
    return 0


if __name__ == '__main__':
    sys.exit(main())
