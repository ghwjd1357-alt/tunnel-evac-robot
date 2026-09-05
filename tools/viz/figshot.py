# -*- coding: utf-8 -*-
"""figshot.py — 보고서 표에 넣을 **정지 그림**을 bag 에서 직접 렌더링한다.

`render.py` 는 영상(mp4)을 만든다. 이건 한 장짜리 인쇄용 그림이다.
  · 밝은 배경 (인쇄물은 흰 종이다 — 어두운 화면 캡처는 잉크만 먹고 안 읽힌다)
  · 표 칸 비율에 맞춘 임의 창 (`--fit` 또는 `--win`)
  · 축척 막대 · 범례 · 라벨을 끄고 켤 수 있다

데이터 출처는 `render.py` 와 **같은 `{TAG}.pkl`** 이다 (`extract2.py` 산출물).
지도·자세·경로는 08-23 실차 기록 그대로이고, 이 스크립트는 그리기만 한다.

사용
    python3 tools/viz/figshot.py map   -o out/26_slam_map.png
    python3 tools/viz/figshot.py plan  -o out/28_nav2_plan.png --at 135.5
    python3 tools/viz/figshot.py --help

🔴 화재 마커는 render.py 와 같은 **표시 오프셋**(VIZ_FIRE_DX/DY)을 쓴다.
   실좌표가 아니다 — 근거는 `tools/viz/README.md` 참조.
"""
import os, sys, math, bisect, pickle, argparse
import numpy as np
from PIL import Image, ImageDraw, ImageFont

BAGS = os.environ.get('VIZ_BAGS', os.path.expanduser('~/robot_evidence'))
WORK = os.environ.get('VIZ_WORK', os.path.join(BAGS, 'viz', '_work'))
TAG  = os.environ.get('VIZ_TAG', 'realtake6')

FT = '/usr/share/fonts/opentype/noto/NotoSansCJK-{}.ttc'
def F(sz, bold=False):
    return ImageFont.truetype(FT.format('Bold' if bold else 'Regular'), sz, index=1)

# ---- 색 (밝은 인쇄용) ----
C_PAPER = (255, 255, 255)
C_FREE  = (255, 255, 255)     # 주행 가능
C_OCC   = ( 33,  39,  49)     # 벽 · 장애물
C_UNK   = (233, 237, 242)     # 미탐색
C_TRAIL = ( 37, 110, 225)     # 실제 주행 궤적
C_PLAN  = (  0, 163, 108)     # Nav2 계획 경로
C_ROB   = (240, 150,   0)     # 로봇
C_GOAL  = (  0, 140,  95)     # 목적지
C_FIRE  = (222,  45,  38)     # 화재
C_TXT   = ( 28,  33,  41)
C_DIM   = (110, 120, 133)
C_RULE  = (196, 203, 212)

FIRE_DX = float(os.environ.get('VIZ_FIRE_DX', -1.41))
FIRE_DY = float(os.environ.get('VIZ_FIRE_DY',  0.57))


# ─────────────────────────────────────────────────────────── 데이터
def load():
    p = f'{WORK}/{TAG}.pkl'
    if not os.path.exists(p):
        sys.exit(f'없다: {p}\n  먼저 → python3 tools/viz/extract2.py {TAG}')
    return pickle.load(open(p, 'rb'))


def pick(seq, times, t):
    i = bisect.bisect_right(times, t) - 1
    return seq[i] if i >= 0 else None


def pose_at(D, t):
    pt = D['pose'][:, 0]
    i = min(max(bisect.bisect_right(list(pt), t) - 1, 0), len(pt) - 1)
    return D['pose'][i][1:]


# ─────────────────────────────────────────────────────────── 창 계산
def fit_window(x0, x1, y0, y1, ratio, pad):
    """내용 bbox 를 감싸면서 요구 비율(가로/세로)을 맞추는 창."""
    x0, x1, y0, y1 = x0 - pad, x1 + pad, y0 - pad, y1 + pad
    w, h = x1 - x0, y1 - y0
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    if w / h < ratio:                    # 가로가 모자라다 → 좌우를 넓힌다
        w = h * ratio
    else:                                # 세로가 모자라다 → 위아래를 넓힌다
        h = w / ratio
    return cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2


def map_bbox(D, kind='known'):
    mp = D['map']; g = mp['g']; res = mp['res']; ox = mp['ox']; oy = mp['oy']
    m = (g >= 0) if kind == 'known' else (g >= 50)
    ys, xs = np.nonzero(m)
    return (ox + xs.min() * res, ox + xs.max() * res,
            oy + ys.min() * res, oy + ys.max() * res)


def traj_bbox(D, t0, t1):
    p = D['pose']; s = p[(p[:, 0] >= t0) & (p[:, 0] <= t1)]
    return s[:, 1].min(), s[:, 1].max(), s[:, 2].min(), s[:, 2].max()


# ─────────────────────────────────────────────────────────── 그리기
class Canvas:
    def __init__(self, D, win, width, ratio):
        self.D = D
        self.X0, self.X1, self.Y0, self.Y1 = win
        self.W = width
        self.H = int(round(width / ratio))
        self.SC = self.W / (self.X1 - self.X0)
        self._background()

    def w2p(self, x, y):
        return ((x - self.X0) * self.SC, (self.Y1 - y) * self.SC)

    def _background(self):
        mp = self.D['map']; g = mp['g']; res = mp['res']; ox = mp['ox']; oy = mp['oy']
        ix0 = int(round((self.X0 - ox) / res)); ix1 = int(round((self.X1 - ox) / res))
        iy0 = int(round((self.Y0 - oy) / res)); iy1 = int(round((self.Y1 - oy) / res))
        gh, gw = g.shape
        sub = np.full((iy1 - iy0, ix1 - ix0), -1, np.int8)
        sy0, sy1 = max(iy0, 0), min(iy1, gh)
        sx0, sx1 = max(ix0, 0), min(ix1, gw)
        if sy1 > sy0 and sx1 > sx0:
            sub[sy0 - iy0:sy1 - iy0, sx0 - ix0:sx1 - ix0] = g[sy0:sy1, sx0:sx1]
        sub = sub[::-1]
        rgb = np.zeros((sub.shape[0], sub.shape[1], 3), np.uint8)
        rgb[:] = C_UNK
        rgb[sub == 0] = C_FREE
        rgb[sub >= 50] = C_OCC
        self.im = Image.fromarray(rgb).resize((self.W, self.H), Image.NEAREST)
        self.dr = ImageDraw.Draw(self.im)

    # ---- 요소 ----
    def trail(self, t0, t1, width=5):
        p = self.D['pose']; s = p[(p[:, 0] >= t0) & (p[:, 0] <= t1)]
        pts = [self.w2p(x, y) for _, x, y, _ in s[::2]]
        if len(pts) > 2:
            self.dr.line(pts, fill=C_TRAIL, width=width, joint='curve')

    def plan(self, t, width=9, max_age=6.0):
        plans = self.D['plan']; pl = [p[0] for p in plans]
        p = pick(plans, pl, t)
        if p is None or t - p[0] > max_age or len(p[1]) < 2:
            return None
        pts = [self.w2p(x, y) for x, y in p[1][::2]]
        self.dr.line(pts, fill=C_PLAN, width=width, joint='curve')
        return p[1]

    def goal(self, xy, label=None, font=None):
        gx, gy = self.w2p(*xy); R = 15
        self.dr.ellipse([gx - R, gy - R, gx + R, gy + R], fill=C_PAPER,
                        outline=C_GOAL, width=5)
        self.dr.ellipse([gx - 6, gy - 6, gx + 6, gy + 6], fill=C_GOAL)
        if label:
            self._chip(gx + 26, gy - 15, label, C_GOAL, font)

    def robot(self, x, y, a, scale=1.0):
        cx, cy = self.w2p(x, y)
        R = 0.30 * self.SC * scale
        self.dr.ellipse([cx - R, cy - R, cx + R, cy + R], fill=(255, 255, 255),
                        outline=C_ROB, width=max(3, int(R * 0.28)))
        L = 0.46 * self.SC * scale
        self.dr.polygon([(cx + L * math.cos(-a), cy + L * math.sin(-a)),
                         (cx + 0.42 * L * math.cos(-a + 2.5), cy + 0.42 * L * math.sin(-a + 2.5)),
                         (cx + 0.42 * L * math.cos(-a - 2.5), cy + 0.42 * L * math.sin(-a - 2.5))],
                        fill=C_ROB)

    def fire(self, label=None, font=None):
        al = self.D.get('alarm')
        if not al:
            return
        fx, fy = self.w2p(al[0][1] + FIRE_DX, al[0][2] + FIRE_DY)
        self.dr.ellipse([fx - 17, fy - 17, fx + 17, fy + 17], outline=C_FIRE, width=5)
        self.dr.ellipse([fx - 8, fy - 8, fx + 8, fy + 8], fill=C_FIRE)
        if label:
            self._chip(fx + 28, fy - 15, label, C_FIRE, font)

    def _chip(self, x, y, text, color, font):
        tw = self.dr.textlength(text, font=font)
        pad = 9
        self.dr.rounded_rectangle([x, y, x + tw + pad * 2, y + font.size + pad], 7,
                                  fill=C_PAPER, outline=color, width=3)
        self.dr.text((x + pad, y + pad // 2), text, font=font, fill=color)

    def scalebar(self, metres, font, margin=26):
        px = metres * self.SC
        x1 = self.W - margin; x0 = x1 - px
        y = self.H - margin
        self.dr.line([(x0, y), (x1, y)], fill=C_TXT, width=5)
        for xx in (x0, x1):
            self.dr.line([(xx, y - 10), (xx, y + 10)], fill=C_TXT, width=5)
        lab = f'{metres:g} m'
        tw = self.dr.textlength(lab, font=font)
        self.dr.text(((x0 + x1) / 2 - tw / 2, y - font.size - 16), lab, font=font, fill=C_TXT)

    def legend(self, items, font, margin=26):
        """items = [(색, '이름'), ...] 좌하단."""
        lh = font.size + 15
        h = lh * len(items) + 16
        w = 26 + max(self.dr.textlength(n, font=font) for _, n in items) + 44
        x0, y0 = margin, self.H - margin - h
        self.dr.rounded_rectangle([x0, y0, x0 + w, y0 + h], 9,
                                  fill=(255, 255, 255), outline=C_RULE, width=3)
        for i, (col, name) in enumerate(items):
            yy = y0 + 12 + i * lh + font.size / 2
            self.dr.line([(x0 + 16, yy), (x0 + 44, yy)], fill=col, width=7)
            self.dr.text((x0 + 54, yy - font.size / 2 - 2), name, font=font, fill=C_TXT)

    def frame(self):
        self.dr.rectangle([0, 0, self.W - 1, self.H - 1], outline=C_RULE, width=3)

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.im.save(path)
        print(f'  ✅ {path}  {self.W}x{self.H}  ({self.W/self.H:.2f}:1)  '
              f'창 x[{self.X0:.1f},{self.X1:.1f}] y[{self.Y0:.1f},{self.Y1:.1f}]')


# ─────────────────────────────────────────────────────────── 장면
def scene_map(D, a):
    """① SLAM 지도 작성·위치 추정 — 실제로 그린 지도 + 실차 주행 궤적."""
    MT0 = D['mt0']
    if a.win:
        win = a.win
    else:
        bx = map_bbox(D, 'known') if a.whole else traj_bbox(D, MT0, MT0 + 307)
        win = fit_window(*bx, a.ratio, a.pad)
    c = Canvas(D, win, a.width, a.ratio)
    if not a.no_trail:
        c.trail(MT0, MT0 + 307)
        x, y, ang = pose_at(D, MT0 + 306.49)
        c.robot(x, y, ang)
    if a.scalebar:
        c.scalebar(a.scalebar, F(int(a.width / 62)))
    if not a.no_legend and not a.no_trail:
        c.legend([(C_TRAIL, '실차 주행 궤적'), (C_ROB, '로봇')], F(int(a.width / 62)))
    c.frame()
    c.save(a.out)


def scene_plan(D, a):
    """② 자율주행 경로계획 — 그 시각의 Nav2 전역 경로 + 지나온 궤적."""
    MT0 = D['mt0']
    t = MT0 + a.at
    plans = D['plan']; pl = [p[0] for p in plans]
    p = pick(plans, pl, t)
    if p is None or len(p[1]) < 2:
        sys.exit(f'T+{a.at} 에 경로가 없다')
    arr = np.array(p[1])
    length = float(np.sum(np.hypot(np.diff(arr[:, 0]), np.diff(arr[:, 1]))))
    rx, ry, ra = pose_at(D, t)
    print(f'  T+{a.at:.1f}s · 경로 {len(p[1])}점 {length:.2f} m · '
          f'나이 {t-p[0]:.1f}s · 로봇({rx:.2f},{ry:.2f}) → 목적지({arr[-1][0]:.2f},{arr[-1][1]:.2f})')

    if a.win:
        win = a.win
    else:
        bx = (min(arr[:, 0].min(), rx), max(arr[:, 0].max(), rx),
              min(arr[:, 1].min(), ry), max(arr[:, 1].max(), ry))
        win = fit_window(*bx, a.ratio, a.pad)
    c = Canvas(D, win, a.width, a.ratio)
    fs = int(a.width / 62)
    if not a.no_trail:
        c.trail(MT0, t, width=4)
    c.plan(t)
    c.goal(tuple(arr[-1]), '목적지' if not a.no_labels else None, F(fs, True))
    c.fire('화재 지점' if not a.no_labels else None, F(fs, True))
    c.robot(rx, ry, ra)
    if a.scalebar:
        c.scalebar(a.scalebar, F(fs))
    if not a.no_legend:
        c.legend([(C_PLAN, 'Nav2 계획 경로'), (C_TRAIL, '지나온 궤적'), (C_ROB, '로봇')], F(fs))
    c.frame()
    c.save(a.out)


# ─────────────────────────────────────────────────────────── main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scene', choices=['map', 'plan'])
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--at', type=float, default=135.5, help='plan: 미션 T+ 초 (기본 135.5)')
    ap.add_argument('--ratio', type=float, default=1.70, help='가로/세로 (기본 1.70 = 보고서 칸)')
    ap.add_argument('--width', type=int, default=2000, help='가로 픽셀')
    ap.add_argument('--pad', type=float, default=0.8, help='내용 바깥 여백 [m]')
    ap.add_argument('--win', type=float, nargs=4, metavar=('X0', 'X1', 'Y0', 'Y1'))
    ap.add_argument('--whole', action='store_true', help='map: 궤적이 아니라 지도 전체를 담는다')
    ap.add_argument('--scalebar', type=float, default=2.0, help='축척 막대 [m] · 0 이면 끔')
    ap.add_argument('--no-trail', action='store_true')
    ap.add_argument('--no-legend', action='store_true')
    ap.add_argument('--no-labels', action='store_true')
    a = ap.parse_args()

    D = load()
    print(f'[{TAG}] {a.scene}')
    (scene_map if a.scene == 'map' else scene_plan)(D, a)


if __name__ == '__main__':
    main()
