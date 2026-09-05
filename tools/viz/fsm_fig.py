# -*- coding: utf-8 -*-
"""fsm_fig.py — 대피 임무 상태 기계 다이어그램 (보고서 인쇄용).

`mission_manager/mission_node.py` 의 `State` 열거형 12개와 실제 전이 지점을
그대로 옮긴 그림이다. 화면 캡처가 아니라 그린 것이므로 인쇄 크기에 맞춰
다시 뽑을 수 있다.

    정상 흐름 8   PATROL · APPROACH · SCAN_AREA · GATHER · GUIDE ·
                 HOLD · SEARCH_BACK · ESCAPED
    예외 위임 4   RESCUE · NO_VICTIM · FAULT · BLOCKED

🔴 상태 이름·전이 조건을 손으로 고치지 않는다. 코드가 바뀌면 코드를 보고 고친다.
   근거 줄번호는 아래 STATES / EDGES 주석에 남겼다.

사용
    python3 tools/viz/fsm_fig.py -o out/29_fsm.png
    python3 tools/viz/fsm_fig.py -o out/29_fsm.png --ratio 1.70 --width 2400
"""
import os, math, argparse
from PIL import Image, ImageDraw, ImageFont

FT = '/usr/share/fonts/opentype/noto/NotoSansCJK-{}.ttc'
def F(sz, w='Regular'):
    return ImageFont.truetype(FT.format(w), int(sz), index=1)

# ---- 색 (밝은 인쇄용) ----
C_BG     = (255, 255, 255)
C_TXT    = ( 24,  29,  37)
C_SUB    = (108, 118, 132)
C_EDGE   = (120, 131, 146)
C_MAIN_F = (233, 241, 254)     # 정상 흐름 채움
C_MAIN_L = ( 37, 110, 225)     # 정상 흐름 테두리
C_LOOP_F = (255, 243, 224)     # 회복 루프 채움
C_LOOP_L = (223, 137,  15)     # 회복 루프 테두리
C_END_F  = (223, 245, 235)
C_END_L  = (  0, 150,  99)
C_EXC_F  = (253, 236, 236)     # 예외 위임
C_EXC_L  = (203,  62,  56)
C_BAND   = (247, 249, 251)
C_BANDL  = (223, 228, 235)

# ── 상태 (mission_node.py State 열거형 · 74~108줄) ────────────────────
#  (키, 영문, 국문, 종류)
MAIN = [
    ('PATROL',      '평시 순찰'),
    ('APPROACH',    '화재 감지 → 출동'),
    ('SCAN_AREA',   '집결지 360° 탐색'),
    ('GATHER',      '집결 대기'),
    ('GUIDE',       '선행 유도 · 후방 감시'),
    ('ESCAPED',     '탈출 완료'),
]
LOOP = [
    ('HOLD',        '제자리 재수집'),
    ('SEARCH_BACK', '마지막 목격 지점 역행'),
]
EXC = [
    ('RESCUE',      '쓰러진 사람 확정 → 신고 후 정지'),
    ('NO_VICTIM',   '집결지에 사람 없음 확정'),
    ('FAULT',       'Nav2 주행 실패 → 재시도 소진'),
    ('BLOCKED',     '안전한 집결지 계산 실패'),
]


def rounded(dr, box, r, fill, outline, width):
    dr.rounded_rectangle(box, r, fill=fill, outline=outline, width=width)


def arrow(dr, pts, color, width, head=17, dashed=False):
    if dashed:
        for i in range(len(pts) - 1):
            (x0, y0), (x1, y1) = pts[i], pts[i + 1]
            L = math.hypot(x1 - x0, y1 - y0)
            n = max(int(L / (width * 4.5)), 1)
            for k in range(n):
                if k % 2:
                    continue
                f0, f1 = k / n, min((k + 1) / n, 1.0)
                dr.line([(x0 + (x1 - x0) * f0, y0 + (y1 - y0) * f0),
                         (x0 + (x1 - x0) * f1, y0 + (y1 - y0) * f1)],
                        fill=color, width=width)
    else:
        dr.line(pts, fill=color, width=width, joint='curve')
    (px, py), (qx, qy) = pts[-2], pts[-1]
    a = math.atan2(qy - py, qx - px)
    dr.polygon([(qx, qy),
                (qx - head * math.cos(a - 0.45), qy - head * math.sin(a - 0.45)),
                (qx - head * math.cos(a + 0.45), qy - head * math.sin(a + 0.45))],
               fill=color)


def label(dr, x, y, text, font, color, anchor='mm', bg=C_BG, pad=6):
    """여러 줄(\n) 지원. 흰 칩을 깔아 선 위에 놓아도 읽힌다."""
    lines = text.split('\n')
    lh = font.size * 1.28
    wmax = max(dr.textlength(t, font=font) for t in lines)
    hh = lh * len(lines)
    if anchor == 'lm':
        bx = x
    elif anchor == 'rm':
        bx = x - wmax
    else:
        bx = x - wmax / 2
    by = y - hh / 2
    if bg:
        dr.rectangle([bx - pad, by - pad * 0.5, bx + wmax + pad, by + hh + pad * 0.5], fill=bg)
    for i, t in enumerate(lines):
        tw = dr.textlength(t, font=font)
        if anchor == 'lm':
            tx = bx
        elif anchor == 'rm':
            tx = bx + wmax - tw
        else:
            tx = bx + (wmax - tw) / 2
        dr.text((tx, by + i * lh), t, font=font, fill=color)


def build(width, ratio):
    """🔴 배치 규칙 — 선이 서로 겹치지 않는다.

    예외·회복 상태를 아래 칸에 **자기 출처 바로 밑**으로 놓아, 모든 전이가
    짧은 세로선·대각선으로 끝나게 했다. 아래로 내려갔다 오는 것은
    HOLD→SEARCH_BACK 하나뿐이고 그 아래에는 아무것도 없다.

        윗줄  PATROL  APPROACH  SCAN_AREA  GATHER     GUIDE   ESCAPED
        아랫줄 BLOCKED  FAULT    RESCUE   NO_VICTIM   HOLD   SEARCH_BACK
    """
    W = int(width); H = int(round(W / ratio))
    S = W / 2000.0
    im = Image.new('RGB', (W, H), C_BG)
    dr = ImageDraw.Draw(im)

    f_ko   = F(24 * S)
    f_edge = F(23 * S)
    f_note = F(22 * S)
    f_leg  = F(23 * S)

    M   = 44 * S
    GAP = 24 * S
    BW  = (W - 2 * M - 5 * GAP) / 6
    BH  = 190 * S
    R   = 15 * S
    LW  = max(int(3.4 * S), 2)

    def bx(i): return M + i * (BW + GAP)
    def cx(i): return bx(i) + BW / 2

    def fit(text, base, w):
        """칸 안에 들어갈 때까지 글자를 줄인다."""
        for sz in range(int(base), int(base * 0.6), -1):
            f = F(sz, 'Bold')
            if dr.textlength(text, font=f) <= w:
                return f
        return F(int(base * 0.6), 'Bold')

    def draw_box(i, y, name, ko, fill, line, w=None):
        x = bx(i); w = w or BW
        rounded(dr, [x, y, x + w, y + BH], R, fill, line, LW)
        fn = fit(name, 40 * S, w - 22 * S)
        dr.text((x + w / 2 - dr.textlength(name, font=fn) / 2, y + BH * 0.22),
                name, font=fn, fill=C_TXT)
        fk = f_ko
        while dr.textlength(ko, font=fk) > w - 16 * S and fk.size > 15 * S:
            fk = F(fk.size - 1)
        dr.text((x + w / 2 - dr.textlength(ko, font=fk) / 2, y + BH * 0.60),
                ko, font=fk, fill=C_SUB)

    Y1 = 150 * S; Y1B = Y1 + BH
    Y2 = 540 * S; Y2B = Y2 + BH

    # ── 윗줄 : 정상 진행 ────────────────────────────────────────
    for i, (n, ko) in enumerate(MAIN):
        last = (i == 5)
        draw_box(i, Y1, n, ko, C_END_F if last else C_MAIN_F,
                 C_END_L if last else C_MAIN_L)
        if i:
            arrow(dr, [(bx(i) - GAP + 1 * S, Y1 + BH / 2), (bx(i) - 4 * S, Y1 + BH / 2)],
                  C_EDGE, LW, 15 * S)
    for i, txt in enumerate(['화재 알람', '집결지 도착', '탐색 완료',
                             '집결 시간 경과', '탈출 지점 도착']):
        label(dr, (cx(i) + cx(i + 1)) / 2, Y1 - 26 * S, txt, f_edge, C_SUB)

    # ── 아랫줄 : 예외 4 + 회복 2 (각자 출처 바로 밑) ──────────────
    draw_box(0, Y2, *EXC[3], C_EXC_F, C_EXC_L)      # BLOCKED  ← PATROL
    draw_box(1, Y2, *EXC[2], C_EXC_F, C_EXC_L)      # FAULT    (출처 여럿)
    draw_box(2, Y2, *EXC[0], C_EXC_F, C_EXC_L)      # RESCUE   ← SCAN_AREA
    draw_box(3, Y2, *EXC[1], C_EXC_F, C_EXC_L)      # NO_VICTIM← SCAN_AREA
    draw_box(4, Y2, *LOOP[0], C_LOOP_F, C_LOOP_L)   # HOLD     ← GUIDE
    draw_box(5, Y2, *LOOP[1], C_LOOP_F, C_LOOP_L)   # SEARCH_BACK

    def vline(x, color, lab=None, side='r', font=None):
        arrow(dr, [(x, Y1B + 3 * S), (x, Y2 - 7 * S)], color, LW, 15 * S)
        if lab:
            label(dr, x + (14 * S if side == 'r' else -14 * S), (Y1B + Y2) / 2,
                  lab, font or f_edge, color, 'lm' if side == 'r' else 'rm')

    # PATROL → BLOCKED
    vline(cx(0), C_EXC_L, '안전한 집결지를\n못 만들면')
    # SCAN_AREA → RESCUE (세로) · → NO_VICTIM (대각)
    vline(bx(2) + BW * 0.34, C_EXC_L, '쓰러짐', 'l')
    arrow(dr, [(bx(2) + BW * 0.80, Y1B + 3 * S), (bx(3) + BW * 0.55, Y2 - 7 * S)],
          C_EXC_L, LW, 15 * S)
    label(dr, (bx(2) + BW * 0.80 + bx(3) + BW * 0.55) / 2 + 30 * S,
          (Y1B + Y2) / 2, '아무도 없음', f_edge, C_EXC_L, 'lm')
    # GUIDE ↓ HOLD · HOLD ↑ GUIDE
    xd = bx(4) + BW * 0.28
    arrow(dr, [(xd, Y1B + 3 * S), (xd, Y2 - 7 * S)], C_LOOP_L, LW, 15 * S)
    label(dr, xd - 12 * S, (Y1B + Y2) / 2, '3초 연속\n미확인', f_edge, C_LOOP_L, 'rm')
    xu = bx(4) + BW * 0.58
    arrow(dr, [(xu, Y2 - 3 * S), (xu, Y1B + 7 * S)], C_END_L, LW, 15 * S)
    label(dr, xu + 12 * S, (Y1B + Y2) / 2, '재발견', f_edge, C_END_L, 'lm')
    # SEARCH_BACK ↗ GUIDE (대각 · 위 두 세로선의 오른쪽만 지난다)
    arrow(dr, [(bx(5) + BW * 0.14, Y2 - 3 * S), (bx(4) + BW * 0.90, Y1B + 7 * S)],
          C_END_L, LW, 15 * S)
    label(dr, bx(5) + BW * 0.22, (Y1B + Y2) / 2 - 34 * S,
          '재발견', f_edge, C_END_L, 'lm')
    # HOLD → SEARCH_BACK (아래로 돌아간다 — 그 밑에는 아무것도 없다)
    lane = Y2B + 62 * S
    arrow(dr, [(cx(4), Y2B + 3 * S), (cx(4), lane), (cx(5), lane), (cx(5), Y2B + 7 * S)],
          C_LOOP_L, LW, 15 * S)
    label(dr, (cx(4) + cx(5)) / 2, lane, '4.5초 재확인 실패', f_edge, C_LOOP_L)
    # FAULT 는 특정 상태에서만 오지 않는다
    label(dr, cx(1), (Y1B + Y2) / 2, '주행 goal 을 내는\n어느 상태에서든', f_edge, C_EXC_L)

    # ── 범례 ────────────────────────────────────────────────────
    LY0 = 880 * S; LY1 = LY0 + 104 * S
    rounded(dr, [M - 14 * S, LY0, W - M + 14 * S, LY1], R, C_BAND, C_BANDL, LW)
    items = [(C_MAIN_F, C_MAIN_L, '정상 진행 6'),
             (C_LOOP_F, C_LOOP_L, '유도 중 회복 2'),
             (C_EXC_F,  C_EXC_L,  '예외 — 멈추고 사람에게 넘긴다 4')]
    x = M + 18 * S
    for fill, line, name in items:
        cw = 46 * S
        rounded(dr, [x, LY0 + 34 * S, x + cw, LY0 + 70 * S], 7 * S, fill, line, LW)
        dr.text((x + cw + 14 * S, LY0 + 38 * S), name, font=f_leg, fill=C_TXT)
        x += cw + 26 * S + dr.textlength(name, font=f_leg) + 58 * S

    dr.text((M, H - 52 * S),
            '총 12개 상태  ·  실차 2026-08-23 : 8종 상태를 9번 거치며 8회 전환, '
            '306.5초에 전 구간 완주',
            font=f_note, fill=C_SUB)
    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--width', type=int, default=2000)
    ap.add_argument('--ratio', type=float, default=1.70)
    a = ap.parse_args()
    im = build(a.width, a.ratio)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    im.save(a.out)
    print(f'  ✅ {a.out}  {im.width}x{im.height}  ({im.width/im.height:.2f}:1)')


if __name__ == '__main__':
    main()
