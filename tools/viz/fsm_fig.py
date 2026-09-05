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


def label(dr, x, y, text, font, color, anchor='mm', bg=C_BG, pad=5):
    w = dr.textlength(text, font=font); h = font.size
    if anchor == 'mm':
        bx, by = x - w / 2, y - h / 2
    elif anchor == 'lm':
        bx, by = x, y - h / 2
    else:
        bx, by = x - w, y - h / 2
    if bg:
        dr.rectangle([bx - pad, by - pad * 0.4, bx + w + pad, by + h + pad * 0.6], fill=bg)
    dr.text((bx, by), text, font=font, fill=color)


def build(width, ratio):
    W = int(width); H = int(round(W / ratio))
    S = W / 2000.0                       # 기준 2000px 대비 배율
    im = Image.new('RGB', (W, H), C_BG)
    dr = ImageDraw.Draw(im)

    f_name = F(44 * S, 'Bold')
    f_ko   = F(25 * S)
    f_edge = F(23 * S)
    f_band = F(27 * S, 'Bold')
    f_note = F(22 * S)
    f_en   = F(34 * S, 'Bold')
    f_ek   = F(21 * S)

    M   = 46 * S
    GAP = 26 * S
    BW  = (W - 2 * M - 5 * GAP) / 6
    BH  = 158 * S
    R   = 15 * S
    LW  = max(int(3.4 * S), 2)
    LWD = max(int(2.4 * S), 1)

    def bx(i): return M + i * (BW + GAP)
    def cx(i): return bx(i) + BW / 2

    def draw_box(x, y, w, h, name, ko, fill, line, fn=None, fk=None):
        fn = fn or f_name; fk = fk or f_ko
        rounded(dr, [x, y, x + w, y + h], R, fill, line, LW)
        tw = dr.textlength(name, font=fn)
        dr.text((x + w / 2 - tw / 2, y + h * 0.20), name, font=fn, fill=C_TXT)
        kw = dr.textlength(ko, font=fk)
        dr.text((x + w / 2 - kw / 2, y + h * 0.58), ko, font=fk, fill=C_SUB)

    # ── 1행 : 정상 흐름 ──────────────────────────────────────────
    Y1 = 130 * S
    Y1B = Y1 + BH
    for i, (n, ko) in enumerate(MAIN):
        last = (i == len(MAIN) - 1)
        draw_box(bx(i), Y1, BW, BH, n, ko,
                 C_END_F if last else C_MAIN_F, C_END_L if last else C_MAIN_L)
        if i:
            arrow(dr, [(bx(i) - GAP + 2 * S, Y1 + BH / 2), (bx(i) - 5 * S, Y1 + BH / 2)],
                  C_EDGE, LW, 16 * S)
    for i, txt in enumerate(['화재 알람', '집결지 도착', '탐색 완료',
                             '집결 시간 경과', '탈출 지점 도착']):
        label(dr, (cx(i) + cx(i + 1)) / 2, Y1 - 26 * S, txt, f_edge, C_SUB)

    # ── 2행 : 유도 중 회복 루프 ──────────────────────────────────
    Y2 = 530 * S
    Y2B = Y2 + BH
    gi = 4                                   # GUIDE
    hold_x = bx(gi)                          # GUIDE 바로 아래
    sbw    = BW * 1.5
    sb_x   = bx(2)
    draw_box(hold_x, Y2, BW, BH, LOOP[0][0], LOOP[0][1], C_LOOP_F, C_LOOP_L)
    draw_box(sb_x, Y2, sbw, BH, LOOP[1][0], LOOP[1][1], C_LOOP_F, C_LOOP_L,
             fn=F(40 * S, 'Bold'))

    gc = cx(gi)
    # GUIDE ↓ HOLD (놓침)
    xd = gc + 58 * S
    arrow(dr, [(xd, Y1B + 3 * S), (xd, Y2 - 7 * S)], C_LOOP_L, LW, 16 * S)
    label(dr, xd + 14 * S, (Y1B + Y2) / 2, '3초 연속 미확인', f_edge, C_LOOP_L, 'lm')
    # HOLD ↑ GUIDE (재발견)
    xu = gc - 58 * S
    arrow(dr, [(xu, Y2 - 3 * S), (xu, Y1B + 7 * S)], C_END_L, LW, 16 * S)
    label(dr, xu - 14 * S, (Y1B + Y2) / 2, '재발견', f_edge, C_END_L, 'rm')
    # HOLD → SEARCH_BACK
    arrow(dr, [(hold_x - 4 * S, Y2 + BH / 2), (sb_x + sbw + 7 * S, Y2 + BH / 2)],
          C_LOOP_L, LW, 16 * S)
    label(dr, (hold_x + sb_x + sbw) / 2, Y2 + BH / 2 - 30 * S,
          '4.5초 재확인 실패', f_edge, C_LOOP_L)
    # SEARCH_BACK ↑→ GUIDE (재발견)
    lane = Y2 - 92 * S
    xr = gc - 132 * S
    arrow(dr, [(sb_x + sbw / 2, Y2 - 4 * S), (sb_x + sbw / 2, lane),
               (xr, lane), (xr, Y1B + 7 * S)], C_END_L, LW, 16 * S)
    label(dr, (sb_x + sbw / 2 + xr) / 2, lane, '재발견 → 유도 복귀', f_edge, C_END_L)

    # ── 3행 : 예외 위임 ──────────────────────────────────────────
    BY0 = 812 * S
    EY  = BY0 + 76 * S
    EBH = 148 * S
    BY1 = EY + EBH + 26 * S
    rounded(dr, [M - 18 * S, BY0, W - M + 18 * S, BY1], R, C_BAND, C_BANDL, LW)
    dr.text((M + 2 * S, BY0 + 20 * S),
            '예외 — 자동 진행을 멈추고 사람에게 넘긴다', font=f_band, fill=C_EXC_L)

    ebw = (W - 2 * M - 3 * (22 * S)) / 4
    exc_cx = []
    for i, (n, ko) in enumerate(EXC):
        x = M + i * (ebw + 22 * S)
        exc_cx.append(x + ebw / 2)
        rounded(dr, [x, EY, x + ebw, EY + EBH], R * 0.85, C_EXC_F, C_EXC_L, LW)
        tw = dr.textlength(n, font=f_en)
        dr.text((x + ebw / 2 - tw / 2, EY + 26 * S), n, font=f_en, fill=C_TXT)
        kw = dr.textlength(ko, font=f_ek)
        dr.text((x + ebw / 2 - kw / 2, EY + 82 * S), ko, font=f_ek, fill=C_SUB)

    # 점선 : 어디서 갈라지는가 (차선을 나눠 겹치지 않게)
    #  RESCUE·NO_VICTIM ← SCAN_AREA(2)  ·  BLOCKED ← 화재 알람 처리  ·  FAULT ← 주행 goal 전반
    routes = [(cx(2), exc_cx[0], 700 * S),          # SCAN_AREA → RESCUE
              (cx(2), exc_cx[1], 748 * S)]          # SCAN_AREA → NO_VICTIM
    for sx, dx_, ly in routes:
        arrow(dr, [(sx, Y1B + 3 * S), (sx, ly), (dx_, ly), (dx_, EY - 7 * S)],
              C_EXC_L, LWD, 14 * S, dashed=True)
    # BLOCKED ← 화재 알람 처리 (on_alarm · PATROL 중 수신)
    lb = 786 * S
    arrow(dr, [(cx(0), Y1B + 3 * S), (cx(0), lb), (exc_cx[3], lb),
               (exc_cx[3], EY - 7 * S)], C_EXC_L, LWD, 14 * S, dashed=True)
    label(dr, (cx(0) + exc_cx[3]) / 2, lb, '화재 알람 · 안전한 집결지를 못 만들면',
          f_ek, C_EXC_L, bg=C_BG)
    # FAULT 는 특정 상태에서만 오지 않는다 — 짧은 꼬리 + 말로 적는다
    fy = BY0 - 40 * S
    arrow(dr, [(exc_cx[2], fy), (exc_cx[2], EY - 7 * S)], C_EXC_L, LWD, 14 * S, dashed=True)
    label(dr, exc_cx[2], fy - 16 * S, '주행 goal 을 내는 어느 상태에서든',
          f_ek, C_EXC_L, bg=C_BG)
    label(dr, (cx(2) + exc_cx[0]) / 2, 700 * S - 20 * S, '사람 상태 판정', f_ek, C_EXC_L, bg=C_BG)

    # ── 주석 ────────────────────────────────────────────────────
    dr.text((M, H - 46 * S),
            '정상 흐름 8 + 예외 위임 4 = 총 12개 상태   ·   '
            '실차 2026-08-23 : 8종 상태를 9번 거치며 8회 전환, 306.5초 완주',
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
