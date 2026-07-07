#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_twin_world.py — 쌍굴(twin-bore) 터널 월드 생성기 (07-07)
============================================================
실행:  python3 tools/gen_twin_world.py
출력:  src/tunnel_sim/worlds/tunnel_twin.world  (덮어씀)

[왜 생성기인가]
  아치 단면 = 기울어진 박스 2단. 기울인 박스의 중심좌표·roll 각은
  삼각함수 계산이라 손으로 SDF 에 박으면 반드시 틀린다 →
  수식 한 곳(여기)에 두고 월드는 '산출물'로 취급. 치수 바꾸면 재실행.

[평면도 — 위에서 내려다본 모습]  (world 좌표, 단위 m)

   y=13 ┌──────────────────────────────────────┐  ← 2번 굴 북벽 (통짜 40m)
        │            2번 굴 (폭 6m)             │
   y=7  ├──────┐    ┌──────┐    ┌──────┐    ┌──┤  ← 2번 굴 남벽 (통로 3곳 뚫림)
        │ 암반  │통로│ 암반  │통로│ 암반  │통로│  │     통로 폭 2.5m, y=3~7 (길이 4m)
   y=3  ├──────┘    └──────┘    └──────┘    └──┤  ← 1번 굴 북벽 (통로 3곳 뚫림)
        │ 🤖         1번 굴 (폭 6m)      🛢 🔶  │
   y=-3 └──────────────────────────────────────┘  ← 1번 굴 남벽 (통짜 40m)
      x=-20      x=-10       x=0        x=+10   x=+20
       (통로 중심 x = -10, 0, +10 — 10m 간격 3개)

  로봇 스폰 = world(-17, 0) → map(0,0).  map좌표 = world좌표 + (17, 0).

[아치 단면 — 굴을 정면(도로 방향)에서 본 모습]

        ← 꼭대기 약 2.8m 개방 (위에서 로봇 보이게) →
      ＼ 65°기움                          65°기움 ／   2단 (slant 1.0m)
       ＼35°기움                        35°기움 ／     1단 (slant 1.2m)
        │수직 1.4m                    수직 1.4m│      0단 ← ★ 라이다 평면(0.65m)은
        └───────────  폭 6m  ─────────────────┘          여기 맞음 = SLAM 은 수직벽만 봄
  → 아치가 위로 좁아져도 라이다 높이에선 벽이 수직 → 지도 품질 영향 없음.

[SDF 요점 복습]
  - 벽 하나 = model(static) 하나, 그 안에 link 3개(수직+아치2단). static 이라 joint 불필요.
  - link 마다 collision+visual 세트 (visual 만 있으면 라이다 통과 — 단골 함정).
  - 기운 박스: roll(x축 회전) = -s·θ  (s=+1 이면 벽이 +y 쪽으로 눕는다).
    중심 = 아랫변 중점 + (slant/2)·(0, s·sinθ, cosθ).
"""

import math
import os

# ===================== 치수 (여기만 고치고 재실행) =====================
X_MIN, X_MAX = -20.0, 20.0        # 굴 길이 40m
HALF_W = 3.0                      # 굴 반폭 (폭 6m)
BORE1_CY = 0.0                    # 1번 굴 중심선 y
BORE2_CY = 10.0                   # 2번 굴 중심선 y (사이 암반 4m)
T = 0.2                           # 벽 두께
H0 = 1.4                          # 0단 수직벽 높이 (라이다 0.65m 넉넉히 커버)
S1, TH1 = 1.2, math.radians(35)   # 1단: slant 길이, 기움각
S2, TH2 = 1.0, math.radians(65)   # 2단: slant 길이, 기움각
PASSAGES = [-10.0, 0.0, 10.0]     # 피난연결통로 중심 x (10m 간격)
P_HALF = 1.25                     # 통로 반폭 (폭 2.5m — inflation 0.9 양쪽에도 중앙 자유폭 0.7m)
P_WALL_H = 1.4                    # 통로 측벽 높이 (아치 없음 — 실제 피난통로도 낮은 사각굴)

OUT = os.path.join(os.path.dirname(__file__), '..',
                   'src', 'tunnel_sim', 'worlds', 'tunnel_twin.world')


def box_link(name, pose, size, color='0.7 0.7 0.7 1'):
    """collision+visual 쌍이 항상 같이 있는 박스 link (라이다 통과 함정 방지)."""
    x, y, z, r, p, yw = pose
    sx, sy, sz = size
    return f"""      <link name="{name}">
        <pose>{x:.4f} {y:.4f} {z:.4f}  {r:.4f} {p:.4f} {yw:.4f}</pose>
        <collision name="collision">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material><ambient>{color}</ambient></material>
        </visual>
      </link>"""


def arch_wall(name, x0, x1, wall_y, s):
    """x0~x1 구간, y=wall_y 에 선 '아치 벽' 모델 (수직 0단 + 기운 1·2단).
    s=+1 → 아치가 +y 쪽(그쪽이 굴 안쪽)으로 좁아짐 / s=-1 → -y 쪽."""
    length = x1 - x0
    xc = (x0 + x1) / 2.0
    links = []
    # 0단 수직 (라이다가 보는 벽)
    links.append(box_link('base', (0, 0, H0 / 2, 0, 0, 0), (length, T, H0)))
    # 1단 (35° 안쪽으로)
    y1 = s * (S1 / 2) * math.sin(TH1)
    z1 = H0 + (S1 / 2) * math.cos(TH1)
    links.append(box_link('arch1', (0, y1, z1, -s * TH1, 0, 0),
                          (length, T, S1), color='0.55 0.55 0.58 1'))
    # 2단 (65° — 이 위는 개방 = 천장 슬롯)
    yb = s * S1 * math.sin(TH1)                 # 1단 윗변 y
    zb = H0 + S1 * math.cos(TH1)                # 1단 윗변 z
    y2 = yb + s * (S2 / 2) * math.sin(TH2)
    z2 = zb + (S2 / 2) * math.cos(TH2)
    links.append(box_link('arch2', (0, y2, z2, -s * TH2, 0, 0),
                          (length, T, S2), color='0.45 0.45 0.5 1'))
    body = '\n'.join(links)
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{xc:.4f} {wall_y:.4f} 0  0 0 0</pose>
{body}
    </model>"""


def flat_wall(name, x, y, yaw, length, height, color='0.6 0.6 0.6 1'):
    """평범한 수직 벽 하나짜리 모델 (끝벽·통로 측벽용)."""
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {height / 2:.4f}  0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{length:.4f} {T} {height:.4f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{length:.4f} {T} {height:.4f}</size></box></geometry>
          <material><ambient>{color}</ambient></material>
        </visual>
      </link>
    </model>"""


def gapped_segments():
    """통로 자리를 비운 벽 구간 [(x0,x1), ...] — 북벽(1번굴)·남벽(2번굴) 공용."""
    segs, cur = [], X_MIN
    for c in PASSAGES:
        segs.append((cur, c - P_HALF))
        cur = c + P_HALF
    segs.append((cur, X_MAX))
    return segs


def main():
    models = []

    # --- 통짜 바깥벽 2개 (아치: 안쪽 = 굴 중심 방향) ---
    models.append(arch_wall('bore1_south', X_MIN, X_MAX, BORE1_CY - HALF_W, s=+1))
    models.append(arch_wall('bore2_north', X_MIN, X_MAX, BORE2_CY + HALF_W, s=-1))

    # --- 통로가 뚫린 안쪽벽 2개 (구간별 아치 벽) ---
    for i, (x0, x1) in enumerate(gapped_segments()):
        models.append(arch_wall(f'bore1_north_{i}', x0, x1, BORE1_CY + HALF_W, s=-1))
        models.append(arch_wall(f'bore2_south_{i}', x0, x1, BORE2_CY - HALF_W, s=+1))

    # --- 끝벽 4개 (x=±20, 각 굴 폭 막음. 아치 높이까지 시각적으로 닫는 2.4m) ---
    for tag, cy in (('bore1', BORE1_CY), ('bore2', BORE2_CY)):
        models.append(flat_wall(f'{tag}_west_end', X_MIN, cy, 1.5708, 6.4, 2.4))
        models.append(flat_wall(f'{tag}_east_end', X_MAX, cy, 1.5708, 6.4, 2.4))

    # --- 피난연결통로 측벽 (통로마다 좌·우, y=3~7 구간을 잇는 복도벽) ---
    py = (BORE1_CY + HALF_W + BORE2_CY - HALF_W) / 2.0   # 통로 중앙 y=5
    plen = (BORE2_CY - HALF_W) - (BORE1_CY + HALF_W) + 2 * T  # 벽 모서리 겹침 여유
    for c in PASSAGES:
        cname = f'p{int(c):+d}'.replace('+', 'e').replace('-', 'w')  # pe0/pw10/pe10 식 이름
        models.append(flat_wall(f'passage_{cname}_west', c - P_HALF, py, 1.5708,
                                plen, P_WALL_H, color='0.65 0.6 0.5 1'))
        models.append(flat_wall(f'passage_{cname}_east', c + P_HALF, py, 1.5708,
                                plen, P_WALL_H, color='0.65 0.6 0.5 1'))

    walls = '\n\n'.join(models)

    world = f"""<?xml version="1.0" ?>
<!--
  tunnel_twin.world — 쌍굴 아치 터널 (07-07, tools/gen_twin_world.py 가 생성)
  ★ 이 파일을 손으로 고치지 말 것 — 치수 변경은 생성기에서 하고 재실행.

  구조: 40m 굴 2개(폭 6m, 중심선 y=0 / y=10) + 피난연결통로 3개(x=-10,0,10, 폭 2.5m)
  아치: 수직 1.4m + 35° 1.2m + 65° 1.0m, 꼭대기 약 2.8m 슬롯 개방 (위에서 관찰용)
  로봇 스폰 = world(-17,0) → map(0,0).  map = world + (17,0)
-->
<sdf version="1.6">
  <world name="tunnel_twin">

    <!-- 모델 위치를 ROS 서비스로 읽고/쓰기 (fake_follower 가 사용) -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>10.0</update_rate>
    </plugin>

    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

{walls}

    <!-- ============ 터널 안 물체 (가벼운 모델만 — GUI 함정 ②) ============ -->
    <!-- 공사 콘: 낮은 장애물(라이다 사각) 실증용 — 남벽 근처, 경로 밖 -->
    <include>
      <uri>model://construction_cone</uri>
      <name>cone_1</name>
      <pose>5 -2.2 0  0 0 0</pose>
    </include>
    <!-- 배럴: 1번 굴 동쪽 장애물 -->
    <include>
      <uri>model://construction_barrel</uri>
      <name>barrel_1</name>
      <pose>8 -1.5 0  0 0 0</pose>
    </include>
    <!-- 대피자: 2번 굴, 순찰 경로(중심선 y=10) 살짝 비켜 북벽 쪽 -->
    <include>
      <uri>model://person_standing</uri>
      <name>victim_1</name>
      <pose>3 11.8 0  0 0 -1.5708</pose>
    </include>

  </world>
</sdf>
"""
    out = os.path.normpath(OUT)
    with open(out, 'w') as f:
        f.write(world)
    print(f'생성 완료: {out}')
    print(f'  벽 모델 {len(models)}개 / 통로 {len(PASSAGES)}개 '
          f'(중심 x={PASSAGES}, 폭 {2 * P_HALF}m)')
    # 아치 개방폭 확인 출력 (위에서 로봇이 보이는 슬롯)
    top_in = HALF_W - (S1 * math.sin(TH1) + S2 * math.sin(TH2))
    print(f'  천장 슬롯 개방폭 ≈ {2 * top_in:.2f} m (높이 '
          f'{H0 + S1 * math.cos(TH1) + S2 * math.cos(TH2):.2f} m)')


if __name__ == '__main__':
    main()
