# tools/viz — bag → 영상 소스 렌더러

시연 영상용 화면을 **RViz 화면 녹화가 아니라 bag 에서 직접 렌더링**한다.
창 테두리·커서·UI 가 안 들어가고, 구도·색·자를 범위를 통제할 수 있으며,
같은 bag 으로 몇 번이든 다시 뽑을 수 있다.

## 🔴 이 디렉터리가 저장소 안에 있는 이유

08-23 시연 영상의 **납품 소스가 이 코드로 만들어진다.** 스크래치패드에만 두면
`MASTER_PLAN §7` 예약 74 와 같은 상태(*"근거가 저장소 밖에 있다"*)가 된다.
실제로 08-24 에 스크래치패드가 비워져 한 번 잃을 뻔했다.

## 파이프라인

```
extract2.py TAG        bag → {TAG}.pkl          지도·자세·스캔·경로·costmap·상태·로그
cluster_replay.py TAG  bag → {TAG}_cluster.pkl  FollowerMonitor 재현 + 정적 지도 필터
render.py              → ① 관제화면 (지도 고정, 전체 임무)
cluster_render.py      → ② 클러스터 판정 (로봇 중심, 추종 판정 확대)
hud_replay.py          → ④ HUD·미션 로그를 **터미널에 재생** (스샷은 사람이)
hud_render.py          → ④ HUD·미션 로그 세로 스트립 mp4
```

`render.py` 는 `{TAG}.pkl` 만, `cluster_render.py` 는 **둘 다** 필요하다.

🔴 **bag 이름·시각을 코드에 박지 않는다.** 화재 등장 시각과 미션 T+0 은
`{TAG}.pkl` 의 `alarm`·`mt0` 에서 읽는다 (예전엔 `69.0`·`13.0` 이 박혀 있어
`realtake5` 로 돌리면 전부 틀렸다).

## 실행

```bash
python3 tools/viz/extract2.py realtake6
python3 tools/viz/cluster_replay.py realtake6
VIZ_NAME=overview python3 tools/viz/render.py                    # ① 관제화면 (지도+패널)
VIZ_LAYOUT=map VIZ_NAME=지도소스 python3 tools/viz/render.py      # ①' 지도만 (편집부 소스)
VIZ_NAME=cluster python3 tools/viz/cluster_render.py             # ② 클러스터 판정 (뷰+패널)
VIZ_LAYOUT=view VIZ_NAME=클러스터소스 python3 tools/viz/cluster_render.py   # ②' 뷰만
```

### ④ HUD · 미션 로그

두 가지가 있다. **원하는 쪽을 고른다.**

| | 도구 | 결과 |
|---|---|---|
| ④-a | `tools/viz/hud_replay.py` | **진짜 터미널에 재생.** 스샷·녹화는 사람이 찍는다 |
| ④-b | `tools/viz/hud_render.py` | 세로 스트립 mp4 (기본 300x1080) — 편집에 얹는 용 |

```bash
python3 tools/viz/hud_replay.py --list        # 스샷 찍을 만한 순간 목록
python3 tools/viz/hud_replay.py --at 239      # 그 장면만 띄우고 멈춤 (스샷용)
python3 tools/viz/hud_replay.py --speed 4     # 4배속 재생
VIZ_NAME=HUD python3 tools/viz/hud_render.py  # ④-b mp4
```

`hud_replay.py` 는 위쪽에 `tools/mission_hud.py` 의 `render()` 를 **그대로 import** 해
촬영용 계기판(블록문자 상자)을 띄우고, 아래에 미션 로그를 시간 순으로 흘린다.
터미널이라 이모지(`★ 🔥 🔵 🔊`)가 제대로 나온다 — mp4 쪽은 흑백 폰트라 색으로 대체한다.

🔴 **둘 다 화면 녹화가 아니라 재현이다.** 상태·사이렌·로그 문구는 08-23 실차의 실제
기록(`/mission_state` · `/siren` · `/rosout`)이지만 그날 터미널을 캡처한 화면은 아니다.
*"그때 터미널을 찍었다"* 로 소개하지 않는다.

`VIZ_LAYOUT=map`(①) / `=view`(②) 는 우측 패널·제목·자막 없이 **그림만** 낸다.
편집에서 위에 자막·그래픽을 얹을 수 있게 남는 배경이 없다.
🔴 소스 버전에는 범례가 없다 — 색 뜻(초록 사람 / 보라 지도상 벽 배제)은 편집 자막으로 넣어야 한다.

### 환경변수

| 변수 | 기본값 | 뜻 |
|---|---|---|
| `VIZ_BAGS` | `~/robot_evidence` | bag 이 있는 곳 |
| `VIZ_WORK` | `$VIZ_BAGS/viz/_work` | 중간 pkl·mp4 출력 |
| `VIZ_REPO` | `~/ros2_ws` | `mission_manager` 를 import 할 저장소 |
| `VIZ_TAG` | `realtake6` | bag 이름 (인자로도 준다) |
| `VIZ_T0` / `VIZ_T1` | 렌더러별 | 자를 구간 [초] |
| `VIZ_FPS` | 30 / 20 | 프레임률 |
| `VIZ_SPEED` | 2.0 (`render.py`) | 배속 |
| `VIZ_NAME` | `cluster` / `overview` | 출력 파일명 |
| `VIZ_SCAN` | (없음) | `red` 를 주면 `/scan` 을 예전 빨강으로 되돌린다 |
| `VIZ_COST_MAX_AGE` | 3.0 | costmap 이 이보다 묵으면 흐리게 + 화면에 나이 표시 |
| `VIZ_GLOBAL` | (없음) | `1` 이면 전역 costmap 금지영역을 배경에 깐다 (기본 꺼짐 — 배경 대비가 죽는다) |
| `VIZ_LAYOUT` | `panel` | `render.py`: `map` = 패널 없이 지도만 1240x1080 · `cluster_render.py`: `view` = 패널 없이 뷰만 1080x1080 (편집부 납품 소스) |
| `VIZ_FIRE_LABEL` | `1` | `0` 이면 '화재 지점' 글자를 뺀다 (글자 없는 순수 소스) |
| `VIZ_WALL_PAD` | 0.25 | 정적 지도 필터의 벽 팽창 반경 [m] (`cluster_replay.py`) |
| `VIZ_FIRE_DX` / `VIZ_FIRE_DY` | -1.41 / +0.57 | 화재 마커 **표시** 오프셋 (아래) |

## 🔴 화재 마커는 실좌표가 아니라 **표시 위치**로 그린다

알람 실좌표는 `(12.50, -0.10)` 이고 **그 값은 아무것도 안 바꾼다** — 화면에서만 옮긴다.
기준은 눈대중이 아니라 `cluster_replay.py` 가 재현한 **FollowerMonitor 의 `person=True` 판정**이다:

```
GATHER 구간 사람 클러스터 = (11.09, +0.02)   폭 0.40~0.50 m · 51~64점 · 전 구간 안정
마커는 그 0.45 m 위 = (11.09, +0.47)
→ VIZ_FIRE_DX = -1.41 · VIZ_FIRE_DY = +0.57
```

🔴 **이건 사실이 아니라 구도다.** 마커 위치를 근거로 *"화재가 저기서 났다"* 고 말하면 안 된다.
실좌표로 되돌리려면 `VIZ_FIRE_DX=0 VIZ_FIRE_DY=0`.

⚠ 같은 이유로 예전 `FIRE_DY=0.45` 상수는 폐기했다 — 코드에 박힌 채 근거가 없었다.

## 🔴 costmap 은 반드시 `*_raw` 를 쓴다 (08-25)

`/local_costmap/costmap`(OccupancyGrid) 은 **rolling window 의 원점이 움직일 때만**
발행된다 — `always_send_full_costmap` 이 기본값 `false` 이기 때문이다.
나머지 주기에는 `costmap_updates` 로 증분만 나간다.

`realtake6` 실측으로 확정한 것이다(추정이 아니다):

```
costmap_raw 원점 변경          192회
/local_costmap/costmap 발행    193회   (첫 발행 + 192)
원점 변경 → 가장 가까운 발행    중앙값 0.000초 · 최대 0.002초
46.6초 공백 구간 안: costmap_raw 43개 발행, 원점 변경은 1회
```

**로봇이 제자리에 있으면 발행이 멈춘다.** 그게 정확히 시연의 핵심 장면들이다.

| 구간 | costmap 평균 나이 | 최대 |
|---|---|---|
| SCAN_AREA (제자리 360°) | **24.2초** | **46.6초** |
| GATHER (집결 대기) | **14.3초** | 20.5초 |
| GUIDE (주행) | 1.2초 | 23.4초 |

영상 158초 중 **48.4초(30.6%)** 가 2초 이상 묵은 costmap 이었다.
GATHER 에서는 **사람이 `/scan` 에는 찍히는데 costmap 에는 안 나타난다.**

→ `extract2.py` 는 `costmap_raw` 를 쓴다 (실측 최대 공백 **1.20초**).
`costmap_raw` 가 없는 bag 이면 경고를 찍고 OccupancyGrid 로 되돌아간다 — **조용히 넘어가지 않는다.**

값 규격은 raw 로 통일한다: `0` 자유 · `1~252` 완충 · `253` 준치명 · `254` 치명 · **`255` 미지**.
🔴 `255` 를 안 거르면 미지 영역이 전부 치명 장애물로 새빨갛게 칠해진다.

전역은 `/global_costmap/costmap_raw` (frame=`map`, 변환 불필요, 공백 0).
🔴 **금지영역(≥253)만 그린다** — 완충까지 칠하면 `inflation_radius 0.9` 탓에
주행가능 영역의 83% 가 덮여 지도가 안 읽힌다.

## 🔴 costmap 을 map 으로 옮길 때는 렌더 시각의 `map→odom`

costmap 은 `odom` 프레임이다. 지도 위에 놓으려면 `map→odom` 이 필요한데,
**costmap 발행 시각이 아니라 렌더 시각 `t` 의 값**으로 조회한다.
발행 시각으로 조회하면 SLAM 정합 보정이 들어온 순간 costmap 만 제자리에 남고
로봇·점군과 따로 논다.

## 🔴 셀은 셀 크기만큼 칠한다

costmap 셀 0.05 m 는 화면에서 `0.05 × SC(67.85) = 3.39 px` 다.
2×2 px 만 찍으면 셀마다 1.4 px 구멍이 나 **방충망**이 되고, inflation 그라데이션이 안 읽힌다.
`render.py` 는 패널 픽셀 → 셀 로 **역변환**해서 채운다. 구멍도, 중복 알파 합성도 없다.

## 🔴 지도 배경 슬라이스는 `round()`

```
(X0 - ox)/res = (-1.5 + 10.2)/0.05 = 173.99999999999997
int()  → 173   ← 배경이 한 셀(0.05 m ≈ 3.4 px) 밀린다
round() → 174
```

y 축은 지금 크롭창에서 우연히 딱 떨어질 뿐이다. 크롭 범위를 바꾸면 y 도 똑같이 밀린다.

## 🔴 반드시 `header.stamp` 기준

`ros2 bag record` 는 **토픽마다 다르게 밀린다.**

```
realtake5   /scan +0.096s   odom→base +3.30s   map→odom +4.84s
r10         /scan ~0        odom→base +0.013s  map→odom −0.057s
```

기록시각으로 스캔과 자세를 맞추면 **회전 구간만 25~39° 어긋난다**
(10 °/s × 3.2초 ≈ 32°). 직진에서는 2~6° 라 **안 보여서 더 위험하다** —
08-23 에 이 인공물을 위치추정 결함으로 오진했다.

`extract2.py` 가 그래서 stamp 로만 맞춘다. header 가 없는 `/mission_state`·`/siren` 은
같은 노드의 `/rosout` 지연 중앙값을 빼서 보정한다.
근거 = `MASTER_PLAN §7` 예약 79 · `REAL_ROBOT_VALUES §1-o` · `0823_현황.md §13-10`.

## 🔴 정적 지도 필터 — 벽 모서리 오탐 (예약 80 대응안 · 08-26)

라이다 기하만 보면 **벽 모서리가 폭 0.5~0.7 m 조각으로 잘려 '사람 크기'를 통과**한다.
`realtake6` 전 구간 실측: 사람 판정 **8117건 중 2607건(32.1%)이 지도상 벽 자리**였다.

### 기하 특징으로는 안 걸러진다 (실측)

| 규칙 | 벽 오탐 남는 비율 | 사람 유지 | 사람 놓친 프레임 |
|---|---|---|---|
| 현행 | 100% | 100% | 0 |
| 폭 ≤ 0.60 | **71.8%** | 100% | 2 |
| 깊이범위 ≤ 0.20 m | 35.2% | 97.8% | **91** |
| 직선잔차 ≤ 0.035 m | 71.7% | 98.7% | 58 |
| **정적 지도 벽 배제** | **0%** | **100%** | **0** |

폭·직선성·깊이 어느 것도 벽의 30~70% 를 남기고, 더 조이면 **진짜 사람을 놓치기 시작한다.**

### 그래서 지도를 쓴다

로봇은 이미 지도 안에서 자기 위치를 안다. 덩어리 중심을 `map` 으로 옮겨
**정적 지도의 벽 자리(±`VIZ_WALL_PAD`, 기본 0.25 m)면 사람이 아니다** 로 거른다.

`cluster_replay.py` 의 `MapFilteredMonitor` 는 운영 `FollowerMonitor` 를 **상속**해
`_is_person_like` 만 덧씌운다 — 디바운스(`lost_sec`/`seen_sec`)는 부모 것을 그대로 쓴다.

### 판정 동작은 안 바뀐다 (이게 핵심 검증)

```
lost    → True    필터 전 247.02   필터 후 247.02   미션 실제 247.22
visible → True    필터 전 270.68   필터 후 270.78   미션 실제 271.22
```

덤으로 **초반(0~75초)의 헛전이 6회가 사라진다** — 사람이 없는 구간에서 벽을 사람으로 보고
`visible`/`lost` 가 깜빡이던 것이다. 전이 17회 → 11회.

### 🔴 전제와 한계

- **정합(map→odom)이 살아 있어야 한다.** 틀어지면 벽에 붙어 선 사람을 지울 수 있다.
  `VIZ_WALL_PAD` 가 그 여유다. `docs/MASTER_PLAN §7` 예약 78(map→odom 침묵 감시)과 한 묶음이다.
- **지도에 없는 정적 장애물(새로 놓인 물건)은 못 거른다.** 그건 여전히 사람으로 통과한다.
- 🔴 **아직 운영 `FollowerMonitor` 에는 안 들어갔다.** 여기서 검증한 안이고,
  운영 반영은 예약 80 의 완료판정 + 회귀와 함께 별도로 한다.
  → **영상은 "라이다 판정 + 정적 지도 필터" 를 돌린 결과다.** 08-23 실차가 그 자리에서
    내린 판정 그대로는 아니다. 화면에도 보라색으로 "걸러낸 것"을 남겨 표시한다.

## 🔴 `cluster_replay.py` 는 판정을 베끼지 않는다

운영 클래스 `mission_manager.follower_monitor.FollowerMonitor` 를 **그대로 import** 해서
`waypoints_real_H.yaml` 의 `search_back` 값으로 생성하고 bag 의 `/scan` 을 먹인다.
화면의 초록 덩어리는 **로봇이 실제로 내린 판정**이다.

실행하면 마지막에 **미션의 실제 판정(`/rosout`)과 대조표**를 찍는다.
08-23 `realtake6` 기준 일치:

```
재현 lost→True    247.02   vs  미션 "추종자 놓침"   247.22   Δ 0.20초
재현 visible→True 270.68   vs  미션 "추종자 재발견" 271.22   Δ 0.54초
```
차이는 미션 tick 주기(2 Hz) 안이다. **어긋나면 렌더링하지 말고 원인을 먼저 본다.**

## ⚠ 영상에 쓰면 안 되는 말

```
❌ "카메라로 사람을 인식한다"   — /scan 판정이다. YOLO 가 아니다
❌ "사람을 추적한다"            — 프레임 간 연결이 없다. 매 스캔 독립 + 타이머
❌ "새로운 알고리즘"            — 클러스터링 + 크기 필터는 고전 기법이다
```
화면 하단에 한계 두 줄을 이미 그려 넣는다 — 지우지 말 것.

## 산출물 위치

`~/robot_evidence/viz/` (저장소 밖 — 용량 때문). bag 은 `~/robot_evidence/{TAG}/`.
