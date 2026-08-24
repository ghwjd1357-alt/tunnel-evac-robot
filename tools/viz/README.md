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
extract2.py TAG        bag → {TAG}.pkl          지도·자세·스캔·경로·costmap·상태
cluster_replay.py TAG  bag → {TAG}_cluster.pkl  FollowerMonitor 재현 + 미션 판정 대조
render.py              → 관제화면 (지도 고정, 전체 임무)
cluster_render.py      → 클러스터 판정 (로봇 중심, 추종 판정 확대)
```

`render.py` 는 `{TAG}.pkl` 만, `cluster_render.py` 는 **둘 다** 필요하다.

## 실행

```bash
python3 tools/viz/extract2.py realtake6
python3 tools/viz/cluster_replay.py realtake6
VIZ_NAME=overview python3 tools/viz/render.py
VIZ_NAME=cluster  python3 tools/viz/cluster_render.py
```

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
