#!/usr/bin/env python3
"""Perception Adapter — `/detections`(역할 B) → `/alarm`(역할 A 미션).

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
이 노드가 왜 있는가 (초보자용 설명)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
역할 B의 YOLO 는 "카메라에서 몇 미터 앞에 불이 있다" 를 알려준다.
그런데 미션 상태머신은 "지도(map) 위 어느 좌표에 불이 있다" 를 받아야 한다.
카메라는 로봇에 붙어 있어서 로봇이 움직이면 같이 움직이므로, 두 좌표계는 다르다.

    카메라 좌표 (내 앞 2m)  ──TF 변환──▶  지도 좌표 (map 에서 x=12.5, y=-0.1)

그 변환과, 변환하기 전의 검증을 여기서 한다. 합의사항 §2 가 역할 A 책임으로
못박은 네 가지가 정확히 이것이다:

    · schema · frame · stamp · 유한값 검증
    · TF 로 map 좌표 변환
    · 반복 관측 · 오탐 억제
    · 이벤트 발행

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🔴 이 노드는 기존 코드를 한 줄도 건드리지 않는다
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
`mission_node` 는 이 노드의 존재를 모른다. 그냥 `/alarm` 을 구독할 뿐이고,
그 토픽은 관제 웹도 발행한다. 즉 **어댑터가 죽어도 관제 수동 클릭이 즉시
대체한다.** 실패 시 후퇴 비용이 0 이도록 이렇게 설계했다.

그리고 우리가 이상한 좌표를 보내도 `mission_node` 가 **이미 거부한다**:
유한값 검사 · `frame_id == 'map'` 검사 · 복도 중심선에서 5m 투영 검사
(`mission_node.on_alarm`). 즉 방어가 두 겹이다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🔴 미검증 신고 — 이 노드가 실패한다면 1순위 용의자는 우리 코드가 아니다
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
`Detection3D.position`(역할 B의 depth 결합 결과)이 **한 번도 검증된 적이 없다.**
합의사항 §9 3단계(rosbag 으로 depth 유효율·map 좌표 측정)를 건너뛰었기 때문이다.
그래서 `use_fixed_range` 스위치를 넣었다 — 방위각만 쓰고 거리는 고정값으로 둔다.
⚠ 그걸 쓰면 정본과 영상 서술에 "거리는 고정값을 썼다" 를 반드시 남긴다.

정본 = `docs/MASTER_PLAN.md §7` 예약 60 · 역할 A 9차 회신 §3.

⚠ **알려진 로그 잡음** — Ctrl+C 종료 시 `tf2_ros` 리스너 스레드가
`ExternalShutdownException` 역추적을 stderr 에 한 번 찍는다(§84.1 로 `spin_thread=True`
를 켠 대가다). 기능 영향은 없고 종료 경로에서만 난다 — **촬영 중 이 문구를 고장으로
읽지 않는다.**
"""

import math

import rclpy
import rclpy.duration
import rclpy.node
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
import tf2_ros

from tunnel_interfaces.msg import Detection3DArray


# ═══════════════════════════════════════════════════════════════════════
# 순수 함수부 — ROS 없이 단위테스트가 가능한 자리
#   프로젝트 관례다(`mission_node` 도 판정 수식을 순수 함수로 뽑아 둔다).
#   여기 있는 것은 노드를 띄우지 않고 pytest 로 전수 검사한다.
# ═══════════════════════════════════════════════════════════════════════

# 🔴 계약이 정한 닫힌 열거 5종 (합의사항 §15-b). 이 밖의 값은 계약 위반이다.
VALID_CLASSES = ('person_fallen', 'person_ok', 'person_unknown', 'fire', 'smoke')


def is_finite_point(x, y, z):
    """NaN·Inf 를 걸러낸다.

    ⚠ 왜 이게 따로 있는가 — `mission_node` 가 07-19 에 실제로 당한 구멍이다.
      NaN 좌표가 예외도 없이 '멀쩡한 집결지'로 둔갑했다(재현: NaN → (4,0)).
      숫자가 아닌 것은 계산 앞에서 걸러야지, 계산 뒤에는 안 걸러진다.
    """
    return math.isfinite(x) and math.isfinite(y) and math.isfinite(z)


def clamp_range(x, y, z, max_range):
    """카메라 원점에서 `max_range` 보다 먼 점을 그 거리로 당겨온다.

    왜 필요한가: depth 가 틀리면 거리가 크게 나온다. 그대로 map 으로 옮기면
    복도 밖 좌표가 되고, `mission_node` 가 '지도 밖 오클릭'으로 거부한다.
    거부 자체는 안전하지만 **시연이 그 자리에서 멈춘다.**
    방위(어느 쪽인가)는 depth 와 무관하게 대체로 맞으므로, 방위를 살리고
    거리만 자른다.

    반환: (x, y, z, clamped?) — 잘랐는지 여부를 같이 돌려준다(로그·정본용).
    ⚠ 거리 0 은 자를 수 없다(방향이 없다). 그때는 그대로 돌려준다.
    """
    d = math.sqrt(x * x + y * y + z * z)
    if d <= max_range or d == 0.0:
        return x, y, z, False
    k = max_range / d
    return x * k, y * k, z * k, True


def fix_range(x, y, z, fixed_range):
    """방위만 쓰고 거리를 `fixed_range` 로 강제한다 (`use_fixed_range` 전용).

    🔴 이건 depth 를 못 믿을 때의 **격하 모드**다. 쓰면 반드시 기록에 남긴다.
    ⚠ 거리 0 이면 방향이 없어서 강제할 수 없다 → None 을 돌려 거부시킨다.
    """
    d = math.sqrt(x * x + y * y + z * z)
    if d == 0.0:
        return None
    k = fixed_range / d
    return x * k, y * k, z * k


def stamp_age_sec(now_sec, stamp_sec):
    """관측 시각이 얼마나 오래됐는지. 음수(미래)도 그대로 돌려준다.

    ⚠ 미래 stamp 를 0 으로 뭉개지 않는 이유 — 시계가 어긋난 상태를 '신선함'으로
      읽으면 그게 곧 조용한 통과다. 호출자가 보고 판단하게 둔다.
    """
    return now_sec - stamp_sec


class ConfirmTracker:
    """반복 관측으로 오탐을 억제한다 — 합의사항 §2 가 역할 A 에 맡긴 책임.

    한 프레임만 보고 출동하면 YOLO 의 한순간 오탐이 그대로 임무가 된다.
    역할 B 5차 §4-b 실측에 따르면 오탐은 '켜자마자 1분 안에, conf 0.48,
    32 px' 로 뜬다 — **한 프레임짜리**다. 그래서 창(window) 안에서 N 번
    보여야 확정한다.

    ⚠ '연속 N 프레임'이 아니라 '창 안에서 N 번'인 이유:
      10Hz 스트림에서 한 프레임 깜빡이는 건 정상이다. 연속을 요구하면
      진짜 화재가 깜빡임 때문에 영원히 확정 안 되는 쪽이 더 위험하다.
      (`mission_node` 가 추종 판정에서 같은 이유로 lost/visible 타이머를
       비대칭으로 둔 것과 같은 논리다.)

    🔴 08-21 Codex §82.4 재현 반영 — 수신 벽시각만 세면 안 된다
    ------------------------------------------------------------------
    구판은 `add(수신시각)` 이었다. 재현: **같은 프레임 한 장**을 0.0~0.4 초에
    다섯 번 넣으면 `[F,F,F,F,True]` 로 확정됐다. 서로 다른 자리의 한-프레임
    오탐 다섯 개도 똑같이 확정됐다. 둘 다 "반복 관측" 이 아니다.

    그래서 근거를 두 개 요구한다:
      ① **촬영시각(stamp)이 새로워야 한다** — 같거나 과거면 같은 프레임이거나
         재전송이다. 새 증거가 아니므로 세지 않는다.
      ② **직전 관측과 공간적으로 가까워야 한다** — 멀면 다른 대상이다.
         그때는 누적을 버리고 처음부터 센다(합치지 않는다).
    """

    def __init__(self, need, window_sec, assoc_radius=1.0):
        self.need = int(need)
        self.window_sec = float(window_sec)
        self.assoc_radius = float(assoc_radius)
        self._hits = []          # [(수신시각, stamp, (x,y,z) 또는 None)]
        self._last_stamp = None

    @staticmethod
    def _far(a, b, r):
        if a is None or b is None:
            return False
        return math.dist(a, b) > r

    def add(self, t, stamp, pos=None):
        """관측 1건 기록 후, 확정됐으면 True.

        t     = 수신 벽시각 [s] (창 계산용)
        stamp = 촬영시각 [s] (동일·역순 거부용). 유한값이 아니면 버린다.
        pos   = 관측 좌표 (x,y,z). 좌표계는 호출부가 일관되게만 주면 된다.
        """
        t = float(t)
        s = float(stamp)
        if not math.isfinite(s):
            return False
        if self._last_stamp is not None and s <= self._last_stamp:
            return False                 # 같은 프레임/재전송 — 새 근거가 아니다
        # 🔴 08-21 §83.2 — 구판은 **직전 점과만** 비교하는 single-link 였다.
        #   재현: 0.0→0.9→1.8→2.7→3.6 m 로 걸어가면 매 걸음이 1.0 m 안이라
        #   첫 점과 3.6 m 떨어졌는데도 확정됐다. 이어 붙기만 하면 복도 끝까지
        #   같은 화재가 된다. → **seed(첫 hit) 기준 반경**으로 잠근다.
        #
        # 🔴 08-21 §84.5 — 그런데 **prune 보다 먼저 비교**하고 있었다. 그래서
        #   창 밖으로 만료된 seed 가 창 안의 정상 누적을 통째로 지웠다.
        #   재현: need=5 window=3.0 radius=1.0 에
        #     (t,x) = (0,0.0) (2.8,0.9) (2.9,0.9) (3.0,0.9) (3.1,1.8) (3.2,1.8)
        #   → 창 안 2.8~3.2 의 5건은 서로 최대 0.9 m 라 계약상 한 seed 반경인데,
        #     이미 만료된 x=0 seed 와 비교해 전부 reset → count 2 · 확정 0.
        #   walking-chain 은 계속 막고 **창 안의 정상 5건은 살려야** 한다.
        #   → **창을 먼저 정리하고, 그 결과의 첫 점을 seed 로 쓴다.**
        self._prune(t)
        # 🔴 08-21 §85.6 — prune 으로 **seed 가 바뀌면** 생존 hit 들도 새 seed
        #   기준으로 다시 걸러야 한다. §84.5 는 incoming 한 점만 새 seed 와
        #   비교했다. 재현: (0,0)(0.2,-1)(2.9,+1)(3.10,-1)(3.11,-1)(3.12,-1) 에서
        #   t=3.10 에 seed 가 0 → -1 로 바뀌는데 +1 이 생존해, 반경 1 m 클러스터에
        #   **2 m 떨어진 점**이 증거로 남고 마지막 입력에 확정됐다.
        #   어느 한 위치도 5회를 못 채웠는데 두 표적의 관측이 합쳐진 것이다.
        #   → 만료된 anchor 를 붙잡지 않으면서(그건 §84.5 의 false negative)
        #     **현재 창의 seed 로 생존분을 재필터**한다.
        if self._hits:
            seed = self._hits[0][2]
            self._hits = [h for h in self._hits
                          if not self._far(h[2], seed, self.assoc_radius)]
        if self._hits and self._far(pos, self._hits[0][2], self.assoc_radius):
            self._hits = []              # 다른 대상 — 누적을 합치지 않는다
        self._last_stamp = s
        self._hits.append((t, s, pos))
        return len(self._hits) >= self.need

    def _prune(self, t):
        lo = t - self.window_sec
        self._hits = [h for h in self._hits if h[0] >= lo]

    def count(self, t=None):
        if t is not None:
            self._prune(t)
        return len(self._hits)

    def reset(self):
        """재무장 — 누적과 stamp 단조 기준을 함께 지운다 (82.5 다음 테이크)."""
        self._hits = []
        self._last_stamp = None


def validate_params(vals):
    """수치 파라미터 전량 검사 — 불량 사유 목록을 돌려준다 (빈 목록 = 통과).

    🔴 08-21 Codex §82.4: 선언 시 아무 검사가 없어 NaN/Inf/음수/0 이 그대로
    돌았다. 재현 = `max_range=-1` → `clamp_range(2,0,0,-1)` 이 `(-1,0,0)`,
    즉 **로봇 뒤쪽** 좌표를 만들었다. 순수 함수로 뽑아 pytest 로 전수 검사한다.
    """
    out = []

    def num(k, lo=None, hi=None, positive=False, integer=False):
        v = vals.get(k)
        try:
            f = float(v)
        except (TypeError, ValueError):
            out.append(f'{k}: 숫자가 아니다 ({v!r})')
            return
        if not math.isfinite(f):
            out.append(f'{k}: 유한값이 아니다 ({v!r})')
            return
        if integer and float(v) != int(f):
            out.append(f'{k}: 정수여야 한다 ({v!r})')
            return
        if positive and f <= 0:
            out.append(f'{k}: 0 이하 ({v!r})')
            return
        if lo is not None and f < lo:
            out.append(f'{k}: {lo} 미만 ({v!r})')
        if hi is not None and f > hi:
            out.append(f'{k}: {hi} 초과 ({v!r})')

    num('min_confidence', lo=0.0, hi=1.0)
    # 🔴 §84.4 — 하한이 1 이면 "반복 관측" 이 한 장이라는 뜻이라 억제가 **없다.**
    #   ConfirmTracker 가 존재하는 이유 자체가 한 프레임 오탐이므로 2 부터 받는다.
    num('confirm_frames', lo=2, integer=True)
    num('confirm_window_sec', positive=True)
    num('max_stamp_age_sec', positive=True)
    num('max_range', positive=True)
    num('fixed_range', positive=True)
    # 🔴 §83.2 — 양수만 보면 `confirm_assoc_radius_m=1000.0` 도 통과한다(재현).
    #   그러면 공간 결합이 사실상 사라져 복도 어디의 오탐이든 한 화재가 된다.
    #   상한 근거 = 연결통로 반폭 0.825 m · 아래복도 반폭 1.18 m. 화재는 정지
    #   물체이므로 map 좌표에서 그보다 크게 튀면 같은 대상이 아니다.
    num('confirm_assoc_radius_m', positive=True, hi=2.0)
    num('tf_wait_sec', lo=0.0, hi=1.0)
    # 🔴 §85.5 — 이 수치가 검증 밖이었다(구 이름 `mission_state_max_age_sec`).
    #   `inf` 를 넣으면 비교가 전부 거짓이 되어 관문이 무기한 열렸다.
    #   상한 10.0 = 사람이 명령을 보내고 기다릴 수 있는 현실적 최대치.
    num('rearm_ack_timeout_sec', positive=True, hi=10.0)
    # §86.4 — 0 은 '중복 병합 안 함' 이라는 뜻이라 허용한다.
    num('rearm_dedup_sec', lo=0.0, hi=30.0)
    # refire_cooldown_sec 은 0 이하가 "평생 1회" 라는 뜻이라 부호를 안 막는다.
    # 다만 NaN/Inf 는 비교가 전부 False 가 되어 억제가 조용히 사라진다.
    num('refire_cooldown_sec')

    # ── 🆕 08-22 사람 경로 (`PROJECT_CONTEXT §4.1-b`) ──────────────────────
    # 🔴 상한을 다 건다. 이 값들이 `inf` 면 확정이 **영원히 안 서고**, 0 이면
    #   한 프레임 오탐이 곧바로 신고가 된다. 둘 다 촬영을 망치는 방향이다.
    # 상한 근거: 확정을 10초 넘게 기다리면 그 사이 사람이 이미 이동한다.
    num('person_confirm_sec_fallen', positive=True, hi=10.0)
    num('person_confirm_sec_leave', positive=True, hi=10.0)
    # 🔴 하한 2 — 화재의 `confirm_frames` 와 같은 논리다. 1 이면 "반복 관측"이
    #   한 장이라는 뜻이라 디바운스가 없는 것과 같다.
    num('person_min_frames', lo=2, integer=True)
    num('person_min_confidence', lo=0.0, hi=1.0)
    # 🔴 상한 5.0 — stale 을 길게 잡으면 센서가 죽은 뒤에도 마지막 판정이
    #   살아 있어, 미션이 사람이 계속 있는 줄 안다. 계약값은 0.5 다.
    num('person_stale_sec', positive=True, hi=5.0)
    return out


def pick_best(detections, want, min_conf):
    """대상 클래스 중 confidence 가 가장 높은 하나를 고른다.

    반환: (best_or_None, violations) — violations 는 계약 열거 밖 class_name 목록.
    ⚠ 계약 위반을 조용히 버리지 않고 같이 돌려주는 이유 — 계약이 깨진 것을
      알아야 하는 쪽은 사람이다(합의사항 §15-b 닫힌 열거).
    """
    best = None
    violations = []
    for d in detections:
        name = getattr(d, 'class_name', '')
        if name not in VALID_CLASSES:
            violations.append(name)
            continue
        if name != want:
            continue
        # 🔴 §82.4 — 계약은 confidence 를 0~1 로 고정한다. NaN 은 어떤 비교에도
        #   False 라 `< min_conf` 를 그냥 통과했다(재현: NaN 이 best 로 채택됨).
        conf = getattr(d, 'confidence', None)
        try:
            conf = float(conf)
        except (TypeError, ValueError):
            violations.append(f'{name}: confidence 숫자 아님({conf!r})')
            continue
        if not math.isfinite(conf) or not (0.0 <= conf <= 1.0):
            violations.append(f'{name}: confidence 범위 밖({conf!r}) — 계약은 0~1')
            continue
        if conf < min_conf:
            continue
        p = d.position
        if not is_finite_point(p.x, p.y, p.z):
            continue
        if best is None or d.confidence > best.confidence:
            best = d
    return best, violations


# ═══════════════════════════════════════════════════════════════════════
# 노드
# ═══════════════════════════════════════════════════════════════════════

class PerceptionAdapter(Node):

    def __init__(self, **kwargs):
        # ⚠ **kwargs 를 통과시키는 이유 — 검증 하네스가 한 프로세스 안에서
        #   `parameter_overrides=` 로 노드를 여러 번 만든다. 그게 없으면
        #   시나리오마다 프로세스를 따로 띄워야 하고, 그 순간 신호·버퍼링
        #   문제가 다시 생긴다(실제로 세 번 깨졌다 — tools/e2e_adapter.py 머리말).
        super().__init__('perception_adapter', **kwargs)

        # ── 파라미터 ────────────────────────────────────────────────────
        # 🔵 전부 런타임 파라미터다. 촬영장에서 노드를 다시 안 띄우고
        #    `ros2 param set` 으로 바꿀 수 있다 — 현장에서 코드를 고치지 않는다.
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('alarm_topic', '/alarm')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('trigger_class', 'fire')
        # 🔴 08-22 역할 B §9 — **불이 없는데 fire 가 0.45~0.58 로 뜬다**(미해결).
        #   0.40 이면 그 오탐이 그대로 통과하고, 3초 창에 11.4 프레임이 들어오므로
        #   `confirm_frames 5` 도 쉽게 채운다 → **본편 원테이크 중 거짓 알람.**
        #   관측된 오탐 구간(0.45~0.58) 위로 올린다.
        #   ⚠ 이것은 근거 있는 문턱이 아니라 **응급 조치**다 — 진짜 화재의 confidence
        #     분포를 아직 아무도 안 쟀다(역할 B §8-b 가 "오탐 포함이라 신뢰 말라" 고 했다).
        #   🔵 그래도 올리는 쪽이 맞다: 거짓 알람은 테이크를 버리고, 놓친 자동 검출은
        #     오퍼레이터가 수동 `/alarm` 으로 즉시 메운다. 되돌릴 수 있는 쪽을 고른다.
        #   ⏸ 되돌리는 조건 = 진짜 화재 confidence 실측이 오면 그 값으로 다시 정한다.
        self.declare_parameter('min_confidence', 0.60)
        self.declare_parameter('confirm_frames', 5)
        self.declare_parameter('confirm_window_sec', 3.0)
        self.declare_parameter('max_stamp_age_sec', 1.0)
        self.declare_parameter('max_range', 5.0)
        # 🔴 격하 모드 — depth 를 못 믿을 때만. 쓰면 기록에 남긴다.
        self.declare_parameter('use_fixed_range', False)
        self.declare_parameter('fixed_range', 2.0)
        # 한 번 쏘면 이 시간 동안 다시 안 쏜다. 0 이하 = 평생 1회.
        self.declare_parameter('refire_cooldown_sec', -1.0)
        # 카메라 optical frame 이 비어 오면 이걸로 대체한다. 빈 문자열이면 거부.
        self.declare_parameter('fallback_source_frame', '')
        # 🔴 §82.4 — 직전 관측과 이만큼 떨어지면 다른 대상으로 본다 [m]
        self.declare_parameter('confirm_assoc_radius_m', 1.0)
        # ── 🆕 08-22 사람 경로 (`PROJECT_CONTEXT §4.1-b`) ───────────────────
        # 🔴 값은 전부 **보수적 기본값**이다. 역할 B 의 Jetson 실측(발행률·confidence
        #   분포)이 오면 **코드가 아니라 이 값만** 바꾼다. 답을 기다리느라 구현이
        #   멈추면 주말이 날아가므로 08-22 새벽에 이렇게 설계했다.
        # 🔴 비대칭 방향이 직관과 반대다 — 쓰러졌는데 `ok` 로 읽으면 로봇이 유도를
        #   시작하고 **떠난다**(= 쓰러진 사람을 버린다). 서 있는데 `fallen` 이면
        #   관제가 확인하고 끝이다. 그러니 **떠나도 된다는 판정이 비싸다.**
        self.declare_parameter('person_confirm_sec_fallen', 1.5)
        self.declare_parameter('person_confirm_sec_leave', 4.0)   # ok · none 공용
        # 🔴 08-22 역할 B 실측 반영 — `/detections` 는 계약 10 Hz 가 아니라 **3.8 Hz**다
        #   (정지 상태 · 회전 중 미측정). 그러면 `fallen 1.5s` 창에 **5.7 프레임**만
        #   들어온다. 구값 6 은 **확정을 영원히 막았다** — 쓰러진 사람을 보고도 신고가
        #   안 나간다. 4 로 내려 1.4 프레임 여유를 둔다.
        #   ⏸ 회전 중 발행률이 오면 다시 계산한다(더 낮으면 3 까지 내려야 할 수 있다).
        self.declare_parameter('person_min_frames', 4)
        self.declare_parameter('person_min_confidence', 0.50)
        # 🔴 08-22 — 계약값 0.5 를 쓰면 **정상 동작 중에 stale 이 뜬다.** 역할 B 실측
        #   프레임 간격이 min 0.000 ~ **max 0.729 s** 라 0.5 를 자주 넘는다. 그러면
        #   판정이 `stale` 로 계속 튕겨 아무것도 확정되지 않는다.
        #   실측 최대 간격의 약 2배로 잡는다. ⚠ 계약(§4.1)은 여전히 0.5 이고,
        #   **구현이 계약을 못 지키고 있는 것**이다 — 역할 B 에게 알린다.
        self.declare_parameter('person_stale_sec', 1.5)
        self.declare_parameter('person_status_topic', '/person_status')
        self.declare_parameter('victim_topic', '/victim')
        # 🔴 §82.3 — 기대 source frame. 빈 문자열이면 검사 안 함(비권장).
        #    이게 없으면 TF 트리에 있는 아무 frame(base_link 등)도 조용히 통과한다.
        self.declare_parameter('expected_source_frame', 'camera_color_optical_frame')
        # 🔴 §83.3 — 촬영시각 TF 가 없을 때 최신 TF 로 후퇴할지.
        #   **기본 False.** 후퇴가 기본이면 촬영시각 계약이 fail-open 이 된다.
        #   켜는 것은 사람이 명시적으로 고르는 도전 모드다.
        self.declare_parameter('allow_latest_tf_fallback', False)
        # 🔴 §83.3 — 촬영시각 TF 를 이만큼 기다린다. detection 이 동적 TF 보다
        #   조금 먼저 오는 **정상 비동기 순서**를 흡수하는 값이지, 없는 TF 를
        #   기다리는 값이 아니다. 10Hz 스트림에서 0.10s 면 한 프레임 안이다.
        self.declare_parameter('tf_wait_sec', 0.10)
        # §82.5 — 테이크 사이 재무장 명령 토픽 (std_msgs/String "rearm")
        self.declare_parameter('cmd_topic', '/adapter_cmd')
        # 🔴 §83.4 — 재무장을 아무 때나 받으면 **지속 오탐이 새 테이크를 자동
        #   시작**시킨다. PATROL 이 아닌 상태(=이미 출동 중)에서의 재무장은
        #   운영자의 실수일 가능성이 높으므로 거부한다. 상태를 한 번도 못 받았으면
        #   **거부**한다 — fail-closed. 미션 없이 어댑터만 시험할 때만 끈다.
        self.declare_parameter('mission_state_topic', '/mission_state')
        self.declare_parameter('rearm_requires_patrol', True)
        # 🔴 §85.4 — **캐시 나이가 아니라 handshake 다.**
        #   §84.3 은 마지막 상태에 수신시각을 붙였는데, 그건 "그 뒤로 바뀌지
        #   않았다" 를 증명하지 못한다. 재현: 미션이 PATROL 을 발행한 0.10초 뒤
        #   실제로 APPROACH 가 됐고 다음 2 Hz tick 이 아직 안 온 창에서, 캐시는
        #   exact PATROL 이고 age 도 1.5초 이하라 그대로 REARMED 가 났다.
        #   → 재무장은 **요청 이후에 새로 관측한** PATROL 에만 성립한다.
        #     이 값은 그 새 관측을 기다리는 상한이다(2 Hz 면 여섯 주기).
        self.declare_parameter('rearm_ack_timeout_sec', 3.0)
        # 🔴 §86.4 — 런북은 전달 유실을 막으려 `-w 1 --times 3` 으로 **같은 의도를
        #   세 번** 보낸다(§83.4). 구판은 그 셋을 **서로 다른 요청**으로 봤다.
        #   재현: rearm→PATROL 쌍 3회에 `REARMED` 가 **3번**, tracker reset 도 3회.
        #   더 나쁜 것은 늦게 도착한 3발째다 — 장면이 시작돼 hits 가 2 였는데
        #   **0 으로 지워졌다.** 즉 런북이 시키는 그대로 하면 매 테이크에 난다.
        #   → 성공 직후 이 시간 안의 재요청은 **같은 의도**로 보고 무시한다.
        self.declare_parameter('rearm_dedup_sec', 5.0)

        p = self.get_parameter
        self.detections_topic = p('detections_topic').value
        self.alarm_topic = p('alarm_topic').value
        self.target_frame = p('target_frame').value

        # ── TF ─────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        # 🔴 08-21 §86.3 — **검증이 보조 노드·스레드보다 먼저**여야 한다.
        #   구판은 `_tf_node` 와 non-daemon 리스너 스레드를 만든 **뒤에** 검증했다.
        #   그래서 `max_range=-1` 같은 불량 override 로 기동하면 `ValueError` 는
        #   나오는데 **프로세스가 자발 종료하지 않는다**(실측: timeout 이 SIGTERM
        #   으로 죽였다). 생성자가 반환을 못 하니 `main()` 의 `finally` 에도 못 닿고,
        #   `/adapter_status` 없는 **반쪽 노드**만 남아 node-list 준비 판정을 오도한다.
        #   08-20 에 세 번 당한 무증상 실패와 같은 형태다.
        #   → 자원을 하나도 만들기 전에 거른다.
        # 🔴 §82.4 — 수치 파라미터를 기동 시 한 곳에서 검증하고, 실패하면
        #    **크게 죽는다.** 재현: max_range=-1 이면 clamp_range 가 (2,0,0) 을
        #    (-1,0,0) 으로 뒤집어 로봇 **뒤쪽** 좌표를 만들었다. 조용히 도는 것보다
        #    안 뜨는 편이 낫다 — 안 뜨면 관제 수동 클릭이라는 후퇴로가 살아 있다.
        # 🔴 08-22 — 구판은 이 dict 를 **손으로 세고 있었다.** 그래서 사람 경로
        #   파라미터 5개를 더하자 `RUNTIME_NUMERIC` 에는 들어갔는데 여기는 빠져
        #   전부 `None` 이 됐고, "숫자가 아니다 (None)" 라는 **원인을 못 읽는
        #   메시지**로 기동이 거부됐다. 이 프로젝트가 반복해서 밟는 그 함정이다 —
        #   같이 고쳐야 하는 자리가 둘인데 하나만 고친다.
        #   → 목록 하나에서 만든다. `unvalidated_numeric_params()` 가 "선언했는데
        #     목록에 없는 것"을 잡고, 이 줄이 "목록에 있는데 안 넘긴 것"을 없앤다.
        # 🔴 §85.5 **패턴 수정** — 목록을 손으로 관리하면 다음 파라미터에서 또 샌다.
        #   실제로 §84.4("모든 수치를 검증한다")를 넣은 **바로 그 커밋**에서
        #   `mission_state_max_age_sec` 를 목록에 안 넣었다.
        #   → 선언된 수치 파라미터와 검증 목록의 **차집합을 기계가 0으로 만든다.**
        #     새 수치를 선언하고 목록에 안 넣으면 **기동이 안 된다.**
        missed = self.unvalidated_numeric_params()
        if missed:
            raise ValueError(
                f'🔴 수치 파라미터 {sorted(missed)} 가 검증 목록 밖이다. '
                f'RUNTIME_NUMERIC 과 validate_params 에 함께 넣을 것 (§85.5)')

        # 🔴 08-22 — **순서가 중요하다.** 목록 검사가 먼저다.
        #   아래 dict 를 `RUNTIME_NUMERIC` 에서 만들기 때문에, 선언만 하고 목록에
        #   안 넣은 파라미터는 여기 안 실려 `None` 이 되고 값 검사가 먼저 터진다.
        #   그러면 "숫자가 아니다 (None)" 라는, **고칠 자리를 안 알려주는 메시지**가
        #   나간다. 목록 검사를 앞에 두면 "RUNTIME_NUMERIC 에 넣어라" 가 뜬다.
        bad = validate_params({k: p(k).value for k in self.RUNTIME_NUMERIC})
        if bad:
            for m in bad:
                self.get_logger().error(f'🔴 파라미터 거부 — {m}')
            raise ValueError(f'perception_adapter 파라미터 {len(bad)}건 불량: {bad}')


        # 🔴 08-21 §85.2 — `spin_thread=True` **만으로는 안 된다.**
        #   §84.1 에서 리스너에 전용 executor 를 줬는데, 그 뒤 `main()` 의
        #   `rclpy.spin(node)` 가 **같은 노드를 도로 가져간다.**
        #   실측: `before_main_spin: nodes=1` → `during_main_spin: nodes=0`.
        #   즉 실제 구독 콜백에서는 detection 과 `/tf` 가 다시 한 executor 에
        #   놓이고, detection 이 `lookup_transform` 을 잡은 동안 늦은 TF 콜백은
        #   돌지 못한다. 🔴 **§84.1 의 "41 ms 성공" 검산은 `main()` 을 안 돌린
        #   구성의 값이었다** — 존재하지 않는 형상을 시험한 것이다.
        #
        #   → 리스너를 **어댑터가 아닌 전용 노드**에 붙인다. 그 노드는 절대
        #     `rclpy.spin` 에 넘기지 않으므로 누가 executor 를 빼앗을 수 없다.
        #   ⚠ 같은 노드를 multi-thread executor 로 바꾸는 길도 있으나, 그러면
        #     parameter·detection 상태에 동시성이 열려 callback group 과 tracker
        #     잠금까지 새 계약이 된다 — 촬영 전날에 열 표면이 아니다.
        self._tf_node = rclpy.node.Node(
            'perception_adapter_tf',
            namespace=self.get_namespace(),
            start_parameter_services=False,
            enable_rosout=False)
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self._tf_node, spin_thread=True)

        # ── 입출력 ──────────────────────────────────────────────────────
        # 🔴 QoS 는 계약이 고정했다 (Detection3DArray.msg 머리말):
        #    RELIABLE / VOLATILE / KEEP_LAST 5.
        #    ⚠ 안 맞으면 **연결 자체가 안 된다.** 그리고 ROS2 는 그걸 에러로
        #      안 알려준다 — 토픽은 보이는데 콜백이 안 온다. 무증상 실패다.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(
            Detection3DArray, self.detections_topic, self.on_detections, qos)
        self.alarm_pub = self.create_publisher(PoseStamped, self.alarm_topic, 10)
        # 관제·기록용 상태 문자열. 왜 안 쐈는지가 여기 남는다.
        self.status_pub = self.create_publisher(String, '/adapter_status', 10)
        # 🆕 08-22 사람 경로. QoS 는 `/alarm` 과 같은 기본값으로 맞춘다 —
        #   이 프로젝트는 RELIABLE/BEST_EFFORT 불일치로 이미 두 번 당했다.
        self.person_status_pub = self.create_publisher(
            String, p('person_status_topic').value, 10)
        self.victim_pub = self.create_publisher(
            PoseStamped, p('victim_topic').value, 10)
        # 사람 경로 내부 상태
        self._p_last_det_t = None     # 마지막 /detections 수신 시각
        self._p_streak_class = None   # 현재 연속의 프레임 판정
        self._p_streak_since = None
        self._p_streak_frames = 0
        self._p_status = 'stale'      # 🔴 기동 직후는 '사람 없음'이 아니라 '못 봤다'
        self._p_victim_pos = None
        self._p_victim_sent = False

        # ── 상태 ────────────────────────────────────────────────────────
        self.tracker = ConfirmTracker(p('confirm_frames').value,
                                      p('confirm_window_sec').value,
                                      p('confirm_assoc_radius_m').value)
        self.fired_at = None
        self._frames = 0
        self._last_reason = '아직 프레임 없음'
        self._tf_degraded = False

        # 🔴 08-21 §84.4 — **런타임 변경도 같은 검증을 통과해야 한다.**
        #   구판은 `__init__` 에서 한 번만 검증했고 parameter callback 이 없었다.
        #   실측: 노드가 뜬 뒤 `ros2 param set` 으로 `max_range=-1` ·
        #   `confirm_frames=1` · `confirm_assoc_radius_m=1000` · `tf_wait_sec=5` 를
        #   넣자 **네 건 다 successful=True** 로 저장됐다. 그런데 적용은 갈렸다 —
        #   `max_range`·`tf_wait_sec` 는 다음 콜백이 **바로 쓰고**, tracker 는 초기
        #   객체라 `confirm_frames`·radius 변경이 **전혀 안 먹었다.**
        #   즉 화면은 "적용됨" 인데 일부는 위험하게 적용되고 일부는 무시됐다.
        #   → 제안값을 현재 집합에 합쳐 `validate_params` 로 보고, 통과할 때만 받는다.
        #     그리고 tracker 파생 상태를 **같은 자리에서** 다시 만든다.
        self.add_on_set_parameters_callback(self._on_set_params)

        # §82.5 재무장 — 촬영은 3~5 테이크다. 구판은 refire_cooldown_sec=-1 이라
        #   **프로세스 평생 1회**였고, 미션 reset 을 보지 않아 두 번째 테이크부터
        #   어댑터가 조용히 안 쐈다. 단순 쿨다운은 이전 테이크의 지속 오탐이 새
        #   테이크를 자동 시작시켜서 더 나쁘다 — 사람이 명시적으로 재무장한다.
        self.create_subscription(
            String, p('cmd_topic').value, self.on_cmd, 10)
        self._mission_state = None
        self._mission_state_t = None
        self._rearm_pending_since = None       # §85.4 handshake 진행 중 표식
        self._last_rearm_t = None              # §86.4 중복 병합 기준 시각
        self.create_subscription(
            String, p('mission_state_topic').value, self.on_mission_state, 10)

        self.create_timer(2.0, self.tick)
        # 🆕 10 Hz — `/person_status` 는 **상시 신호**다. `/detections` 콜백에만
        #   매달면 발행이 끊긴 순간 아무 말도 못 하게 되는데, 그 침묵이야말로
        #   미션이 알아야 할 정보다(`stale`).
        self.create_timer(0.1, self._person_tick)

        self.get_logger().info(
            f'어댑터 기동 — {self.detections_topic} → {self.alarm_topic} '
            f'(target_frame={self.target_frame}, '
            f'trigger={p("trigger_class").value}, '
            f'confirm={p("confirm_frames").value}/{p("confirm_window_sec").value}s)')
        if p('use_fixed_range').value:
            self.get_logger().warn(
                f'🔴 격하 모드 — use_fixed_range=True. depth 거리를 무시하고 '
                f'{p("fixed_range").value} m 로 강제한다. 기록에 남길 것.')

    # ------------------------------------------------------------------
    def destroy_node(self):
        """§85.2 — 전용 TF 노드를 함께 내린다. 안 그러면 리스너 스레드가 남는다."""
        try:
            self._tf_node.destroy_node()
        except Exception:
            pass
        return super().destroy_node()

    #: §84.4 — 런타임 변경을 지원하는 수치 파라미터. 여기 없는 수치는 재기동 전용.
    RUNTIME_NUMERIC = (
        'min_confidence', 'confirm_frames', 'confirm_window_sec',
        'max_stamp_age_sec', 'max_range', 'fixed_range',
        'refire_cooldown_sec', 'confirm_assoc_radius_m', 'tf_wait_sec',
        'rearm_ack_timeout_sec', 'rearm_dedup_sec',
        # 🆕 08-22 사람 경로
        'person_confirm_sec_fallen', 'person_confirm_sec_leave',
        'person_min_frames', 'person_min_confidence', 'person_stale_sec',
    )
    #: tracker 를 다시 만들어야 하는 것들 (파생 상태)
    TRACKER_KEYS = ('confirm_frames', 'confirm_window_sec', 'confirm_assoc_radius_m')

    def unvalidated_numeric_params(self):
        """§85.5 — 선언은 됐는데 검증 목록에 없는 수치 파라미터 집합.

        빈 집합이어야 한다. 이 함수가 계약의 **기계 대조**이고, 기동이 그것을
        강제한다. `bool` 은 `int` 의 하위 타입이라 명시적으로 제외한다."""
        declared = set()
        for name, pv in self.get_parameters_by_prefix('').items():
            v = pv.value
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                continue
            declared.add(name)
        return declared - set(self.RUNTIME_NUMERIC)

    def _current_numeric(self):
        return {k: self.get_parameter(k).value for k in self.RUNTIME_NUMERIC}

    def _on_set_params(self, params):
        """§84.4 — 런타임 set 을 원자적으로 검증하고, 통과분만 파생 상태에 반영한다.

        ⚠ 이 콜백은 값이 **저장되기 전에** 불린다. 그래서 제안값을 현재 집합에
          합쳐서 본다 — 한 건만 보면 상호관계(예: guide<normal)를 못 본다.
        🔴 tracker 를 다시 만들면 **누적 관측이 사라진다.** 그것이 옳다 —
          확정 정책이 바뀌었는데 옛 정책으로 모은 근거를 이어 쓰면 안 된다.
        """
        from rcl_interfaces.msg import SetParametersResult
        proposed = {}
        for pr in params:
            if pr.name in self.RUNTIME_NUMERIC:
                proposed[pr.name] = pr.value
        if not proposed:
            return SetParametersResult(successful=True)

        cand = self._current_numeric()
        cand.update(proposed)
        bad = validate_params(cand)
        if bad:
            for m in bad:
                self.get_logger().error(f'🔴 런타임 파라미터 거부 — {m}')
            return SetParametersResult(
                successful=False,
                reason=f'perception_adapter 파라미터 불량: {bad}')

        if any(k in proposed for k in self.TRACKER_KEYS):
            self.tracker = ConfirmTracker(cand['confirm_frames'],
                                          cand['confirm_window_sec'],
                                          cand['confirm_assoc_radius_m'])
            self._last_reason = '확정 정책 변경 — 누적 관측 초기화'
            self.get_logger().warn(
                f'🔵 tracker 재구성 need={cand["confirm_frames"]} '
                f'window={cand["confirm_window_sec"]}s '
                f'radius={cand["confirm_assoc_radius_m"]}m — 누적 관측을 지웠다')
            self.say('TRACKER_RECONFIGURED')
        self.get_logger().warn(f'🔵 런타임 파라미터 적용: {sorted(proposed)}')
        return SetParametersResult(successful=True)

    def _finish_or_reject_rearm(self, st):
        """§85.4 — 요청 **이후에 새로 온** 관측으로 handshake 를 끝낸다."""
        if self._rearm_pending_since is None:
            return
        self._rearm_pending_since = None
        if st == 'PATROL':
            self._do_rearm()
        else:
            self.get_logger().error(
                f'🔴 재무장 거부 — 요청 뒤 관측한 상태가 "{st}" 다. '
                f'요청 시점의 캐시는 PATROL 이었지만 지금은 아니다')
            self.say(f'REARM_REJECTED (요청 뒤 {st})')

    def on_mission_state(self, msg: String):
        """§83.4 — 재무장 관문에 쓸 미션 상태. 판정은 on_cmd 가 한다.

        🔴 §84.3 — 값과 함께 **관측 시각**을 남긴다. 마지막 문자열만 들고 있으면
        "알람 직전 PATROL 을 받았고 그 뒤 실제로는 APPROACH 로 바뀌었지만 다음
        tick 이 아직 안 온" 창에서 **과거 상태를 현재로 승인**한다.
        """
        self._mission_state = (msg.data or '').strip().upper()
        self._mission_state_t = self.now_sec()
        self._finish_or_reject_rearm(self._mission_state)

    def on_cmd(self, msg: String):
        """§82.5 — 테이크 사이 재무장.

        🔴 §83.4 — `REARMED` 를 **ack 로 쓴다.** 런북은 `-w 1 --times 3` 로 보내고
        이 문자열을 눈으로 확인한 뒤에야 다음 단계로 간다. 안 보이면 DDS 매칭이
        안 된 것이므로 재전송한다 — 단발 `--once` 는 discovery 전에 유실된다.
        """
        cmd = (msg.data or '').strip().lower()
        if cmd != 'rearm':
            if cmd:
                self.get_logger().warn(f'알 수 없는 명령 "{cmd}" — rearm 만 받는다')
            return

        # 🔴 §83.4 관문 — 이미 출동 중인데 재무장하면 지속 오탐이 새 테이크를
        #   자동으로 시작시킨다. 상태 미수신도 거부다(fail-closed).
        if self.get_parameter('rearm_requires_patrol').value:
            st = self._mission_state
            if st is None:
                self.get_logger().error(
                    '🔴 재무장 거부 — 미션 상태를 한 번도 못 받았다. 미션이 떠 있는지 '
                    '확인할 것 (미션 없이 시험하려면 rearm_requires_patrol:=false)')
                self.say('REARM_REJECTED (미션 상태 없음)')
                return
            # 🔴 §84.3 — **완전일치**로 좁힌다. 구판 `'PATROL' not in st` 는
            #   `NOT_PATROL` · `PATROLLING` · `BLOCKED PATROL` 을 전부 통과시켰다
            #   (직접 입력 재현: 셋 다 REARMED 를 냈다).
            if st != 'PATROL':
                self.get_logger().error(
                    f'🔴 재무장 거부 — 미션이 "{st}" 다(PATROL 완전일치 아님). '
                    f'지금 재무장하면 지속 오탐이 다음 출동을 자동으로 시작시킨다. '
                    f'미션을 reset 해 PATROL 로 돌린 뒤 다시 보낼 것')
                self.say(f'REARM_REJECTED ({st})')
                return
            # 🔴 §86.4 — 전달용 중복은 **하나의 의도**다. 성공 직후의 재요청과
            #   이미 열려 있는 요청을 새 handshake 로 만들지 않는다.
            dedup = self.get_parameter('rearm_dedup_sec').value
            if (self._last_rearm_t is not None
                    and self.now_sec() - self._last_rearm_t <= dedup):
                self.get_logger().info(
                    '🔵 재무장 중복 무시 — 방금 재무장했다(전달용 재전송으로 본다)',
                    throttle_duration_sec=2.0)
                self.say('REARM_DUP_IGNORED')
                return
            if self._rearm_pending_since is not None:
                self.get_logger().info(
                    '🔵 재무장 요청이 이미 열려 있다 — 중복 무시',
                    throttle_duration_sec=2.0)
                self.say('REARM_DUP_IGNORED')
                return
            # 🔴 §85.4 — 캐시가 PATROL 이어도 **지금** PATROL 이라는 뜻은 아니다.
            #   여기서 끝내지 않고, **요청 이후에 새로 오는 관측**을 기다린다.
            #   그 관측이 PATROL 이면 그때 재무장한다(`on_mission_state` 가 마무리).
            self._rearm_pending_since = self.now_sec()
            self.get_logger().warn(
                '🔶 재무장 요청 접수 — 다음 /mission_state 관측을 기다린다. '
                'REARMED 를 보기 전에는 다음 단계로 가지 말 것')
            self.say('REARM_PENDING (다음 PATROL 관측 대기)')
            return

        self._do_rearm()

    def _do_rearm(self):
        """실제 재무장 — 관문을 통과한 자리에서만 부른다."""
        self.fired_at = None
        self.tracker.reset()
        self._last_reason = '재무장됨 — 다음 테이크 대기'
        self._last_rearm_t = self.now_sec()      # §86.4 중복 병합 기준
        self.get_logger().warn('🔵 재무장 — 발사 이력과 누적 관측을 지웠다')
        self.say('REARMED')

    # ===================================================================
    # 🆕 사람 경로 (08-22) — `PROJECT_CONTEXT §4.1-b`
    # 🔴 화재 경로와 **한 줄도 공유하지 않는다.** 어댑터는 검토 다섯 회차로 동결한
    #    사슬이고, 촬영 전날 밤에 그 안을 리팩터링할 이유가 없다. 아래 map 변환이
    #    `on_detections` 의 것과 겹치는 것은 **의도된 중복**이다.
    #    ⏸ 합칠 조건 = 촬영이 끝나고 사람 경로가 독립 검토를 받은 뒤.
    # ===================================================================
    PERSON_CLASSES = ('person_fallen', 'person_ok', 'person_unknown')

    def _person_to_map(self, det, header):
        """탐지 1건의 위치를 map 으로. 실패하면 None (신고 좌표 없이 상태만 간다)."""
        src = header.frame_id or self.get_parameter('fallback_source_frame').value
        expect = self.get_parameter('expected_source_frame').value
        if not src or (expect and src != expect):
            return None
        if not is_finite_point(det.position.x, det.position.y, det.position.z):
            return None
        pt = PointStamped()
        pt.header = header
        pt.point.x, pt.point.y, pt.point.z = (
            det.position.x, det.position.y, det.position.z)
        for stamp in (rclpy.time.Time.from_msg(header.stamp), rclpy.time.Time()):
            # 촬영시각 → 없으면 최신 TF. 회전 중(SCAN_AREA)이면 최신은 밀리지만,
            # 좌표가 조금 밀리는 것과 **신고를 못 하는 것**은 값이 다르다.
            try:
                tr = self.tf_buffer.lookup_transform(
                    self.target_frame, src, stamp,
                    timeout=rclpy.duration.Duration(seconds=0.1))
                out = do_transform_point(pt, tr)
                if is_finite_point(out.point.x, out.point.y, out.point.z):
                    return (out.point.x, out.point.y)
            except Exception:                                    # noqa: BLE001
                continue
        return None

    @staticmethod
    def _conf_ok(d, min_conf):
        """🔴 08-22 (§87.5) — 유한한 0~1 만 신뢰한다.

        구판은 `conf < min_conf` 로 건너뛰기만 했다. 파이썬에서 `NaN < 0.5` 는
        **False** 라 NaN 이 그대로 통과했고, `Inf` 도 통과했다. 검토가 실제로
        재현했다: `person_ok(conf=NaN)` → `ok`. 즉 **쓰레기 한 프레임이
        "떠나도 된다"로 접혔다.**
        """
        c = getattr(d, 'confidence', None)
        if not isinstance(c, (int, float)) or isinstance(c, bool):
            return False
        return math.isfinite(c) and 0.0 <= c <= 1.0 and c >= min_conf

    def _person_frame_verdict(self, msg):
        """이 프레임 한 장의 판정 — fallen | ok | unknown | none.

        🔴 **두 층으로 나눈다** (08-22 §87.5 보완). 구판은 신뢰도 문턱을 먼저
        적용하고 남은 것이 없으면 `none` 으로 접었다. 그래서 **저조도에서 사람
        후보가 계속 낮은 confidence 로 오면 4초 뒤 `NO_VICTIM`** 이 됐다 —
        사람이 눈앞에 있는데 "아무도 없다"고 신고하는 것이다.
          ① **사람 후보가 있는가** — 신뢰도와 무관하게 class 만 본다
          ② **자세를 신뢰할 수 있는가** — 유한한 0~1 이고 문턱 이상인가

        우선순위 **fallen > unknown > ok > none**:
          · 신뢰 가능한 `fallen` 이 하나라도 있으면 `fallen` (유기는 못 되돌린다)
          · **모든 후보가 신뢰 가능한 `ok`** 일 때만 `ok` — 하나라도 판정 불가면
            `unknown` 이 이긴다. 구판은 여러 사람 중 한 명이 unknown 이어도 다른
            ok 한 명 때문에 **떠났다**(§87.5 재현).
          · 후보가 전혀 없는 프레임에서만 `none`
        """
        min_conf = float(self.get_parameter('person_min_confidence').value)
        cands = [d for d in msg.detections
                 if getattr(d, 'class_name', '') in self.PERSON_CLASSES]
        if not cands:
            return 'none', None            # ① 후보 자체가 없다 = 정상 미탐지

        best_fallen = None
        all_trusted_ok = True
        for d in cands:
            trusted = self._conf_ok(d, min_conf)
            name = d.class_name
            if trusted and name == 'person_fallen':
                if best_fallen is None or d.confidence > best_fallen.confidence:
                    best_fallen = d
            if not (trusted and name == 'person_ok'):
                all_trusted_ok = False
        if best_fallen is not None:
            return 'fallen', best_fallen
        if all_trusted_ok:
            return 'ok', None
        return 'unknown', None             # 후보는 있는데 자세를 못 믿는다

    def _p_set_status(self, v):
        """상태 전이 + `fallen` 상승엣지에서 `/victim` 1회 발행."""
        if v == 'fallen':
            if not self._p_victim_sent and self._p_victim_pos is not None:
                m = PoseStamped()
                m.header.frame_id = self.target_frame
                m.header.stamp = self.get_clock().now().to_msg()
                m.pose.position.x, m.pose.position.y = self._p_victim_pos
                m.pose.position.z = 0.0        # 미션은 평면만 쓴다
                m.pose.orientation.w = 1.0
                self.victim_pub.publish(m)
                self._p_victim_sent = True
                self.get_logger().error(
                    f'🔴 쓰러진 사람 확정 → /victim 발행 '
                    f'({self._p_victim_pos[0]:.2f}, {self._p_victim_pos[1]:.2f})')
        else:
            # 🔵 상태가 fallen 을 벗어나면 재무장한다 — 같은 임무에서 두 번째
            #   쓰러짐이 생겼을 때 신고가 막히면 안 된다.
            self._p_victim_sent = False
            # ⚠ 08-22 — 여기서 좌표를 지우면 **안 된다.** 확정 전 streak 를 쌓는
            #   동안 상태는 `unknown` 이라 이 가지를 매 프레임 지나간다. 그때 지우면
            #   앞 프레임들이 얻어둔 변환 결과가 전부 버려지고, **확정되는 그 한
            #   프레임에서 TF 가 실패하면 신고 좌표가 없다.** 검토(§87.6)가 요구한
            #   것은 "그 **세대**에서 성공한 좌표" 이지 "확정 프레임의 좌표" 가 아니다.
            #   폐기는 **세대 경계**(`_p_reset_streak` · 새 fallen streak)에서 한다.
        if v != self._p_status:
            self.get_logger().info(f'사람 상태 {self._p_status} → {v}')
        self._p_status = v

    def _p_reset_streak(self):
        self._p_streak_class = None
        self._p_streak_since = None
        self._p_streak_frames = 0
        self._p_victim_pos = None      # §87.6 — 세대가 끊기면 좌표도 끊긴다

    def _update_person(self, msg, t):
        """🔴 `on_detections` 의 **맨 앞**에서 불린다 — 그 아래 어떤 return 에도 걸리면 안 된다.

        특히 ①번 재발사 억제는 화재가 한 번 나가면 기본값이 **평생 return** 이다.
        사람 판정을 그 뒤에 두면 **화재 경보가 나간 순간 죽는다** — 그런데
        `SCAN_AREA`(사람 찾기)는 바로 그 다음 국면이다.
        """
        self._p_last_det_t = t

        # stamp 신선도를 **자체로** 본다 (화재 경로의 검사를 빌리지 않는다).
        st = msg.header.stamp
        age = t - (st.sec + st.nanosec / 1e9)
        if abs(age) > float(self.get_parameter('max_stamp_age_sec').value):
            # 🔴 "오고는 있는데 못 믿는다" 는 `unknown` 이다. `stale`(안 온다)도
            #   `none`(봤는데 없다)도 아니다. 셋을 섞으면 아무도 없는 자리에서
            #   유도가 시작되거나 센서가 죽은 채로 신고가 나간다.
            self._p_reset_streak()
            self._p_set_status('unknown')
            return

        verdict, best = self._person_frame_verdict(msg)
        if verdict == 'unknown':
            self._p_reset_streak()
            self._p_set_status('unknown')
            return

        if verdict != self._p_streak_class:
            self._p_streak_class = verdict
            self._p_streak_since = t
            self._p_streak_frames = 0
            if verdict == 'fallen':
                # 🔴 새 fallen 세대는 **자기 세대에서 성공한 변환만** 쓴다.
                #   옛 좌표를 물려받으면 못 얻었을 때 조용히 그것이 나간다.
                self._p_victim_pos = None
        self._p_streak_frames += 1

        if verdict == 'fallen' and best is not None:
            pos = self._person_to_map(best, msg.header)
            if pos is not None:
                self._p_victim_pos = pos

        # 🔴 `fallen` 은 빠르게, **떠나도 된다는 판정(`ok`·`none`)은 신중하게.**
        need = float(self.get_parameter(
            'person_confirm_sec_fallen' if verdict == 'fallen'
            else 'person_confirm_sec_leave').value)
        frames_ok = self._p_streak_frames >= int(
            self.get_parameter('person_min_frames').value)
        if (t - self._p_streak_since) >= need and frames_ok:
            self._p_set_status(verdict)
        else:
            # 판정이 아직 안 섰다 = 보류. 미션은 여기서 아무 분기도 하지 않는다.
            self._p_set_status('unknown')

    def _person_tick(self):
        """10 Hz 상시 발행 + 침묵 감지.

        🔴 `/detections` 콜백에만 매달면 **발행이 끊긴 순간 아무 말도 못 한다.**
        그 침묵이야말로 미션이 알아야 할 정보다 — 마지막 상태가 `ok` 였다면
        미션은 사람이 계속 따라오는 줄 안다.
        """
        t = self.now_sec()
        stale_sec = float(self.get_parameter('person_stale_sec').value)
        if self._p_last_det_t is None or (t - self._p_last_det_t) > stale_sec:
            if self._p_status != 'stale':
                self._p_reset_streak()
                self._p_set_status('stale')
        m = String()
        m.data = self._p_status
        self.person_status_pub.publish(m)

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def say(self, text):
        m = String()
        m.data = text
        self.status_pub.publish(m)

    def _expire_rearm(self):
        """§85.4 — 새 관측이 상한 안에 안 오면 요청을 버린다. 조용히 두면
        운영자가 REARMED 를 영원히 기다린다."""
        if self._rearm_pending_since is None:
            return
        limit = self.get_parameter('rearm_ack_timeout_sec').value
        if self.now_sec() - self._rearm_pending_since <= limit:
            return
        self._rearm_pending_since = None
        self.get_logger().error(
            f'🔴 재무장 만료 — {limit}s 안에 새 /mission_state 관측이 없었다. '
            f'미션이 살아 있는지 확인하고 다시 보낼 것')
        self.say('REARM_REJECTED (관측 없음 — 만료)')

    def tick(self):
        """2초마다 현재 상태를 흘린다.

        ⚠ 이게 왜 필요한가 — 어댑터가 '안 쏘고 있다' 는 상태가 두 가지다:
          ① 불이 안 보인다(정상)  ② 뭔가 막혀 있다(고장).
          로그가 없으면 둘이 똑같이 보인다. 08-20 에 우리가 세 번 당한 그 형태다.
        """
        t = self.now_sec()
        self._expire_rearm()          # §85.4 — handshake 대기 상한
        if self.fired_at is not None:
            self.say(f'FIRED at t={self.fired_at:.1f}')
            return
        self.say(f'frames={self._frames} hits={self.tracker.count(t)}/'
                 f'{self.tracker.need} last={self._last_reason}')

    # ------------------------------------------------------------------
    def on_detections(self, msg: Detection3DArray):
        self._frames += 1
        t = self.now_sec()

        # 🆕 08-22 — 🔴 **맨 앞이어야 한다.** 아래 ①(재발사 억제)은 화재가 한 번
        #   나가면 기본값이 평생 return 이고, `SCAN_AREA` 는 그 **다음** 국면이다.
        #   여기 순서를 내리면 사람 판정이 화재 경보와 함께 죽는다.
        self._update_person(msg, t)

        # ── ① 재발사 억제 ──────────────────────────────────────────────
        cooldown = self.get_parameter('refire_cooldown_sec').value
        if self.fired_at is not None:
            if cooldown is None or cooldown <= 0.0:
                return                       # 평생 1회
            if t - self.fired_at < cooldown:
                return
            self.fired_at = None
            self.tracker.reset()

        # ── ② stamp 신선도 ─────────────────────────────────────────────
        # 🔴 계약상 stamp 는 **color 촬영시각**이다(합의사항 §4.3).
        #    오래된 프레임으로 출동하면 이미 지나간 위치로 간다.
        st = msg.header.stamp
        age = stamp_age_sec(t, st.sec + st.nanosec / 1e9)
        max_age = self.get_parameter('max_stamp_age_sec').value
        if abs(age) > max_age:
            # ⚠ abs() 인 이유 — 미래 stamp 도 거부한다. 시계가 어긋난 것이고,
            #   그 상태의 좌표를 믿을 근거가 없다.
            self._last_reason = f'stamp {age:+.2f}s (허용 ±{max_age})'
            return

        # ── ③ 대상 클래스 고르기 ───────────────────────────────────────
        want = self.get_parameter('trigger_class').value
        min_conf = self.get_parameter('min_confidence').value
        best, violations = pick_best(msg.detections, want, min_conf)
        for bad in violations:
            self.get_logger().warn(
                f'🔴 계약 위반 class_name="{bad}" — 무시. '
                f'허용 = {VALID_CLASSES}', throttle_duration_sec=5.0)

        if best is None:
            # 빈 배열은 "정상 미탐지"다 — 실패가 아니다(합의사항 §6).
            self._last_reason = f'{want} 없음 (탐지 {len(msg.detections)}건)'
            return

        # ── ④ 좌표계 검증 ──────────────────────────────────────────────
        # 🔴 08-21 §83.2 재현 반영 — **검증보다 tracker 가 먼저 돌고 있었다.**
        #   구판 순서는 ③ → tracker → 거리 → frame 검사였다. 그래서
        #   `frame_id=base_link` 인 (결국 거부될) 프레임 5장이 hit 를 쌓아 두고,
        #   그다음 정상 프레임 **한 장**이 즉시 `/alarm` 으로 승격됐다.
        #   재현: 거부 5장 뒤 `tracker.count()==5`, 정상 1장에 True.
        #   → **완전히 통과한 후보만 센다.** 검증은 전부 tracker 앞으로 옮긴다.
        src = msg.header.frame_id or self.get_parameter('fallback_source_frame').value
        if not src:
            self.get_logger().error(
                '🔴 header.frame_id 가 비어 있다 — 좌표계 불명. 변환 불가. '
                '(fallback_source_frame 파라미터로 지정 가능)')
            self._last_reason = 'frame_id 없음'
            return
        # 🔴 §82.3 — 기대 frame 잠금. 구판은 "비었나" 만 봤다. 그래서 TF 트리에
        #   존재하는 아무 frame(`base_link` 등)도 조용히 map 으로 변환됐고,
        #   3m 투영 게이트 안이면 **잘못된 화재 위치가 첫 goal 을 정했다.**
        expect = self.get_parameter('expected_source_frame').value
        if expect and src != expect:
            self.get_logger().error(
                f'🔴 source frame 불일치: "{src}" (기대 "{expect}") — 변환 거부. '
                f'계약은 color optical frame 이다(합의사항 §4.3). '
                f'검사를 끄려면 expected_source_frame:="" (비권장)',
                throttle_duration_sec=5.0)
            self._last_reason = f'frame 불일치 {src}≠{expect}'
            return

        # ── ⑤ 거리 처리 ────────────────────────────────────────────────
        px, py, pz = best.position.x, best.position.y, best.position.z
        degraded_range = False
        if self.get_parameter('use_fixed_range').value:
            r = fix_range(px, py, pz, self.get_parameter('fixed_range').value)
            if r is None:
                self._last_reason = '거리 0 — 방위가 없어 고정거리 적용 불가'
                return
            px, py, pz = r
            degraded_range = True
        else:
            px, py, pz, clamped = clamp_range(
                px, py, pz, self.get_parameter('max_range').value)
            if clamped:
                self.get_logger().warn(
                    '⚠ 거리 클램프 발동 — depth 가 max_range 를 넘었다. '
                    '방위는 살리고 거리만 잘랐다.')
                degraded_range = True

        # ── ⑥ map 으로 변환 ────────────────────────────────────────────
        pt = PointStamped()
        pt.header.frame_id = src
        pt.header.stamp = msg.header.stamp
        pt.point.x, pt.point.y, pt.point.z = px, py, pz

        # 🔴 §82.3 — **촬영시각의 TF** 로 조회한다.
        #   구판은 `rclpy.time.Time()`(=최신)을 썼고, 주석에 "속도 0.087 m/s 면
        #   이동은 cm 단위" 라고 근거까지 적어놨다. 그 계산이 **병진만** 본 것이다.
        #   회전 중에는 이동이 아니라 **각도**가 문제다 — 제자리 0.13 rad/s 로도
        #   0.3초면 2.2°, 2m 앞 목표가 7.7cm 옮겨간다. 코너에서는 더 크다.
        #
        # 🔴 08-21 §83.3 재현 반영 — 후퇴가 **기본값이면 그 계약은 fail-open 이다.**
        #   구판은 `allow_latest_tf_fallback` 기본이 True 였다. detection 이 같은
        #   시각의 동적 TF 보다 조금 먼저 도착하는 **정상 비동기 순서**에서도
        #   future extrapolation 이 나므로, 후퇴는 예외가 아니라 상시 경로였다.
        #   → 기본을 False 로 내리고, 대신 짧은 **대기**(tf_wait_sec)로 정상 도착
        #     순서만 흡수한다. 최신 TF 는 사람이 명시적으로 켠 도전 모드다.
        stamp_time = rclpy.time.Time.from_msg(msg.header.stamp)
        wait = self.get_parameter('tf_wait_sec').value
        stamp_err, out = None, None
        try:
            tr = self.tf_buffer.lookup_transform(
                self.target_frame, src, stamp_time,
                timeout=rclpy.duration.Duration(seconds=float(wait)))
            out = do_transform_point(pt, tr)
        except Exception as e:
            # ⚠ 파이썬은 except 블록을 벗어나면 `as e` 를 지운다 — 문자열로 붙잡는다.
            stamp_err = str(e)

        degraded_tf = False
        if out is None:
            if not self.get_parameter('allow_latest_tf_fallback').value:
                self.get_logger().warn(
                    f'TF 변환 실패 (촬영시각 {src}→{self.target_frame}, '
                    f'{wait}s 대기): {stamp_err} — 후퇴 금지(기본)라 발행하지 않는다',
                    throttle_duration_sec=3.0)
                self._last_reason = f'TF 실패(촬영시각) {src}→{self.target_frame}'
                return
            try:
                tr = self.tf_buffer.lookup_transform(
                    self.target_frame, src, rclpy.time.Time())
                out = do_transform_point(pt, tr)
            except Exception as e:
                self.get_logger().warn(
                    f'TF 변환 실패 ({src} → {self.target_frame}): {e}',
                    throttle_duration_sec=3.0)
                self._last_reason = f'TF 실패 {src}→{self.target_frame}'
                return
            degraded_tf = True
            self._tf_degraded = True
            self.get_logger().warn(
                f'⚠ 촬영시각 TF 없음 → 최신 TF 로 후퇴 (격하). 회전 중이면 좌표가 '
                f'밀린다. 사유: {stamp_err}', throttle_duration_sec=5.0)

        if not is_finite_point(out.point.x, out.point.y, out.point.z):
            self.get_logger().error('🔴 변환 결과가 유한값이 아님 — 발행 취소')
            return

        # ── ⑦ 반복 관측 확정 (map 좌표에서) ────────────────────────────
        # 🔴 §82.4 — 촬영시각과 좌표를 함께 넘긴다. 수신시각만으로는 같은 프레임
        #   한 장의 재전송과 진짜 반복 관측을 구별할 수 없다(재현: [F,F,F,F,True]).
        # 🔴 §83.2 — **map 좌표로 센다.** 카메라 좌표에서는 로봇이 돌기만 해도
        #   같은 화재가 움직여 결합이 끊긴다. map 에서는 정지 화재가 제자리다.
        stamp_sec = st.sec + st.nanosec / 1e9
        pos = (out.point.x, out.point.y, out.point.z)
        if not self.tracker.add(t, stamp_sec, pos):
            self._last_reason = (f'{want} conf={best.confidence:.2f} '
                                 f'확정대기 {self.tracker.count(t)}/{self.tracker.need}')
            return

        # ── ⑦ 발행 ─────────────────────────────────────────────────────
        a = PoseStamped()
        a.header.frame_id = self.target_frame
        a.header.stamp = self.get_clock().now().to_msg()
        a.pose.position.x = out.point.x
        a.pose.position.y = out.point.y
        a.pose.position.z = 0.0          # 미션은 평면만 쓴다
        a.pose.orientation.w = 1.0
        self.alarm_pub.publish(a)
        self.fired_at = t

        # 🔴 §83.3 — 격하 두 종류를 **합치지 않는다.** 구판은 TF 후퇴도
        #   `[격하: 거리 보정됨]` 으로 표시해, 운영자가 좌표 **시각**이 격하된
        #   것을 거리 clamp 로 오독할 수 있었다. 원인이 다르면 문구도 다르다.
        marks = []
        if degraded_range:
            marks.append('거리 보정됨')
        if degraded_tf:
            marks.append('좌표 시각 = 최신 TF (회전 중이면 밀림)')
        mark = f' 🔴[격하: {" · ".join(marks)}]' if marks else ''
        self.get_logger().warn(
            f'🔥 화재 확정 → /alarm 발행 '
            f'({out.point.x:.2f}, {out.point.y:.2f}) '
            f'conf={best.confidence:.2f} '
            f'관측 {self.tracker.need}회/{self.tracker.window_sec}s{mark}')
        self.say(f'FIRED ({out.point.x:.2f},{out.point.y:.2f}){mark}')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()          # §85.2 — 전용 TF 노드까지 여기서 정리된다
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
