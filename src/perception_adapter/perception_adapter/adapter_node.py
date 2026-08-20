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
"""

import math

import rclpy
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
        if self._hits and self._far(pos, self._hits[-1][2], self.assoc_radius):
            self._hits = []              # 다른 대상 — 누적을 합치지 않는다
        self._last_stamp = s
        self._hits.append((t, s, pos))
        self._prune(t)
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
    num('confirm_frames', lo=1, integer=True)
    num('confirm_window_sec', positive=True)
    num('max_stamp_age_sec', positive=True)
    num('max_range', positive=True)
    num('fixed_range', positive=True)
    num('confirm_assoc_radius_m', positive=True)
    # refire_cooldown_sec 은 0 이하가 "평생 1회" 라는 뜻이라 부호를 안 막는다.
    # 다만 NaN/Inf 는 비교가 전부 False 가 되어 억제가 조용히 사라진다.
    num('refire_cooldown_sec')
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
        self.declare_parameter('min_confidence', 0.40)
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
        # 🔴 §82.3 — 기대 source frame. 빈 문자열이면 검사 안 함(비권장).
        #    이게 없으면 TF 트리에 있는 아무 frame(base_link 등)도 조용히 통과한다.
        self.declare_parameter('expected_source_frame', 'camera_color_optical_frame')
        # 🔴 §82.3 — 촬영시각 TF 가 없을 때 최신 TF 로 후퇴할지. 후퇴하면 격하 표시.
        self.declare_parameter('allow_latest_tf_fallback', True)
        # §82.5 — 테이크 사이 재무장 명령 토픽 (std_msgs/String "rearm")
        self.declare_parameter('cmd_topic', '/adapter_cmd')

        p = self.get_parameter
        self.detections_topic = p('detections_topic').value
        self.alarm_topic = p('alarm_topic').value
        self.target_frame = p('target_frame').value

        # ── TF ─────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

        # 🔴 §82.4 — 수치 파라미터를 기동 시 한 곳에서 검증하고, 실패하면
        #    **크게 죽는다.** 재현: max_range=-1 이면 clamp_range 가 (2,0,0) 을
        #    (-1,0,0) 으로 뒤집어 로봇 **뒤쪽** 좌표를 만들었다. 조용히 도는 것보다
        #    안 뜨는 편이 낫다 — 안 뜨면 관제 수동 클릭이라는 후퇴로가 살아 있다.
        bad = validate_params({
            'min_confidence': p('min_confidence').value,
            'confirm_frames': p('confirm_frames').value,
            'confirm_window_sec': p('confirm_window_sec').value,
            'max_stamp_age_sec': p('max_stamp_age_sec').value,
            'max_range': p('max_range').value,
            'fixed_range': p('fixed_range').value,
            'refire_cooldown_sec': p('refire_cooldown_sec').value,
            'confirm_assoc_radius_m': p('confirm_assoc_radius_m').value,
        })
        if bad:
            for m in bad:
                self.get_logger().error(f'🔴 파라미터 거부 — {m}')
            raise ValueError(f'perception_adapter 파라미터 {len(bad)}건 불량: {bad}')

        # ── 상태 ────────────────────────────────────────────────────────
        self.tracker = ConfirmTracker(p('confirm_frames').value,
                                      p('confirm_window_sec').value,
                                      p('confirm_assoc_radius_m').value)
        self.fired_at = None
        self._frames = 0
        self._last_reason = '아직 프레임 없음'
        self._tf_degraded = False

        # §82.5 재무장 — 촬영은 3~5 테이크다. 구판은 refire_cooldown_sec=-1 이라
        #   **프로세스 평생 1회**였고, 미션 reset 을 보지 않아 두 번째 테이크부터
        #   어댑터가 조용히 안 쐈다. 단순 쿨다운은 이전 테이크의 지속 오탐이 새
        #   테이크를 자동 시작시켜서 더 나쁘다 — 사람이 명시적으로 재무장한다.
        self.create_subscription(
            String, p('cmd_topic').value, self.on_cmd, 10)

        self.create_timer(2.0, self.tick)

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
    def on_cmd(self, msg: String):
        """§82.5 — 테이크 사이 재무장. `ros2 topic pub --once /adapter_cmd ... rearm`"""
        cmd = (msg.data or '').strip().lower()
        if cmd == 'rearm':
            self.fired_at = None
            self.tracker.reset()
            self._last_reason = '재무장됨 — 다음 테이크 대기'
            self.get_logger().warn('🔵 재무장 — 발사 이력과 누적 관측을 지웠다')
            self.say('REARMED')
        elif cmd:
            self.get_logger().warn(f'알 수 없는 명령 "{cmd}" — rearm 만 받는다')

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def say(self, text):
        m = String()
        m.data = text
        self.status_pub.publish(m)

    def tick(self):
        """2초마다 현재 상태를 흘린다.

        ⚠ 이게 왜 필요한가 — 어댑터가 '안 쏘고 있다' 는 상태가 두 가지다:
          ① 불이 안 보인다(정상)  ② 뭔가 막혀 있다(고장).
          로그가 없으면 둘이 똑같이 보인다. 08-20 에 우리가 세 번 당한 그 형태다.
        """
        t = self.now_sec()
        if self.fired_at is not None:
            self.say(f'FIRED at t={self.fired_at:.1f}')
            return
        self.say(f'frames={self._frames} hits={self.tracker.count(t)}/'
                 f'{self.tracker.need} last={self._last_reason}')

    # ------------------------------------------------------------------
    def on_detections(self, msg: Detection3DArray):
        self._frames += 1
        t = self.now_sec()

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

        # ── ④ 반복 관측 확정 ───────────────────────────────────────────
        # 🔴 §82.4 — 촬영시각과 좌표를 함께 넘긴다. 수신시각만으로는 같은 프레임
        #   한 장의 재전송과 진짜 반복 관측을 구별할 수 없다(재현: [F,F,F,F,True]).
        stamp_sec = st.sec + st.nanosec / 1e9
        bp = best.position
        if not self.tracker.add(t, stamp_sec, (bp.x, bp.y, bp.z)):
            self._last_reason = (f'{want} conf={best.confidence:.2f} '
                                 f'확정대기 {self.tracker.count(t)}/{self.tracker.need}')
            return

        # ── ⑤ 거리 처리 ────────────────────────────────────────────────
        px, py, pz = best.position.x, best.position.y, best.position.z
        degraded = False
        if self.get_parameter('use_fixed_range').value:
            r = fix_range(px, py, pz, self.get_parameter('fixed_range').value)
            if r is None:
                self._last_reason = '거리 0 — 방위가 없어 고정거리 적용 불가'
                return
            px, py, pz = r
            degraded = True
        else:
            px, py, pz, clamped = clamp_range(
                px, py, pz, self.get_parameter('max_range').value)
            if clamped:
                self.get_logger().warn(
                    '⚠ 거리 클램프 발동 — depth 가 max_range 를 넘었다. '
                    '방위는 살리고 거리만 잘랐다.')
                degraded = True

        # ── ⑥ map 으로 변환 ────────────────────────────────────────────
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

        pt = PointStamped()
        pt.header.frame_id = src
        pt.header.stamp = msg.header.stamp
        pt.point.x, pt.point.y, pt.point.z = px, py, pz
        # 🔴 §82.3 — **촬영시각의 TF** 로 조회한다.
        #   구판은 `rclpy.time.Time()`(=최신)을 썼고, 주석에 "속도 0.087 m/s 면
        #   이동은 cm 단위" 라고 근거까지 적어놨다. 그 계산이 **병진만** 본 것이다.
        #   회전 중에는 이동이 아니라 **각도**가 문제다 — 제자리 0.13 rad/s 로도
        #   0.3초면 2.2°, 2m 앞 목표가 7.7cm 옮겨간다. 코너에서는 더 크다.
        #   버퍼에 그 시점이 없으면(초기화 직후 등) 최신으로 후퇴하되 **격하 표시**한다.
        stamp_err = None
        out = None
        try:
            tr = self.tf_buffer.lookup_transform(
                self.target_frame, src, rclpy.time.Time.from_msg(msg.header.stamp))
            out = do_transform_point(pt, tr)
        except Exception as e:
            # ⚠ 파이썬은 except 블록을 벗어나면 `as e` 를 지운다 — 문자열로 붙잡는다.
            stamp_err = str(e)

        if out is None:
            if not self.get_parameter('allow_latest_tf_fallback').value:
                self.get_logger().warn(
                    f'TF 변환 실패 (촬영시각 {src}→{self.target_frame}): {stamp_err} '
                    f'— 후퇴 금지 설정이라 발행하지 않는다', throttle_duration_sec=3.0)
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
            degraded = True
            self._tf_degraded = True
            self.get_logger().warn(
                f'⚠ 촬영시각 TF 없음 → 최신 TF 로 후퇴 (격하). 회전 중이면 좌표가 '
                f'밀린다. 사유: {stamp_err}', throttle_duration_sec=5.0)

        if not is_finite_point(out.point.x, out.point.y, out.point.z):
            self.get_logger().error('🔴 변환 결과가 유한값이 아님 — 발행 취소')
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

        mark = ' 🔴[격하: 거리 보정됨]' if degraded else ''
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
