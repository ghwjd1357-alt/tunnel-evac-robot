# -*- coding: utf-8 -*-
"""
test_gate_fakes_periods.py — 가짜 센서 '주기'가 실물과 같은지, 그리고 갈라지지 않는지.

============================================================
[왜 필요한가 — 예약 18]
  게이트 회귀(`tools/test_gate_regression.sh`)의 입력을 만드는 것이 `gate_fakes.py` 다.
  그 하네스가 실물과 **다른 입력**을 만들면, 게이트가 PASS 를 줘도 그 PASS 가 실차를
  대변하지 않는다. 예약 17(무상한 CLI)과 같은 종류의 결함이다 —
  *"판정기·하네스가 실물과 다른 입력을 만들어 통과를 준다."*

  실제로 갈라져 있었다. 같은 사실이 세 자리에 서로 다르게 적혀 있었다:
    타이머(코드) 50Hz  /  독스트링 100Hz  /  구동부 2차 회신 실측 41.63Hz.

[★ 08-01 §21 검토 보완 — 이 파일 자신이 같은 결함이었다]
  초판은 `_simulate_rcl_timer()` 라는 **시뮬레이터를 테스트 안에 다시 구현**하고
  그 출력을 단언했다. 그래서 생산 코드의 커서를 부숴도(`+= 1` → `+= 0`) 검사는
  전부 통과했다 — *채점자가 답안을 안 보고 자기가 푼 답을 채점한 것*이다.
  같은 이유로 정본 봉쇄도 `<숫자>Hz` 표기 하나만 봐서, 단위어 없는 상수를 블록
  밖으로 옮기면 그냥 통과했다. 두 사각 모두 반례 주입으로 재현됐다 (거짓 PASS 11/11).
  → 시뮬레이터를 **삭제**하고 실제 rclpy 타이머를 돌린다.
  → `Hz` 정규식 대신 **의존 폐포**(그 값을 만드는 정의가 전부 블록 안인가)를 본다.
  → 반례 4종을 **영구 부정 회귀**로 박제한다 (한 번 해보고 마는 것은 또 뚫린다).

[★ 08-01 검토 §22 보완 — 같은 실수가 한 겹 더 있었다]
  §21 보완도 불승인됐다. 검토자가 세 가지 거짓 PASS 를 실행으로 재현했고, 전부 재현됐다:
    ① 폐포가 **뿌리 자신의 위치**를 안 봤다 → `SENSOR_PERIODS` 표를 통째로 블록 밖으로
       옮겨도 "밖에 0건". 정본 블록이 텅 비어도 녹색이었다.
    ② 자유변수 계산이 `ast.walk()` 로 모든 `Store` 를 평평하게 걷어 "지역"으로 쳤다.
       그런데 comprehension·중첩 함수·lambda·class 본문은 **각각 다른 스코프**다 —
       이름을 가리면 블록 밖 값을 읽으면서도 통과했다 (검토자가 실행으로 증명).
    ③ 분류 허용치 ±8ms 하나가 **두 일을 겸했다**. 순서 분류에 필요한 여유가 같은 열
       안의 +7ms 체계 편향을 통째로 승인했다 (생산 `period_ns()` 변이로 3/3 거짓 PASS).
  근인은 §21 과 **같은 모양**이다 — 언어/생산의 의미론을 테스트 안에 다시 구현한 것.
  이번엔 스코프 규칙을 '더 정확히 다시 짜는' 길을 택하지 않는다:
  → 스코프 판단을 표준 라이브러리 **`symtable`(= CPython 자신)** 에 **물어본다**.
  → 뿌리도 위치 검사 대상에 넣고, 정적 분석이 못 보는 문법은 **fail-closed** 한다.
  → 순서 계약과 **시간 편향 계약**을 분리한다 (잔차 중앙값, 임계 근거는 아래 상수).

[이 파일이 지키는 것 — 값이 아니라 '갈라질 자리']
  1. IMU 주기 열이 **실측 5통계를 재현**하는가 (평균·min·max·σ·창)  — 순수 계산
  2. 등간격으로 되돌리면 **깨지는가** (부정 회귀 — 이게 없으면 조용히 되돌아간다)
  3. **실제 rclpy 타이머**가 계획한 순서대로 콜백을 부르는가 — 생산 배선 통과
  3-b. 그 간격이 계획 대비 **체계적으로 치우치지** 않았는가 (순서와 별개 계약)
  4. 주기 값을 만드는 정의가 **정본 블록 안에 닫혀 있는가** (폐포 — 표기 무관)
  5. 실측을 못 받은 항목이 **'미확보'로 선언**돼 있는가 (추정으로 채우면 또 다른 거짓 입력)
  6. EKF 주기와의 관계가 **문서에 적힌 그대로**인가 (한쪽이 바뀌면 여기서 걸린다)

[왜 295표본을 실시간으로 다시 재지 않는가]
  실측 창 한 바퀴는 약 7초다. 회귀가 그걸 매번 돌 이유는 없다. 나누면 된다:
    (a) **순수 계산**으로 실측 열이 5통계를 재현하는지 본다 (빠르고 정확하다)
    (b) **실제 타이머**로 '계획한 열이 그대로 나오는지'를 짧은 시험용 계획으로 잠근다
  (a)+(b) 의 합이 "실측 최대 간격이 실제로 발행된다"를 준다 — 배선은 계획과 무관하게
  성립하고, 실측 계획은 그 최대값을 담고 있기 때문이다.

[실행]
  cd ~/ros2_ws && python3 -m pytest src/tunnel_bringup/test/test_gate_fakes_periods.py -q
  (colcon test --packages-select tunnel_bringup 로도 돈다)
"""

import ast
import builtins
import importlib.util
import os
import re
import statistics
import symtable
import textwrap
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

import yaml


_HERE = os.path.dirname(os.path.abspath(__file__))
_FAKES_PATH = os.path.join(_HERE, 'gate_fakes.py')
_PKG_ROOT = os.path.dirname(_HERE)

_BLOCK_START = '--- 주기 정본 시작 ---'
_BLOCK_END = '--- 주기 정본 끝 ---'

# ★ '착수 전 전수 대조표'의 미확보 열을 여기 박제한다. 실측을 받아 채우는 날에는
#   표와 이 집합을 **같이** 고쳐야 한다 — 한쪽만 고치면 그게 다음 회차의 결함이다.
UNMEASURED_KEYS = frozenset({'scan', 'odom'})

# 실제 타이머 회귀용 **시험 계획** (실측값이 아니다 — 배선만 잠근다).
#   · 네 값이 전부 다르고 최소 간격차가 20ms 라, 관측 지터(±1ms 관측)로는 서로 섞이지 않는다.
#   · 회전대칭이 없어서 '한 칸 어긋남'이 다른 열로 보인다 (그래서 부정 회귀가 산다).
PROBE_PERIODS_US = (30_000, 90_000, 50_000, 140_000)
PROBE_STEPS = 10          # 2.5 바퀴 — 되감기(wraparound)까지 덮는다
MUTATION_STEPS = 4        # 순서 부정 회귀는 첫 바퀴에서 이미 갈라진다
PROBE_TOL_US = 8_000      # **순서 분류 전용** 허용 (값 간격 20ms 의 절반보다 작다)
PROBE_DOMAIN_ID = '89'    # 다른 세션의 DDS 와 섞이지 않게 격리

# ★ 08-01 검토 §22 P2-② — 분류 허용치 하나가 두 가지 일을 겸하고 있었다.
#   최근접 분류는 '관측이 어느 계획값인가'(순서)만 답한다. 그런데 값 간격이 20ms 라
#   같은 열 안에서 +7ms 씩 **한결같이 늦어도** 분류는 그대로 맞는다 — 실제로 생산
#   `period_ns()` 에 +7ms 를 주입했더니 순서 검사가 3/3 통과했다(거짓 PASS).
#   그래서 **시간 정확도**를 별도 계약으로 뽑는다: 부호 있는 잔차의 **중앙값**.
#   중앙값을 쓰는 이유 = 스케줄러가 한 번 튀어도 안 흔들리고 지속적 편향에만 반응한다.
#
#   임계값 3.0ms 의 근거는 임의 숫자가 아니라 **안전량**이다: 실측 IMU 최대 간격 30ms 와
#   EKF 주기 33.33ms 의 여유가 3.33ms 다(아래 §4 가 감시하는 그 관계). 즉 이 임계를
#   넘는 편향은 '실측 최대를 EKF 한 주기 밖으로 밀 수 있는 크기'다.
#
#   [08-01 측정] 무변이 편향을 8회씩 쟀다 — 무부하 |편향| 최대 **0.137ms**,
#   16 코어를 전부 점유한 부하 아래에서도 **0.165ms**. 단일 표본이 2.3ms 튄 회차에도
#   중앙값은 0.137ms 에 머물렀다(중앙값을 쓴 이유가 이것이다). 즉 임계 3.0ms 는
#   **잡음이 정한 값이 아니라 안전 의미가 정한 값**이고, 잡음 대비 18 배 여유가 있다.
#   ⚠ 그래서 남는 구멍: 0.2~3.0ms 사이의 체계적 편향은 **의도적으로 통과시킨다.**
#     더 조일 수도 있지만(잡음만 보면 1ms 도 가능) 이 회귀는 Jetson(aarch64)에서도
#     돌고 거기 타이머 잡음은 **아직 못 쟀다.** 간헐 실패하는 게이트는 결국 꺼지므로
#     구멍보다 나쁘다. 재개방 조건 = Jetson 에서 같은 측정을 하면 임계를 다시 정한다.
BIAS_TOL_US = 3_000
BIAS_SHIFT_US = 7_000     # 부정 회귀가 주입하는 편향 크기 (§22 검토자의 반례와 같은 값)

# ★ 08-01 검토 §22 P2-① — 정적 분석이 원리적으로 못 보는 자리.
#   아래가 정본 블록 안에 나타나면 폐포 검사는 **참을 증명할 수 없으므로 멈춘다**(fail-closed).
#   `global` 은 함수 안에서 모듈 이름을 새로 묶어 아래 바인딩 지도를 무력화하고,
#   `eval`/`globals()` 류는 이름을 실행 시각에 만든다. 조용히 통과시키는 것이 결함이다.
DYNAMIC_ESCAPES = frozenset({'eval', 'exec', 'globals', 'locals', 'vars', '__import__'})

# 폐포가 **반드시 닿아야 하는** 뿌리들 — 하나라도 안 닿으면 검사가 헛돈 것이다.
CLOSURE_ANCHORS = ('PeriodPlan', '_build_imu_periods', 'IMU_MAX_US',
                   'SCAN_PERIOD_US', 'ODOM_PERIOD_US', 'EKF_FREQUENCY_HZ')

# 스코프를 여는 문법 — 이 안쪽 이름은 모듈 전역 읽기가 아니다.
# ★ 여기서 스코프 '규칙'을 다시 구현하지 않는다. 구조 판별에만 쓰고, 이름이 어느
#   스코프에 묶이는가는 전부 `symtable`(= CPython 자신)에 묻는다. 아래 `_outside_names` 참고.
_SCOPE_NODES = (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda, ast.ClassDef,
                ast.ListComp, ast.SetComp, ast.DictComp, ast.GeneratorExp)


def _load_fakes(path=_FAKES_PATH):
    """`gate_fakes.py` 를 경로로 직접 적재한다 (설치 패키지가 아니라 test/ 안의 파일)."""
    spec = importlib.util.spec_from_file_location('gate_fakes_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _source(path=_FAKES_PATH):
    with open(path, encoding='utf-8') as fp:
        return fp.read()


def _fake_sensors_class(src=None):
    """AST 에서 FakeSensors 클래스 노드만 꺼낸다."""
    for node in ast.parse(src if src is not None else _source()).body:
        if isinstance(node, ast.ClassDef) and node.name == 'FakeSensors':
            return node
    raise AssertionError('FakeSensors 클래스를 못 찾음 — 이름이 바뀌었다면 이 회귀도 같이 고칠 것')


def _calls_named(node, attr):
    """주어진 노드 하위의 `something.attr(...)` 호출을 전부 모은다."""
    return [n for n in ast.walk(node)
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute) and n.func.attr == attr]


# ============================================================
# A. 실제 rclpy 타이머 관측 — 생산 배선을 '통과해서' 본다
#
# ★ 확정한 rcl 순서 (2026-08-01 실측, 추론 아님):
#   rcl 은 콜백을 부르기 **전에** `next_call_time += 현재 주기` 로 다음 만기를 확정한다.
#   그래서 k 번째 콜백이 심은 값은 k+1 번째 간격을 지배하고, 초기 주기 = seq[0] ·
#   k 번째 콜백이 seq[k] 를 심는 배선이면 **콜백 간 간격 열이 정확히 seq** 가 된다.
#   이 파일은 그 순서를 흉내 내지 않는다 — 진짜로 돌려서 관측한다.
# ============================================================
def _observe_real_intervals(gf, periods_us=PROBE_PERIODS_US, steps=PROBE_STEPS):
    """
    생산 `FakeSensors` 를 실제 rclpy executor 로 돌려 **콜백 간 간격(us)** 을 관측한다.

    관측점은 `_imu` 진입 시각이다. 즉 `_imu()` → `_advance_imu_period()` →
    `timer_period_ns` setter 사슬을 **전부 통과한** 결과만 본다.
    (DDS 를 관측 경로에 넣지 않으므로 전송 지연이 섞이지 않는다 — 스케줄만 본다.)
    """
    plan = gf.PeriodPlan('/imu/data', periods_us, basis='회귀 전용 시험 계획 (실측 아님)')

    class _Probe(gf.FakeSensors):
        """생산 노드에 **관측점만** 덧댄다 — 주기 로직은 super() 가 그대로 돈다."""

        def __init__(self, *args, **kwargs):
            self.marks = []
            super().__init__(*args, **kwargs)

        def _imu(self):
            self.marks.append(time.monotonic())
            super()._imu()

    budget = 3.0 * sum(periods_us[i % len(periods_us)]
                       for i in range(steps + 1)) / 1e6 + 2.0
    saved_domain = os.environ.get('ROS_DOMAIN_ID')
    saved_plan = gf.SENSOR_PERIODS['imu']
    os.environ['ROS_DOMAIN_ID'] = PROBE_DOMAIN_ID
    gf.SENSOR_PERIODS['imu'] = plan
    rclpy.init()
    try:
        node = _Probe(messages=0, with_filtered=False)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        deadline = time.monotonic() + budget
        while len(node.marks) <= steps and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        marks = list(node.marks)
        executor.remove_node(node)
        node.destroy_node()
    finally:
        gf.SENSOR_PERIODS['imu'] = saved_plan
        rclpy.shutdown()
        if saved_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = saved_domain

    assert len(marks) > steps, (
        f'{budget:.1f}s 안에 콜백이 {steps + 1} 번 돌지 않았다 (관측 {len(marks)} 회). '
        '타이머가 아예 안 돌거나 주기가 계획보다 훨씬 길다')
    return [round((b - a) * 1e6) for a, b in zip(marks, marks[1:])]


def _nearest_planned(observed_us, periods_us):
    """관측 간격을 계획값 중 가장 가까운 것으로 분류한다 (허용 밖이면 None)."""
    best = min(periods_us, key=lambda p: abs(p - observed_us))
    return best if abs(best - observed_us) <= PROBE_TOL_US else None


def _real_timer_report(gf, steps=PROBE_STEPS):
    """
    실제 타이머 관측을 **두 계약으로 나눠** 돌려준다 (§22 P2-② 보완).

      order_ok — 관측 열이 계획 열과 **같은 순서**인가 (최근접 분류)
      bias_us  — 계획 대비 부호 있는 잔차의 **중앙값** = 체계적으로 늦는가/빠른가

    두 계약을 한 숫자로 묶으면 안 되는 이유: 분류 허용치는 값 간격(20ms)에서 나오므로
    같은 열 안의 +7ms 편향을 전부 승인한다. 순서는 맞는데 시간이 틀린 상태가 그것이다.
    """
    observed = _observe_real_intervals(gf, steps=steps)
    expected = [PROBE_PERIODS_US[i % len(PROBE_PERIODS_US)] for i in range(len(observed))]
    classified = [_nearest_planned(v, PROBE_PERIODS_US) for v in observed]
    bias_us = statistics.median(o - e for o, e in zip(observed, expected))
    return classified == expected, bias_us, observed, expected


def _real_timer_follows_plan(gf, steps=PROBE_STEPS):
    """순서 계약만 — 부정 회귀(커서·setter 4종)가 이 함수의 False 를 요구한다."""
    order_ok, _, observed, expected = _real_timer_report(gf, steps=steps)
    return order_ok, observed, expected


def _shifted_period_source(shift_us):
    """생산 `period_ns()` 가 모든 후속 주기를 shift_us 만큼 밀도록 만드는 변이 앵커."""
    anchor = 'return self.periods_us[index % len(self.periods_us)] * 1000'
    return anchor, f'{anchor} + ({shift_us * 1000})'


# ============================================================
# B. 정본 블록 폐포 — '주기 값을 만드는 정의'가 전부 블록 안인가
#    (AGENTS.md §3-10 ★ 커버리지 폐포 ② — 열거를 검사기 안으로)
#
# ★ 단위어(`Hz`·`ms`·`us`)를 더 열거하지 않는다. 표기가 아니라 **정의의 위치**를 본다:
#   `SENSOR_PERIODS` 에서 출발해 참조하는 이름을 전이적으로 따라가고, 그렇게 닿은
#   모듈 수준 정의의 줄 번호가 전부 두 표식 사이인지 검사한다. import 로 끌어온
#   이름도 블록 밖이므로 걸린다.
# ============================================================
def _block_bounds(src):
    """정본 블록 표식의 줄 번호 (1-기반, 각각 정확히 하나여야 한다)."""
    lines = src.splitlines()
    starts = [i + 1 for i, ln in enumerate(lines) if _BLOCK_START in ln]
    ends = [i + 1 for i, ln in enumerate(lines) if _BLOCK_END in ln]
    assert len(starts) == 1 and len(ends) == 1, '정본 블록 표식이 하나씩 있어야 한다'
    assert starts[0] < ends[0], '정본 블록 표식 순서가 뒤집혔다'
    return starts[0], ends[0]


def _module_statements(tree):
    """
    모듈 수준에서 **실행되는** 문장 전부 — `if`/`try`/`for`/`with` 안까지 들어간다.

    ★ 08-01 검토 §22 보완(자체 발견). 예전에는 `tree.body` 한 겹만 봐서, 모듈 수준
      `try:` 안의 정의가 지도에 안 잡혔다. 같은 이름이 위에도 있으면 **엉뚱한 노드의
      줄 번호**를 검사하게 된다. 함수·클래스 **안쪽**은 모듈 수준이 아니므로 안 들어간다.
    """
    out = []
    for node in tree.body:
        out.append(node)
        if not isinstance(node, _SCOPE_NODES):
            for sub in ast.walk(node):
                if sub is not node and isinstance(sub, ast.stmt):
                    out.append(sub)
    return out


def _module_bindings(tree):
    """모듈 수준에서 이름을 묶는 노드 전부 (대입·함수·클래스·import)."""
    out = {}
    for node in _module_statements(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                for sub in ast.walk(target):
                    if isinstance(sub, ast.Name):
                        out.setdefault(sub.id, []).append(node)
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            out.setdefault(node.target.id, []).append(node)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            out.setdefault(node.name, []).append(node)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            for alias in node.names:
                out.setdefault((alias.asname or alias.name).split('.')[0], []).append(node)
    return out


def _outside_names(node):
    """
    이 정의가 **모듈 전역에서 읽는** 이름. 스코프 판단은 전부 CPython 에 맡긴다.

    ★ 08-01 검토 §22 P2-① 의 근인과 그 처방.
      초판은 `ast.walk()` 로 모든 `Store` 를 평평하게 걷어 "지역변수"로 쳤다. 그런데
      Python 3 에서 comprehension·lambda·중첩 함수·class 본문은 **각각 다른 스코프**다.
      그래서 `[X for X in ()]` 뒤의 `return X` 가 블록 밖 전역을 읽는데도 검사기는
      지역으로 오인해 통과시켰다 (검토자가 실행으로 반증: 실제로 24020 을 읽었다).

      고치는 방법으로 '스코프 규칙을 더 정확히 다시 구현'하는 길은 택하지 않는다.
      그건 §21 에서 불승인된 `_simulate_rcl_timer()` 와 **같은 실수**다 — 생산/언어의
      의미론을 테스트 안에 재구현하면, 검사기는 자기가 푼 답을 채점하게 된다.
      대신 표준 라이브러리 `symtable`(= 인터프리터가 스코프를 컴파일할 때 쓰는 그 구현)
      에 **물어본다**. 정의를 probe 함수로 감싸 넘기면, 모듈 전역 읽기만
      `is_global()` 로 표시돼 나온다 — 근사가 아니라 CPython 의 답이다.
    """
    if isinstance(node, (ast.Assign, ast.AnnAssign)):
        # 대입은 **오른쪽만** 넘긴다. 왼쪽(대상)을 같이 넣으면 probe 안에서 지역이 돼
        # `X = X + 1` 류의 자기참조 읽기가 사라진다.
        if node.value is None:
            return set()
        segment = ast.unparse(node.value)
    else:
        segment = ast.unparse(node)

    probe = symtable.symtable('def __probe__():\n' + textwrap.indent(segment, '    '),
                              'closure_probe.py', 'exec')

    def collect(table):
        names = {sym.get_name() for sym in table.get_symbols()
                 if sym.is_referenced() and sym.is_global()}
        for child in table.get_children():
            names |= collect(child)
        return names

    return collect(probe)


def _dynamic_escapes(node):
    """정적 분석이 못 보는 문법이 이 정의 안에 있는가 (있으면 fail-closed)."""
    found = set()
    for sub in ast.walk(node):
        if isinstance(sub, (ast.Global, ast.Nonlocal)):
            found.add('global' if isinstance(sub, ast.Global) else 'nonlocal')
        elif isinstance(sub, ast.Name) and sub.id in DYNAMIC_ESCAPES:
            found.add(sub.id + '()')
        elif isinstance(sub, ast.ImportFrom) and any(a.name == '*' for a in sub.names):
            found.add('import *')
    return found


def _period_closure(tree):
    """`SENSOR_PERIODS` 가 (전이적으로) 의존하는 모듈 수준 정의 전부 + 정체불명 이름."""
    bindings = _module_bindings(tree)
    roots = bindings.get('SENSOR_PERIODS')
    assert roots, 'SENSOR_PERIODS 를 못 찾음 — 이름이 바뀌었다면 이 회귀도 같이 고칠 것'

    # ★ 08-01 검토 §22 P2-① — 뿌리도 **위치 검사 대상**이다.
    #   예전에는 뿌리를 큐에만 넣고 결과에는 안 넣어서, `SENSOR_PERIODS = {...}` 를
    #   통째로 블록 밖으로 옮기면 "밖에 0건"으로 거짓 통과했다 (검토자 반례).
    reached = [('SENSOR_PERIODS', node) for node in roots]
    queued = {id(node) for node in roots}
    queue = list(roots)
    unknown, escapes = set(), set()
    while queue:
        node = queue.pop()
        escapes |= _dynamic_escapes(node)
        for name in _outside_names(node):
            if name in bindings:
                for definition in bindings[name]:
                    if id(definition) not in queued:
                        queued.add(id(definition))
                        reached.append((name, definition))
                        queue.append(definition)
            elif not hasattr(builtins, name):
                unknown.add(name)
    return reached, unknown, escapes


def _assert_period_definitions_closed(src):
    """주기 값을 만드는 정의가 전부 정본 블록 안인지 — 아니면 AssertionError."""
    start, end = _block_bounds(src)
    reached, unknown, escapes = _period_closure(ast.parse(src))

    assert not escapes, (
        '주기 계산이 정적 분석으로 따라갈 수 없는 문법에 기댄다: ' + ', '.join(sorted(escapes))
        + '\n  이 검사는 여기서 **멈춘다**(fail-closed). 조용히 통과시키는 것이 결함이다.')

    assert not unknown, (
        '주기 계산이 정체불명 이름에 기댄다 (빌트인도 모듈 정의도 아님): '
        + ', '.join(sorted(unknown)))

    outside = [f'{name} (line {node.lineno})' for name, node in reached
               if not (start < node.lineno and node.end_lineno < end)]
    assert not outside, (
        '주기 값을 만드는 정의가 정본 블록 **밖**에 있다 (언젠가 갈라진다):\n  '
        + '\n  '.join(sorted(outside)))

    # ★ 검사가 헛돌지 않는지 — 폐포가 실제로 네 센서의 뿌리까지 닿았는가.
    names = {name for name, _ in reached}
    for required in CLOSURE_ANCHORS:
        assert required in names, f'폐포가 {required} 에 닿지 않았다 — 이 검사가 헛돌고 있다'


# ============================================================
# C. 변이 주입 — 반례를 **영구 회귀**로 박제한다
#    (한 번 손으로 해보고 마는 반례는 다음 회차에 또 뚫린다)
# ============================================================
def _source_with(replacements, src=None):
    """
    원본에 문자열 변이를 넣은 **소스 문자열**을 준다 (앵커가 1회가 아니면 크게 실패).

    파일로 안 떨구는 이유: AST 검사는 소스만 있으면 되고, 썼다가 다시 읽는 왕복은
    변이가 실제로 적용됐는지를 흐린다. 문법 유효성은 `compile()` 로 여기서 확인한다.
    """
    out = _source() if src is None else src
    for old, new in replacements:
        assert out.count(old) == 1, f'변이 앵커가 정확히 1회가 아니다: {old!r} — 회귀를 같이 고칠 것'
        out = out.replace(old, new)
    compile(out, 'mutated_gate_fakes.py', 'exec')   # 파싱만 되는 변이는 반례가 못 된다
    return out


def _closure_rejects(src):
    """폐포 검사가 이 소스를 거부하는가 — 거부하면 그 사유(문자열)를, 통과하면 None."""
    try:
        _assert_period_definitions_closed(src)
    except AssertionError as exc:
        return str(exc)
    return None


def _mutate(tmp_path, name, replacements=(), inserts=()):
    """`gate_fakes.py` 사본에 변이를 넣고 경로를 준다. 앵커가 없으면 **크게 실패**한다."""
    src = _source()
    for old, new in replacements:
        assert src.count(old) == 1, f'변이 앵커가 정확히 1회가 아니다: {old!r} — 회귀를 같이 고칠 것'
        src = src.replace(old, new)
    for pattern, repl, extra in inserts:
        matches = re.findall(pattern, src, re.M)
        assert len(matches) == 1, f'변이 앵커가 정확히 1회가 아니다: {pattern!r}'
        value = matches[0]
        src = re.sub(pattern, repl, src, count=1, flags=re.M)
        # 블록 **시작 표식 앞**에 심는다 → 블록 밖이면서 import 는 되는 자리.
        src = src.replace('# ' + _BLOCK_START,
                          extra.format(value=value) + '# ' + _BLOCK_START, 1)
    path = os.path.join(str(tmp_path), f'gate_fakes_{name}.py')
    with open(path, 'w', encoding='utf-8') as fp:
        fp.write(src)
    return path


# ============================================================
# 1. 실측 재현 — 하네스가 만드는 입력이 실물과 같은가 (순수 계산)
# ============================================================
def test_imu_sequence_reproduces_measured_statistics():
    """IMU 주기 열이 구동부 2차 회신 §11 의 5통계를 그대로 재현한다."""
    gf = _load_fakes()
    seq = gf.SENSOR_PERIODS['imu'].periods_us

    assert len(seq) == gf.IMU_WINDOW, '창 크기가 실측 표본 수와 다르다'
    assert round(statistics.mean(seq) / 1000, 2) == round(gf.IMU_MEAN_US / 1000, 2)
    assert min(seq) == gf.IMU_MIN_US, '관측 최소가 시퀀스에 없다'
    assert max(seq) == gf.IMU_MAX_US, '관측 최대가 시퀀스에 없다'
    # σ 는 모집단·표본 어느 정의로 계산해도 실측값으로 반올림돼야 한다
    # (구동부가 어느 쪽을 썼는지 안 알려줬으므로 양쪽 다 만족해야 '재현'이다).
    target = round(gf.IMU_SIGMA_US / 1000, 2)
    assert round(statistics.pstdev(seq) / 1000, 2) == target
    assert round(statistics.stdev(seq) / 1000, 2) == target


def test_imu_sequence_is_not_uniform():
    """
    ★ 부정 회귀 — 누가 등간격으로 되돌리면 여기서 깨진다.

    이 단언이 없으면 '평균만 맞는 등간격'으로 조용히 되돌아가도 아무도 모른다.
    등간격은 고치기 전과 **같은 입력**이다.
    """
    gf = _load_fakes()
    seq = gf.SENSOR_PERIODS['imu'].periods_us
    assert min(seq) != max(seq), '등간격으로 되돌아갔다 — 실측 분포가 아니다'
    assert gf.SENSOR_PERIODS['imu'].varies is True
    assert statistics.pstdev(seq) > 0


def test_imu_extremes_appear_exactly_once_per_window():
    """
    실측 최대·최소가 창마다 **정확히 한 번씩** 계획에 들어 있다.

    ★ 이건 '계획에 있다'까지만 말한다. 그 계획이 실제로 그대로 발행되는지는
      아래 §2 의 실제 타이머 회귀가 별도로 잠근다 (둘을 합쳐야 주장이 선다).
    창당 1회는 임의 선택이 아니라 5통계에서 나오는 산술적 귀결이다
    (`gate_fakes.py` 의 유도 — 편차제곱합의 80%를 극단 2개가 쓴다).
    """
    gf = _load_fakes()
    seq = gf.SENSOR_PERIODS['imu'].periods_us
    assert seq.count(gf.IMU_MAX_US) == 1, '창 한 바퀴에 최대 간격이 정확히 1회가 아니다'
    assert seq.count(gf.IMU_MIN_US) == 1, '창 한 바퀴에 최소 간격이 정확히 1회가 아니다'


def test_imu_schedule_is_deterministic():
    """결정론 — 돌릴 때마다 다른 게이트는 판정기가 아니다."""
    first = _load_fakes().SENSOR_PERIODS['imu'].periods_us
    second = _load_fakes().SENSOR_PERIODS['imu'].periods_us
    assert first == second, '적재할 때마다 시퀀스가 달라진다 (난수·시간 의존)'
    plan = _load_fakes().SENSOR_PERIODS['imu']
    assert plan.period_ns(3) == plan.period_ns(3 + len(first)), '되감기가 결정적이지 않다'


# ============================================================
# 2. 실제 rclpy 타이머 — 생산 배선이 계획한 순서를 정말 만드는가
#    ★ §21 P2-① 보완. 여기서 시뮬레이터를 쓰면 검사의 의미가 사라진다.
# ============================================================
def test_real_timer_emits_the_planned_sequence():
    """
    ★ 역회귀 — 실제 타이머·실제 노드로 관측한 간격 열이 계획 열과 같다.

    되감기(계획 길이 4, 관측 10)까지 덮으므로 '한 바퀴 끝에서 처음으로 돌아오는' 자리도
    함께 잠긴다. 관측은 분류(가장 가까운 계획값)로 판정한다 — ms 를 정확히 단언하면
    스케줄러 지터로 깨지는 거짓 실패가 되고, 그건 게이트가 아니라 소음이다.
    """
    ok, observed, expected = _real_timer_follows_plan(_load_fakes())
    assert ok, (f'실제 발행 순서가 계획과 다르다\n  관측(us): {observed}\n  계획(us): {expected}')


def test_real_timer_first_interval_comes_from_the_table():
    """첫 간격이 표의 첫 주기다 — 초기 주기를 다른 데서 가져오면 여기서 걸린다."""
    gf = _load_fakes()
    observed = _observe_real_intervals(gf, steps=2)
    assert _nearest_planned(observed[0], PROBE_PERIODS_US) == PROBE_PERIODS_US[0], \
        f'첫 간격이 계획의 첫 주기가 아니다: {observed[0]}us'


def test_negative_frozen_cursor_breaks_the_real_timer(tmp_path):
    """
    ★ 부정 회귀 — 생산 커서를 고정(`+= 1` → `+= 0`)하면 회귀가 반드시 깨진다.

    이 변이는 IMU 를 다시 **등간격**으로 만든다 = 예약 18 이전 상태.
    초판 회귀는 이걸 11/11 통과시켰다 (시뮬레이터가 대신 세어 줬기 때문).
    """
    path = _mutate(tmp_path, 'frozen',
                   replacements=(('self._imu_step += 1', 'self._imu_step += 0'),))
    ok, observed, _ = _real_timer_follows_plan(_load_fakes(path), steps=MUTATION_STEPS)
    assert not ok, f'커서를 고정했는데도 통과한다 — 이 회귀가 생산 코드를 안 보고 있다: {observed}'


def test_negative_skewed_cursor_breaks_the_real_timer(tmp_path):
    """★ 부정 회귀 — 커서를 한 칸 어긋나게(`+= 2`) 하면 회귀가 반드시 깨진다."""
    path = _mutate(tmp_path, 'skewed',
                   replacements=(('self._imu_step += 1', 'self._imu_step += 2'),))
    ok, observed, _ = _real_timer_follows_plan(_load_fakes(path), steps=MUTATION_STEPS)
    assert not ok, f'커서가 어긋났는데도 통과한다: {observed}'


def test_negative_missing_setter_breaks_the_real_timer(tmp_path):
    """★ 부정 회귀 — 타이머 setter 를 없애면(주기를 안 갈아 끼움) 회귀가 반드시 깨진다."""
    setter = ("        self.imu_timer.timer_period_ns = "
              "SENSOR_PERIODS['imu'].period_ns(self._imu_step)")
    path = _mutate(tmp_path, 'nosetter',
                   replacements=((setter, '        pass  # 변이: setter 제거'),))
    ok, observed, _ = _real_timer_follows_plan(_load_fakes(path), steps=MUTATION_STEPS)
    assert not ok, f'주기를 갈아 끼우지 않는데도 통과한다: {observed}'


def test_negative_setter_reading_other_plan_breaks_the_real_timer(tmp_path):
    """
    ★ 부정 회귀 — setter 가 **다른 계획**을 참조하면 회귀가 반드시 깨진다.

    setter 를 지우는 것(위)과 엉뚱한 표를 읽는 것은 다른 결함이다. 후자는 코드가
    '주기를 갈아 끼우는 것처럼' 보여서 눈으로는 더 잘 통과한다.
    """
    path = _mutate(tmp_path, 'otherplan',
                   replacements=(("SENSOR_PERIODS['imu'].period_ns(self._imu_step)",
                                  "SENSOR_PERIODS['scan'].period_ns(self._imu_step)"),))
    ok, observed, _ = _real_timer_follows_plan(_load_fakes(path), steps=MUTATION_STEPS)
    assert not ok, f'다른 센서의 계획을 읽는데도 통과한다: {observed}'


def test_real_timer_has_no_systematic_period_bias():
    """
    ★ 역회귀 — 관측 간격이 계획보다 **한결같이 늦거나 빠르지** 않다 (§22 P2-② 보완).

    위 순서 검사와 **다른 계약**이다. 순서는 '어느 계획값인가'만 보므로, 값 간격
    20ms 안에서 매번 +7ms 늦어도 통과한다. 여기서는 부호 있는 잔차의 중앙값을 본다.
    """
    _, bias_us, observed, expected = _real_timer_report(_load_fakes())
    assert abs(bias_us) <= BIAS_TOL_US, (
        f'주기가 계획 대비 체계적으로 {bias_us / 1000:+.2f}ms 치우쳤다 '
        f'(허용 ±{BIAS_TOL_US / 1000:.1f}ms)\n  관측(us): {observed}\n  계획(us): {expected}')


def test_negative_late_period_survives_order_but_fails_bias(tmp_path):
    """
    ★ 부정 회귀 — 생산 `period_ns()` 가 매번 +7ms 늦으면 **시간 계약**이 깨진다.

    §22 검토자의 직접 반례다. 이 변이는 순서 검사를 3/3 통과했다(거짓 PASS) —
    분류가 20ms 간격이라 +7ms 로는 다른 열로 안 넘어가기 때문이다.
    같은 편향이 실측 대표열에 생기면 최대 간격이 30 → 37ms 로 EKF 주기 33.33ms 를
    넘어, 정본이 감시하는 3.33ms 여유가 사라진다. 그래서 이건 안전 관계를 뒤집는다.

    ★ 여기서 순서 계약은 단언하지 않는다. 7ms 편향은 분류 허용치 8ms 의 **경계 근처**라
      지터에 따라 순서가 깨지기도 한다 — 그걸 단언하면 간헐 실패하는 게이트가 되고,
      간헐 실패하는 게이트는 결국 꺼지므로 구멍보다 나쁘다. 이 반례가 지는 계약은
      '편향을 잡는가' 하나다. ("순서만으로는 못 잡는다"는 §22 검토자가 3/3 로 재현했다.)
    """
    path = _mutate(tmp_path, 'late', replacements=(_shifted_period_source(BIAS_SHIFT_US),))
    _, bias_us, observed, _ = _real_timer_report(_load_fakes(path))
    assert abs(bias_us) > BIAS_TOL_US, (
        f'주기를 매번 +{BIAS_SHIFT_US / 1000:.0f}ms 늦췄는데 편향 계약이 통과한다 '
        f'(측정 {bias_us / 1000:+.2f}ms): {observed}')


def test_negative_early_period_fails_the_bias_contract(tmp_path):
    """★ 부정 회귀 — 같은 크기로 **빨라지는** 편향도 잡는다 (증가·감소 양쪽 잠금)."""
    path = _mutate(tmp_path, 'early', replacements=(_shifted_period_source(-BIAS_SHIFT_US),))
    _, bias_us, observed, _ = _real_timer_report(_load_fakes(path))
    assert abs(bias_us) > BIAS_TOL_US, (
        f'주기를 매번 -{BIAS_SHIFT_US / 1000:.0f}ms 당겼는데 편향 계약이 통과한다 '
        f'(측정 {bias_us / 1000:+.2f}ms): {observed}')


# ============================================================
# 3. 갈라질 자리 봉쇄 — 목록과 구현이 어긋날 곳을 기계가 없앤다
# ============================================================
def test_every_sensor_timer_period_comes_from_the_table():
    """가짜 센서의 모든 타이머 주기가 '주기 정본' 표에서 나온다 (리터럴 0건)."""
    timers = _calls_named(_fake_sensors_class(), 'create_timer')
    assert timers, 'create_timer 를 하나도 못 찾았다 — 이 검사가 헛돌고 있다'
    for call in timers:
        arg = ast.unparse(call.args[0])
        assert not isinstance(call.args[0], ast.Constant), \
            f'주기를 리터럴로 박았다: create_timer({arg}, …) — 표를 통해서만 준다'
        assert 'SENSOR_PERIODS[' in arg, f'표를 거치지 않은 주기: {arg}'


def test_every_published_topic_has_a_period_entry():
    """새 가짜 센서를 추가하면서 표를 안 고치는 경로를 막는다."""
    gf = _load_fakes()
    published = set()
    for call in _calls_named(_fake_sensors_class(), 'create_publisher'):
        topic = call.args[1]
        assert isinstance(topic, ast.Constant), '토픽 이름이 리터럴이 아니다 — 검사 불가'
        published.add(topic.value)
    assert published == {plan.topic for plan in gf.SENSOR_PERIODS.values()}


def test_period_definitions_are_closed_inside_the_canonical_block():
    """
    ★ 역회귀 — 주기 값을 만드는 **모든** 정의가 정본 블록 안에 닫혀 있다.

    §21 P2-② 보완. 표기(`Hz`·`ms`·`us`)를 열거하는 대신 의존 폐포를 따라가므로,
    단위어가 없는 숫자나 import 로 끌어온 값도 블록 밖이면 걸린다.
    """
    _assert_period_definitions_closed(_source())


def test_negative_constant_moved_outside_block_is_caught(tmp_path):
    """
    ★ 부정 회귀 — 정본 상수를 블록 밖으로 옮기면 반드시 걸린다.

    초판은 `<숫자>Hz` 만 봤기 때문에 `IMU_MEAN_US = <블록 밖 이름>` 으로 정본을
    통째로 내보내도 11/11 통과했다. 그게 이 묶음의 발단(같은 사실이 두 자리)이다.
    ★ 미끼 이름을 `LEAKED_VALUE` 로 둔다 — `Hz`·`us`·`ms` 는 물론 **단위어가 하나도
      없어야** 표기 의존이 남아 있지 않음을 보인다.
    """
    path = _mutate(tmp_path, 'outside_const',
                   inserts=((r'^IMU_MEAN_US\s*=\s*(\S+)',
                             'IMU_MEAN_US = LEAKED_VALUE',
                             'LEAKED_VALUE = {value}\n\n\n'),))
    try:
        _assert_period_definitions_closed(_source(path))
    except AssertionError:
        return
    raise AssertionError('정본 상수를 블록 밖으로 옮겼는데 검사가 통과했다')


def test_negative_period_from_outside_helper_is_caught(tmp_path):
    """★ 부정 회귀 — /scan·/odom 주기를 블록 밖 헬퍼가 공급해도 반드시 걸린다."""
    for name in ('SCAN_PERIOD_US', 'ODOM_PERIOD_US'):
        path = _mutate(tmp_path, f'outside_helper_{name}',
                       inserts=((rf'^{name}\s*=\s*(\S+)',
                                 f'{name} = _leaked_period()',
                                 'def _leaked_period():\n    return {value}\n\n\n'),))
        try:
            _assert_period_definitions_closed(_source(path))
        except AssertionError:
            continue
        raise AssertionError(f'{name} 를 블록 밖 헬퍼가 공급하는데 검사가 통과했다')


def test_negative_root_table_moved_outside_the_block_is_caught():
    """
    ★ 부정 회귀 — `SENSOR_PERIODS` **표 자체**를 블록 밖으로 옮기면 반드시 걸린다.

    §22 P2-① 검토자 반례. 예전 폐포는 뿌리를 '출발점'으로만 쓰고 위치는 안 봤다.
    그래서 표를 통째로 블록 끝 뒤로 옮겨도 "밖에 있는 정의 0건"으로 거짓 통과했다 —
    정본 블록이 텅 비어도 녹색이 나오는 상태였다.
    """
    src = _source()
    match = re.search(r'^SENSOR_PERIODS = \{.*?^\}\n', src, re.S | re.M)
    assert match, 'SENSOR_PERIODS 대입을 못 찾음 — 모양이 바뀌었다면 이 회귀도 같이 고칠 것'
    moved = _source_with(((f'# {_BLOCK_END}', f'# {_BLOCK_END}\n\n\n' + match.group(0)),),
                         src=src[:match.start()] + src[match.end():])

    reason = _closure_rejects(moved)
    assert reason, '주기 정본 표를 블록 밖으로 옮겼는데 폐포 검사가 통과했다'
    assert 'SENSOR_PERIODS' in reason, f'거부는 했는데 사유가 뿌리 이동이 아니다:\n{reason}'


# 블록 밖 값을 '지역변수처럼 보이게' 가리는 네 가지 스코프 — 전부 같은 결함 클래스다.
# ★ 초판의 평평한 `ast.walk()` 는 넷 다 지역으로 오인했다. 넷을 각각 박제하는 이유는
#   §19 의 불승인 근인이 "검토가 준 목록을 항목별로 대조하지 않은 것"이었기 때문이다.
SCOPE_SHADOWS = {
    'comprehension': '    _ = [LEAKED_VALUE for LEAKED_VALUE in ()]\n'
                     '    return LEAKED_VALUE\n',
    'nested-function': '    def _inner():\n'
                       '        LEAKED_VALUE = 0\n'
                       '        return LEAKED_VALUE\n'
                       '    _inner()\n'
                       '    return LEAKED_VALUE\n',
    'lambda-argument': '    _ = (lambda LEAKED_VALUE: LEAKED_VALUE)(0)\n'
                       '    return LEAKED_VALUE\n',
    'class-body': '    class _Holder:\n'
                  '        LEAKED_VALUE = 0\n'
                  '    _Holder()\n'
                  '    return LEAKED_VALUE\n',
}


def test_negative_outside_value_hidden_by_scope_shadowing_is_caught():
    """
    ★ 부정 회귀 — 블록 **밖** 값을 이름 가림으로 숨겨도 반드시 걸린다 (네 스코프 전부).

    §22 P2-① 검토자 반례와 그 이웃들. Python 3 에서 comprehension·중첩 함수·lambda·
    class 본문은 각각 다른 스코프라, 그 안의 대입은 바깥 이름을 **가리지 않는다.**
    검토자는 실행으로 반증했다 — `return LEAKED_VALUE` 가 실제로 블록 밖 24020 을 읽는데
    검사기는 "지역변수"로 보고 통과시켰다.
    이제 스코프 판단을 `symtable`(CPython 자신)에 맡기므로 넷 다 걸려야 한다.
    """
    for label, body in SCOPE_SHADOWS.items():
        mutated = _source_with((
            (f'# {_BLOCK_START}', f'LEAKED_VALUE = 24_020\n\n\n# {_BLOCK_START}'),
            ('def _build_imu_periods():',
             f'def _leaked_value():\n{body}\n\ndef _build_imu_periods():'),
            ('rest = [IMU_MEAN_US + d for d in jitter]',
             'rest = [IMU_MEAN_US + _leaked_value() + d for d in jitter]'),
        ))
        reason = _closure_rejects(mutated)
        assert reason, f'{label} 로 가린 블록 밖 값이 폐포를 통과했다'
        assert 'LEAKED_VALUE' in reason, \
            f'{label}: 거부는 했는데 사유가 블록 밖 값이 아니다 (검사가 다른 데서 깨졌다):\n{reason}'


def test_negative_dynamic_name_lookup_is_fail_closed():
    """
    ★ 부정 회귀 — 이름을 **실행 시각에** 만들면 검사기가 멈춘다 (조용히 통과하지 않는다).

    정적 분석으로 `globals()['X']` 를 따라가는 것은 원리적으로 불가능하다. 여기서
    할 수 있는 정직한 최선은 "못 본다"를 **크게 말하는 것**이다. 이 단언이 없으면
    폐포 검사는 이 경로에서 아무것도 못 보고 녹색을 준다 — 그게 예약 18 의 결함 모양이다.
    """
    mutated = _source_with((
        (f'# {_BLOCK_START}', f'LEAKED_VALUE = 24_020\n\n\n# {_BLOCK_START}'),
        ('def _build_imu_periods():',
         "def _leaked_value():\n    return globals()['LEAKED_VALUE']\n\n\n"
         'def _build_imu_periods():'),
        ('rest = [IMU_MEAN_US + d for d in jitter]',
         'rest = [IMU_MEAN_US + _leaked_value() + d for d in jitter]'),
    ))
    reason = _closure_rejects(mutated)
    assert reason, '동적 이름 조회를 폐포가 조용히 통과시켰다'
    assert 'globals()' in reason, f'거부 사유가 동적 조회가 아니다:\n{reason}'


def test_period_docstring_describes_the_real_timer_regression():
    """
    ★ 문서 대조 — 생산 설명이 **현행** 회귀를 가리킨다 (§22 P2-③ 보완).

    §21 에서 `_simulate_rcl_timer()` 를 지웠는데 독스트링은 "회귀가 시뮬레이터로 다시
    대조한다"를 현재형으로 남겨 뒀다. 런타임 영향은 없지만, 유지보수자가 그 문장을 읽고
    **테스트 안 시뮬레이터를 정상 설계로 되돌릴 수 있다** — 이번 근인이 바로 그것이다.
    초판 결함의 '역사' 설명은 반대로 반드시 남아 있어야 한다.
    """
    fn = next(node for node in ast.walk(_fake_sensors_class())
              if isinstance(node, ast.FunctionDef) and node.name == '_advance_imu_period')
    doc = ast.get_docstring(fn) or ''

    assert '시뮬레이터로 다시 대조' not in doc, \
        '삭제된 시뮬레이터를 아직 현행 검사로 설명한다 — 현재형 서술을 고칠 것'
    assert 'rclpy executor' in doc, \
        '현행 회귀(실제 FakeSensors + rclpy executor 관측)를 설명하지 않는다'
    assert '§21' in doc, \
        '초판 결함의 역사 설명이 사라졌다 — 근거가 없으면 같은 설계로 되돌아간다'


def test_harness_script_timers_stay_outside_the_table_by_convention():
    """
    ★ 역회귀 — 하네스 자신의 **대본 타이머**는 표 대상이 아니고, 그래도 폐포가 통과한다.

    규약(`gate_fakes.py` 정본 블록 머리말): 표에 넣는 것은 **가짜 '센서 입력'의 발행 주기**
    뿐이다. `FakeLifecycle` 의 소실/복구 tick 같은 시험 대본 시간은 실물이 만드는 입력이
    아니므로 리터럴로 남고, 폐포 검사의 대상도 아니다.
    이 단언이 없으면 규약이 문서에만 있고 기계가 안 지키는 상태가 된다 — 누가 대본
    타이머를 표에 밀어 넣거나, 반대로 센서 타이머를 `FakeSensors` 밖으로 빼도 조용하다.
    """
    src = _source()
    tree = ast.parse(src)          # ★ 한 번만 파싱한다 — 두 번 파싱하면 노드 id 가 달라져
    sensors = next(node for node in tree.body     # 교집합이 항상 비고, 검사가 조용히 헛돈다.
                   if isinstance(node, ast.ClassDef) and node.name == 'FakeSensors')
    in_sensors = {id(call) for call in _calls_named(sensors, 'create_timer')}
    outside = [call for call in _calls_named(tree, 'create_timer') if id(call) not in in_sensors]

    assert outside, 'FakeSensors 밖 타이머가 하나도 없다 — 이 규약 검사가 헛돌고 있다'
    for call in outside:
        assert isinstance(call.args[0], ast.Constant), (
            f'대본 타이머가 표를 참조한다: create_timer({ast.unparse(call.args[0])}, …) — '
            '규약상 표는 센서 입력 전용이다. 규약을 바꾸려면 이 검사도 같이 고칠 것')
    # 규약대로면 이 상태에서 폐포 검사는 통과해야 한다 (대본 시간이 오탐을 만들지 않는다).
    _assert_period_definitions_closed(src)


def test_no_hz_notation_outside_the_canonical_block():
    """
    주파수 **표기**(<숫자>Hz)가 정본 블록 밖 산문에 다시 적히지 않았다.

    ★ 범위를 정직하게: 이건 주석·독스트링이 갈라지는 것을 막는 검사일 뿐이고,
      '주기 값이 한 자리에만 있다'는 계약은 위의 폐포 검사가 진다.
      원래 결함의 절반이 독스트링의 '100Hz' 였으므로 이 산문 검사도 남긴다.
    """
    src = _source()
    start, end = _block_bounds(src)
    leaked = [f'{i + 1}: {ln.strip()}' for i, ln in enumerate(src.splitlines())
              if re.search(r'\d\s*Hz', ln) and not start <= i + 1 <= end]
    assert not leaked, '정본 블록 밖에 주파수 표기가 있다 (언젠가 갈라진다):\n' + '\n'.join(leaked)


def test_unmeasured_periods_are_declared_unmeasured():
    """실측을 못 받은 항목은 '미확보'로 공개돼 있다 — 추정으로 조용히 채우지 않는다."""
    gf = _load_fakes()
    actual = {key for key, plan in gf.SENSOR_PERIODS.items() if plan.unmeasured}
    assert actual == UNMEASURED_KEYS, (
        f'미확보 목록이 대조표와 다르다: 코드 {sorted(actual)} vs 표 {sorted(UNMEASURED_KEYS)}')
    for key in actual:
        assert gf.SENSOR_PERIODS[key].unmeasured.strip(), f'{key}: 미확보 사유가 비어 있다'
    for plan in gf.SENSOR_PERIODS.values():
        assert plan.basis.strip(), f'{plan.topic}: 주기의 출처가 비어 있다'


# ============================================================
# 4. 바깥 설정과의 결합 — EKF 쪽이 바뀌면 여기서 걸린다
# ============================================================
def test_filtered_period_matches_ekf_config():
    """/odometry/filtered 흉내 주기가 실제 `ekf_real.yaml` 의 frequency 와 같다."""
    gf = _load_fakes()
    with open(os.path.join(_PKG_ROOT, gf.EKF_CONFIG_REL), encoding='utf-8') as fp:
        params = yaml.safe_load(fp)['ekf_filter_node']['ros__parameters']
    assert params['frequency'] == gf.EKF_FREQUENCY_HZ, \
        'EKF 설정과 하네스가 갈라졌다 — 두 자리 중 하나만 고쳤다'
    assert gf.SENSOR_PERIODS['filtered'].periods_us[0] == round(1e6 / params['frequency'])


def test_imu_max_gap_versus_ekf_period_is_the_documented_relation():
    """
    IMU 최대 간격과 EKF 주기의 관계를 기계가 계산해 감시한다.

    ★ 정직하게: 실측 분포에서는 **EKF 가 굶지 않는다.** 최대 간격 30ms 는 EKF 한 주기
      33.33ms 보다 짧아서, 길이 33.33ms 짜리 창은 항상 IMU 를 한 장 이상 포함한다.
      즉 "IMU 최대 간격 >= EKF 주기"는 산술적으로 성립하지 않는다 (핸드오프 문장의 정정).
      의미 있는 것은 **여유가 얼마나 좁은가**다: 구판 등간격 20ms 는 13.33ms 여유였고,
      실측 분포는 3.33ms 뿐이다 — 같은 EKF 설정에서 여유가 1/4 로 줄었다.
    이 단언은 어느 쪽이든 바뀌면 깨진다. 깨졌다면 값을 맞추지 말고 **판단을 다시 하고**
    문서(`docs/REAL_ROBOT_VALUES.md §1`)를 같이 고칠 것.
    """
    gf = _load_fakes()
    ekf_period_ms = 1000.0 / gf.EKF_FREQUENCY_HZ
    imu_max_ms = gf.IMU_MAX_US / 1000.0

    assert imu_max_ms < ekf_period_ms, (
        f'관계가 뒤집혔다 — IMU 최대 간격 {imu_max_ms}ms >= EKF 주기 {ekf_period_ms:.2f}ms. '
        '이제 EKF 한 주기가 IMU 없이 지나갈 수 있다. 문서와 융합 설정을 다시 판단할 것')
    assert round(ekf_period_ms - imu_max_ms, 2) == 3.33, \
        '문서화된 여유(3.33ms)가 바뀌었다 — REAL_ROBOT_VALUES.md §1 을 같이 갱신할 것'
