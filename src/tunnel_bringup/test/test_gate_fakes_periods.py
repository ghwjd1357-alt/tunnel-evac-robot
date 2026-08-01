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

[이 파일이 지키는 것 — 값이 아니라 '갈라질 자리']
  1. IMU 주기 열이 **실측 5통계를 재현**하는가 (평균·min·max·σ·창)
  2. 등간격으로 되돌리면 **깨지는가** (부정 회귀 — 이게 없으면 조용히 되돌아간다)
  3. 실제 발행 간격 열이 그 시퀀스와 **같은가** (rcl 타이머 순서까지 포함해서)
  4. 주기 수치가 **정본 블록 한 곳에만** 있는가 (AST·정규식 전수 — 사람 기억이 아니라 기계)
  5. 실측을 못 받은 항목이 **'미확보'로 선언**돼 있는가 (추정으로 채우면 또 다른 거짓 입력)
  6. EKF 주기와의 관계가 **문서에 적힌 그대로**인가 (한쪽이 바뀌면 여기서 걸린다)

[실행]
  cd ~/ros2_ws && python3 -m pytest src/tunnel_bringup/test/test_gate_fakes_periods.py -q
  (colcon test --packages-select tunnel_bringup 로도 돈다)
"""

import ast
import importlib.util
import os
import re
import statistics

import yaml


_HERE = os.path.dirname(os.path.abspath(__file__))
_FAKES_PATH = os.path.join(_HERE, 'gate_fakes.py')
_PKG_ROOT = os.path.dirname(_HERE)

# ★ '착수 전 전수 대조표'의 미확보 열을 여기 박제한다. 실측을 받아 채우는 날에는
#   표와 이 집합을 **같이** 고쳐야 한다 — 한쪽만 고치면 그게 다음 회차의 결함이다.
UNMEASURED_KEYS = frozenset({'scan', 'odom'})


def _load_fakes():
    """`gate_fakes.py` 를 경로로 직접 적재한다 (설치 패키지가 아니라 test/ 안의 파일)."""
    spec = importlib.util.spec_from_file_location('gate_fakes_under_test', _FAKES_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _source():
    with open(_FAKES_PATH, encoding='utf-8') as fp:
        return fp.read()


def _fake_sensors_class():
    """AST 에서 FakeSensors 클래스 노드만 꺼낸다."""
    for node in ast.parse(_source()).body:
        if isinstance(node, ast.ClassDef) and node.name == 'FakeSensors':
            return node
    raise AssertionError('FakeSensors 클래스를 못 찾음 — 이름이 바뀌었다면 이 회귀도 같이 고칠 것')


def _calls_named(node, attr):
    """주어진 노드 하위의 `something.attr(...)` 호출을 전부 모은다."""
    return [n for n in ast.walk(node)
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute) and n.func.attr == attr]


def _simulate_rcl_timer(plan, steps):
    """
    rclpy(Humble) 타이머가 실제로 만드는 **발행 간격 열**을 그대로 흉내 낸다.

    확정한 순서 (2026-08-01 rclpy 실측 — 추론 아님):
      ① 만기가 되면 rcl 이 먼저 `next_call_time += 현재 주기` 로 다음 만기를 확정하고,
      ② **그 다음에** 파이썬 콜백이 돈다 (거기서 주기를 갈아 끼운다).
    그래서 k 번째 콜백이 심은 값은 k+1 번째 간격을 지배한다. 초기 주기 seq[0] +
    k 번째 콜백이 seq[k] 를 심는 배선이면 간격 열은 정확히 seq 가 된다.
    실측 확인: 심은 [50,10,90,20,70,30]ms → 관측 [49.9,9.7,90.3,19.7,70.2,30.0]ms.
    """
    period_ns = plan.periods_us[0] * 1000
    fired_at = period_ns
    fires = [fired_at]
    step = 0
    for _ in range(steps):
        next_fire = fired_at + period_ns    # ① 현재 주기로 다음 만기 확정
        step += 1                           # ② 콜백 = _advance_imu_period()
        period_ns = plan.period_ns(step)
        fired_at = next_fire
        fires.append(fired_at)
    return [(b - a) // 1000 for a, b in zip(fires, fires[1:])]


# ============================================================
# 1. 실측 재현 — 하네스가 만드는 입력이 실물과 같은가
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


def test_imu_max_gap_actually_occurs_in_every_window():
    """최대 간격이 '표에만 있는 수'가 아니라 **실제로 발행 간격으로 나온다**."""
    gf = _load_fakes()
    plan = gf.SENSOR_PERIODS['imu']
    emitted = _simulate_rcl_timer(plan, gf.IMU_WINDOW)
    assert gf.IMU_MAX_US in emitted, '창을 한 바퀴 돌아도 최대 간격이 한 번도 안 나온다'
    assert gf.IMU_MIN_US in emitted, '창을 한 바퀴 돌아도 최소 간격이 한 번도 안 나온다'
    # 창당 정확히 1회 — '5통계에서 나오는 산술적 귀결'(gate_fakes.py 의 유도)과 일치한다.
    assert emitted.count(gf.IMU_MAX_US) == 1
    assert emitted.count(gf.IMU_MIN_US) == 1


def test_emitted_intervals_equal_the_measured_sequence():
    """실제 발행 간격 열 == 시퀀스. rcl 의 '콜백 전에 만기 확정' 순서까지 포함해서."""
    gf = _load_fakes()
    plan = gf.SENSOR_PERIODS['imu']
    steps = gf.IMU_WINDOW + 7          # 창 경계를 넘겨 되감기까지 확인
    emitted = _simulate_rcl_timer(plan, steps)
    expected = [plan.periods_us[i % len(plan.periods_us)] for i in range(steps)]
    assert emitted == expected, 'rcl 타이머 순서가 바뀌었거나 배선이 한 칸 어긋났다'


def test_imu_schedule_is_deterministic():
    """결정론 — 돌릴 때마다 다른 게이트는 판정기가 아니다."""
    first = _load_fakes().SENSOR_PERIODS['imu'].periods_us
    second = _load_fakes().SENSOR_PERIODS['imu'].periods_us
    assert first == second, '적재할 때마다 시퀀스가 달라진다 (난수·시간 의존)'
    plan = _load_fakes().SENSOR_PERIODS['imu']
    assert plan.period_ns(3) == plan.period_ns(3 + len(first)), '되감기가 결정적이지 않다'


# ============================================================
# 2. 갈라질 자리 봉쇄 — 목록과 구현이 어긋날 곳을 기계가 없앤다
#    (AGENTS.md §3-10 ★ 커버리지 폐포 ② — 열거를 검사기 안으로)
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


def test_no_period_numbers_outside_the_canonical_block():
    """주기 수치(<숫자>Hz)가 정본 블록 **밖**에 다시 적히지 않았다."""
    lines = _source().splitlines()
    starts = [i for i, ln in enumerate(lines) if '--- 주기 정본 시작 ---' in ln]
    ends = [i for i, ln in enumerate(lines) if '--- 주기 정본 끝 ---' in ln]
    assert len(starts) == 1 and len(ends) == 1, '정본 블록 표식이 하나씩 있어야 한다'
    assert starts[0] < ends[0]

    leaked = [f'{i + 1}: {ln.strip()}' for i, ln in enumerate(lines)
              if re.search(r'\d\s*Hz', ln) and not starts[0] <= i <= ends[0]]
    assert not leaked, '정본 블록 밖에 주기 표기가 있다 (언젠가 갈라진다):\n' + '\n'.join(leaked)


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
# 3. 바깥 설정과의 결합 — EKF 쪽이 바뀌면 여기서 걸린다
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
