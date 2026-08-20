# -*- coding: utf-8 -*-
"""real_bringup 의 게이트 **사슬 구조** 검사 (08-21, Codex §82.2 · 동결 예외 9).

[무엇을 잡나]
  `camera:=true` 의 depth 게이트가 `when_ready(gate_sensors, [slam,…])` 와
  **형제**로 놓여 있었다. 형제는 동시에 진행한다 — core 센서가 먼저 준비되면
  slam→Nav2→미션이 depth 보다 **먼저** 뜨고, depth 가 끝내 없으면 90초 뒤에야
  전체가 내려간다. 종국 fail-closed 는 돌지만 그 90초가 반쪽 인지 구간이다.

[왜 텍스트가 아니라 구조로 보나]
  주석이나 문자열 검사로는 "형제인가 직렬인가" 를 구별할 수 없다. 그래서
  런치 서술을 실제로 만들고 `RegisterEventHandler` 의 target → on_exit 결과를
  따라가 **사슬을 그린다.** 실차 게이트 회귀(`tools/test_gate_regression.sh`)는
  ROS 런타임이 필요하지만 이 검사는 노드를 하나도 안 띄운다.

⚠ 이 검사가 증명하지 않는 것: depth 토픽이 실제로 흐르는지. 그건 실차 몫이다.
"""

import importlib.util
import os
import tempfile
import types

import pytest

LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'src', 'tunnel_bringup',
                      'launch', 'real_bringup.launch.py')

launch_pkg = pytest.importorskip('launch', reason='launch 미설치 환경(노트북 밖)')


def _load():
    spec = importlib.util.spec_from_file_location('real_bringup_mod', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _actions(camera):
    """launch_setup 을 실제 LaunchContext 로 돌려 액션 목록을 얻는다."""
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    mod = _load()
    ctx = LaunchContext()
    # ⚠ 필수 인자는 **선언을 실행하기 전에** 넣어야 한다 — DeclareLaunchArgument
    #   가 기본값 없는 인자를 그 자리에서 RuntimeError 로 막기 때문이다.
    # ⚠ check_misconfig 가 지도 파일 **존재**를 본다. 없으면 launch_setup 이
    #   노드를 하나도 안 만들고 Shutdown 만 돌려준다 — 그 설계가 옳고, 그래서
    #   구조 검사도 진짜 파일을 만들어 줘야 한다.
    d = tempfile.mkdtemp(prefix='camgate_')
    base = os.path.join(d, 'fakemap')
    for ext in ('.posegraph', '.data'):
        with open(base + ext, 'wb') as f:
            f.write(b'\x00')
    ctx.launch_configurations['map_file'] = base
    ctx.launch_configurations['camera'] = 'true' if camera else 'false'
    for a in mod.generate_launch_description().entities:
        if isinstance(a, DeclareLaunchArgument):
            a.visit(ctx)
    return mod.launch_setup(ctx), ctx


def _chain(actions, ctx):
    """{target 라벨: [다음에 뜨는 액션 라벨들]} 로 사슬을 펴낸다.

    조건이 걸린 GroupAction 은 조건을 평가해 **통과한 것만** 따라간다."""
    from launch.actions import GroupAction, RegisterEventHandler

    def label(a):
        n = getattr(a, '_Node__node_name', None)
        if isinstance(n, str) and n:
            return n
        return type(a).__name__

    out = {}

    def walk(items):
        for a in items:
            if isinstance(a, GroupAction):
                cond = getattr(a, 'condition', None)
                if cond is not None and not cond.evaluate(ctx):
                    continue
                walk(a._GroupAction__actions)
            elif isinstance(a, RegisterEventHandler):
                h = a.event_handler
                # launch 내부 필드명 — `_OnActionEventBase__*` (Humble).
                tgt = label(h._OnActionEventBase__action_matcher)
                fn = h._OnActionEventBase__on_event
                ev = types.SimpleNamespace(returncode=0, action=None)
                nxt = fn(ev, ctx)
                out.setdefault(tgt, []).extend(label(x) for x in (nxt or []))
    walk(actions)
    return out


def _gate_label(a):
    """게이트 노드 이름에서 라벨을 뽑는다 (`readiness_gate_<label>`)."""
    # ⚠ `a.node_name` 프로퍼티는 실행 전에 RuntimeError 를 던진다 — 우리는
    #   서술만 보므로 내부 필드를 직접 읽는다.
    n = getattr(a, '_Node__node_name', None)
    if not isinstance(n, str):
        return None
    return n[len('readiness_gate_'):] if n.startswith('readiness_gate_') else None


def test_camera_false_keeps_the_old_chain():
    """🟢 camera:=false 는 08-20 까지와 같은 순서여야 한다 (거동 불변)."""
    actions, ctx = _actions(camera=False)
    labels = {_gate_label(a) for a in actions}
    assert 'camera' not in labels, 'camera:=false 인데 카메라 게이트가 떴다'


def test_camera_true_puts_depth_before_slam():
    """🔴 재현본 — depth 게이트는 slam 의 **선행 조건**이어야 한다.

    구판 사슬:  sensors ─▶ slam ─▶ Nav2      (camera 게이트는 옆에서 따로)
    수정 사슬:  sensors ─▶ camera ─▶ slam ─▶ Nav2
    """
    actions, ctx = _actions(camera=True)
    chain = _chain(actions, ctx)
    assert chain['readiness_gate_sensors'] == ['readiness_gate_camera'], chain
    assert 'slam_toolbox' in chain['readiness_gate_camera'], chain
    # 센서 게이트가 slam 을 **직접** 띄우면 안 된다 (그게 병렬 구판이다)
    assert 'slam_toolbox' not in chain['readiness_gate_sensors'], chain


def test_camera_false_chain_is_byte_for_byte_the_old_order():
    """🟢 camera:=false 사슬은 08-20 까지와 같아야 한다."""
    actions, ctx = _actions(camera=False)
    chain = _chain(actions, ctx)
    assert chain['readiness_gate_sensors'] == ['slam_toolbox',
                                               'readiness_gate_localized'], chain
    assert 'readiness_gate_camera' not in chain, chain


def test_downstream_chain_is_unchanged_in_both_modes():
    """localized→Nav2→미션 은 카메라 여부와 무관해야 한다 (범위 최소 증명)."""
    ca, cc = _actions(camera=True)
    fa, fc = _actions(camera=False)
    on, off = _chain(ca, cc), _chain(fa, fc)
    for k in ('readiness_gate_localized', 'readiness_gate_nav2'):
        assert on[k] == off[k], (k, on[k], off[k])


def test_camera_gate_is_not_a_sibling_of_the_sensor_chain():
    """🔴 재현본 — 게이트가 최상위 형제로 떠 있으면 병렬이다.

    구판은 `camera_gate` GroupAction 이 `gate_sensors` 와 나란히 return 됐고,
    그 안에서 `gate_camera` 프로세스를 **바로** 띄웠다. 즉 센서 사슬과 동시에
    돌았다. 이제 depth 게이트는 `gate_sensors` 가 끝난 **뒤에** 시작해야 하므로
    최상위 액션 목록에는 없어야 한다."""
    actions, _ = _actions(camera=True)
    top = [_gate_label(a) for a in actions]
    assert 'sensors' in top, top
    assert 'camera' not in top, (
        '카메라 게이트가 최상위에 있다 = 센서 게이트와 병렬로 돈다 (§82.2)')
