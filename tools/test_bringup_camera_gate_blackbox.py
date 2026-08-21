# -*- coding: utf-8 -*-
"""camera 게이트 사슬 **black-box** 검사 — 실제 LaunchService 로 돌린다.

(08-21, Codex §83.7)

[왜 한 층 더 두는가]
  `tools/test_bringup_camera_gate.py` 는 launch **서술**을 정적으로 따라간다.
  빠르고 유용하지만 세 가지를 못 본다:
    ① 게이트가 **nonzero 로 죽었을 때** 실제로 `Shutdown` 이 도는가
    ② 하류(slam·mission)가 **정말 0회** 뜨는가
    ③ Humble 내부 필드 이름(`_OnActionEventBase__*`)에 안 묶인 판정인가

  그래서 여기서는 production 노드를 하나도 안 쓰고, `make_gate`/`when_ready` 와
  같은 모양의 **가짜 게이트 프로세스 + sentinel 하류**를 실제 LaunchService 로
  실행해 사슬을 관측한다. 검사 대상은 `real_bringup` 이 쓰는 **합성 규칙**이다.

⚠ 이 검사가 증명하지 않는 것: depth 토픽이 실제로 흐르는지, readiness_gate 노드의
  판정 로직. 그 둘은 각각 실차와 `test_readiness_gate.py` 몫이다.
"""

import os
import sys

import pytest

pytest.importorskip('launch', reason='launch 미설치 환경')

from launch import LaunchDescription, LaunchService            # noqa: E402
from launch.actions import ExecuteProcess, GroupAction         # noqa: E402
from launch.conditions import IfCondition, UnlessCondition     # noqa: E402
from launch.substitutions import LaunchConfiguration           # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'tunnel_bringup'))
from tunnel_bringup.launch_util import when_ready              # noqa: E402


def proc(tmp, name, rc=0):
    """가짜 게이트/하류 — **표식 파일**을 남기고 지정한 rc 로 끝난다.

    ⚠ stdout 대신 파일을 쓰는 이유: launch 는 프로세스 출력을 자기 로깅으로
      돌리므로 `redirect_stdout` 으로는 안 잡힌다(첫 구현이 여기서 전부 FAIL 했다).
      파일은 캡처 방식과 무관하게 관측된다.
    """
    f = os.path.join(tmp, name)
    code = f'open({f!r}, "a").write("x"); raise SystemExit({rc})'
    return ExecuteProcess(cmd=[sys.executable, '-c', code], output='log')


def count(tmp, name):
    f = os.path.join(tmp, name)
    return len(open(f).read()) if os.path.exists(f) else 0


def build(tmp, camera, camera_rc=0, sensors_rc=0):
    """real_bringup 과 **같은 합성 규칙**으로 사슬을 만든다."""
    gate_sensors = proc(tmp, 'gate_sensors', sensors_rc)
    gate_camera = proc(tmp, 'gate_camera', camera_rc)
    slam = proc(tmp, 'slam')
    return LaunchDescription([
        gate_sensors,
        GroupAction(
            actions=[
                when_ready(gate_sensors, [gate_camera], '카메라 depth 스트림 확인'),
                when_ready(gate_camera, [slam], '위치추정(slam_toolbox)'),
            ],
            condition=IfCondition(LaunchConfiguration('camera')),
        ),
        GroupAction(
            actions=[when_ready(gate_sensors, [slam], '위치추정(slam_toolbox)')],
            condition=UnlessCondition(LaunchConfiguration('camera')),
        ),
    ])


def run(tmp, camera, camera_rc=0, sensors_rc=0):
    """LaunchService 로 실제 실행하고 종료코드를 돌려준다."""
    ls = LaunchService(noninteractive=True)
    ls.context.launch_configurations['camera'] = 'true' if camera else 'false'
    ls.include_launch_description(build(tmp, camera, camera_rc, sensors_rc))
    return ls.run(shutdown_when_idle=True)


def test_camera_true_runs_depth_gate_then_slam(tmp_path):
    """🟢 depth 성공 → slam 이 정확히 1회 뜬다."""
    t = str(tmp_path)
    rc = run(t, camera=True, camera_rc=0)
    assert count(t, 'gate_camera') == 1
    assert count(t, 'slam') == 1
    assert rc == 0


def test_camera_true_depth_failure_shuts_down_without_slam(tmp_path):
    """🔴 §83.7 의 핵심 — depth 실패면 slam 이 **0회**여야 한다.

    구조 검사는 returncode=0 만 가짜로 넣어 이 경로를 못 봤다."""
    t = str(tmp_path)
    rc = run(t, camera=True, camera_rc=1)
    assert count(t, 'gate_camera') == 1
    assert count(t, 'slam') == 0, 'depth 실패인데 slam 이 떴다'
    # ⚠ rc 를 fail-closed 신호로 쓰지 않는다 — `Shutdown` 은 런치를 **정상**
    #   종료시켜 rc=0 이다(실측). 증거는 "하류가 0회" 쪽이다.
    assert rc == 0, rc


def test_camera_false_skips_the_depth_gate_entirely(tmp_path):
    """🟢 camera:=false 는 08-20 까지와 같은 순서 — 게이트 자체가 안 뜬다."""
    t = str(tmp_path)
    rc = run(t, camera=False)
    assert count(t, 'gate_camera') == 0
    assert count(t, 'slam') == 1
    assert rc == 0


def test_sensor_gate_failure_stops_everything_in_both_modes(tmp_path):
    """센서 게이트가 죽으면 카메라 여부와 무관하게 하류가 0회여야 한다."""
    for cam in (True, False):
        t = str(tmp_path / f'sensors_{cam}')
        os.makedirs(t, exist_ok=True)
        rc = run(t, camera=cam, sensors_rc=1)
        assert count(t, 'slam') == 0, cam
        assert count(t, 'gate_camera') == 0, cam
        assert rc == 0, (cam, rc)      # ⚠ 위와 같은 이유 — rc 는 신호가 아니다
