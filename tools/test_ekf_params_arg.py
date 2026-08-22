#!/usr/bin/env python3
"""`ekf_params` 런치 인자 · `ekf_real_db.yaml` 사본 회귀 (2026-08-21).

🔴 **왜 사본이 있나** — 21:32 에 우측 합성 계측이 좌측의 약 **0.52배**였다
(`kL=1` 정규화 · 1채널 dead 는 **가설**이다). 그래서 `/odom` 의 `vyaw` 가 직진 중
**−0.058 rad/s** 를 지어낸다. 사본은 그 한 칸만 끈다.

🔴 **이 회귀가 지키는 것**

  ① 사본이 정본과 **한 칸만** 달라야 한다. 우회용 사본은 시간이 지나면 조용히
     벌어진다 — 벌어진 사본은 "지금 무엇이 다른가" 를 아무도 모르는 물건이 된다.
  ② `vx` 는 살아 있어야 한다. 🔴 vx 까지 빼면 EKF 에 전진 속도원이 하나도 안 남아
     위치가 아예 안 나간다. "이상하니 다 빼자" 가 제일 그럴듯한 오답이다.
  ③ IMU 가 yaw·vyaw 를 **둘 다** 줘야 한다. 안 그러면 vyaw 를 끈 자리가 빈다.
  ④ 런치 기본값은 **정본**이어야 한다. 기본이 사본으로 바뀌면 고장을 가린 설정이
     조용히 표준이 된다.
"""
import importlib.util
import os

import pytest

yaml = pytest.importorskip('yaml')

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, '..', 'src', 'tunnel_bringup', 'config')
LAUNCH = os.path.join(HERE, '..', 'src', 'tunnel_bringup', 'launch',
                      'real_bringup.launch.py')
VYAW = 11          # odom0_config / imu0_config 의 vyaw 칸
VX = 6             # vx 칸
YAW = 5            # 절대 yaw 칸


def params(name):
    with open(os.path.join(CFG, name), encoding='utf-8') as f:
        return yaml.safe_load(f)['ekf_filter_node']['ros__parameters']


def test_01_the_copy_differs_in_exactly_one_flag():
    """🔴 사본이 정본과 **한 칸만** 달라야 한다."""
    a, b = params('ekf_real.yaml'), params('ekf_real_db.yaml')
    assert set(a) == set(b), '키 집합이 달라졌다'
    diff = [k for k in a if a[k] != b[k]]
    assert diff == ['odom0_config'], f'예상 밖의 차이: {diff}'
    pairs = [i for i, (x, y) in enumerate(zip(a['odom0_config'],
                                              b['odom0_config'])) if x != y]
    assert pairs == [VYAW], f'odom0_config 에서 {pairs} 칸이 달라졌다 (vyaw 만이어야)'


def test_02_the_copy_turns_vyaw_off():
    assert params('ekf_real.yaml')['odom0_config'][VYAW] is True
    assert params('ekf_real_db.yaml')['odom0_config'][VYAW] is False


def test_03_vx_survives_in_both():
    """🔴 부정 회귀 — vx 를 끄면 EKF 에 전진 속도원이 없어져 위치가 안 나간다."""
    for f in ('ekf_real.yaml', 'ekf_real_db.yaml'):
        assert params(f)['odom0_config'][VX] is True, f


def test_04_the_imu_still_covers_rotation_in_the_copy():
    """vyaw 를 끈 자리를 IMU 가 메워야 한다 — 절대 yaw 와 각속도 둘 다."""
    p = params('ekf_real_db.yaml')
    assert p['imu0_config'][YAW] is True
    assert p['imu0_config'][VYAW] is True


def test_05_the_launch_default_is_the_canonical_file():
    """🔴 기본값이 사본으로 바뀌면 고장을 가린 설정이 조용히 표준이 된다."""
    pytest.importorskip('launch', reason='launch 미설치 환경(노트북 밖)')
    from launch.actions import DeclareLaunchArgument

    spec = importlib.util.spec_from_file_location('rb_ekf_mod', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    got = {a.name: a.default_value[0].text
           for a in mod.generate_launch_description().entities
           if isinstance(a, DeclareLaunchArgument) and a.default_value}
    assert got.get('ekf_params') == 'ekf_real.yaml', got.get('ekf_params')


def test_06_the_copy_is_reachable_by_that_argument():
    """사본 파일이 실제로 있어야 인자가 의미를 갖는다."""
    assert os.path.isfile(os.path.join(CFG, 'ekf_real_db.yaml'))
