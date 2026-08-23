#!/usr/bin/env python3
"""🔴 구동부 명령 상한의 **복사본들이 펌웨어와 같은지** 기계적으로 대조한다.

왜 있나 (2026-08-23 §91 2회차 P1-2)
-----------------------------------
`.ino` 의 `MAX_LINEAR_CMD` 는 08-22 에 0.12 → 0.20 으로 굽혔다. 그 값을 소비하는
파이썬 도구들은 **각자 자기 복사본**을 들고 있다. 08-22 에 한 자리를 옮기고 세 자리를
빠뜨렸고(§91 1회차 P1-3), 고친 뒤에도 *"복사본이 여러 벌"* 이라는 사실 자체는 그대로다
— 다음 상한 변경에서 같은 drift 가 반복된다.

🔴 그래서 **주석으로 "정본을 따라가라" 고 적는 대신 검사로 묶는다.**
`.ino` 를 고치고 도구를 안 고치면 여기서 FAIL 한다. 그 반대도 마찬가지다.

한계 (일부러 이렇게 한다)
-------------------------
`.ino` 를 컴파일하지 않고 **정규식으로 리터럴을 읽는다.** 상수 선언 형태가 바뀌면
이 검사가 먼저 깨지는데, 그건 결함이 아니라 **알림**이다 — 그때 여기를 같이 고친다.
⚠ 실제로 보드에 실린 값은 `.ino` 가 아니라 **마지막으로 구운 바이너리**다. 이 검사는
소스 간 정합만 본다. 보드 정합은 `firmware_precheck.sh` 와 `build=` 지문이 담당한다.
"""
import pathlib
import re

import pytest

_ROOT = pathlib.Path(__file__).resolve().parents[1]
_INO = _ROOT / 'firmware' / 'teensy_integrated_base_v1_4' / \
    'teensy_integrated_base_v1_4.ino'

# 도구 쪽 복사본 — (파일, 파이썬 상수명, `.ino` 상수명, 배수)
#   `odom_guard.VX_ABS_MAX` 는 상한의 **2배**로 정의된 가드다(그 파일 주석 참조).
_COPIES = [
    ('tools/apply_measurements.py', 'MAX_LINEAR_CMD', 'MAX_LINEAR_CMD', 1.0),
    ('tools/drive_linear_deadband.py', 'MAX_LINEAR_CMD', 'MAX_LINEAR_CMD', 1.0),
    ('tools/odom_guard.py', 'VX_ABS_MAX', 'MAX_LINEAR_CMD', 2.0),
]


def _ino_const(name):
    """`static const double NAME = 0.20;` 에서 값을 읽는다."""
    src = _INO.read_text(encoding='utf-8', errors='replace')
    m = re.search(rf'^\s*static\s+const\s+double\s+{name}\s*=\s*([-\d.eE+]+)\s*;',
                  src, re.MULTILINE)
    assert m, (f'🔴 {_INO.name} 에서 {name} 선언을 못 찾았다 — 선언 형태가 바뀌었으면 '
               f'이 검사를 같이 고쳐라')
    return float(m.group(1))


def _py_const(rel, name):
    src = (_ROOT / rel).read_text(encoding='utf-8')
    m = re.search(rf'^{name}\s*=\s*([-\d.eE+]+)\s*$', src, re.MULTILINE)
    assert m, f'🔴 {rel} 에서 {name} 선언(모듈 최상위 리터럴)을 못 찾았다'
    return float(m.group(1))


def test_ino_declares_the_three_caps():
    """전제 — `.ino` 가 세 상한을 리터럴로 들고 있다."""
    for n in ('MAX_LINEAR_CMD', 'MAX_ANGULAR_CMD', 'MAX_WHEEL_CMD'):
        v = _ino_const(n)
        assert v > 0, f'{n} 가 양수가 아니다: {v}'


@pytest.mark.parametrize('rel,py_name,ino_name,factor', _COPIES)
def test_python_copies_track_the_firmware(rel, py_name, ino_name, factor):
    """🎯 복사본 = `.ino` 값 × 배수. 어긋나면 여기서 멈춘다."""
    want = _ino_const(ino_name) * factor
    got = _py_const(rel, py_name)
    assert abs(got - want) < 1e-9, (
        f'🔴 상한 drift — {rel}:{py_name} = {got} 인데 '
        f'{_INO.name}:{ino_name}({_ino_const(ino_name)}) × {factor} = {want} 여야 한다. '
        f'펌웨어를 고쳤으면 이 복사본도 같이 옮겨라 (§91 P1-2).')


def test_every_known_copy_is_listed():
    """🔴 목록 자체를 계약으로 — 새 복사본이 생기면 여기 추가하지 않고는 못 지나간다.

    `tools/**` 에서 모듈 최상위에 `MAX_LINEAR_CMD`/`VX_ABS_MAX` 를 선언한 파일을
    전부 훑어, `_COPIES` 에 없는 것이 있으면 실패한다. 목록을 손으로 유지하는 대신
    **파일이 목록을 검증하게** 한다(§91 1회차가 '전수 열거를 눈으로 했다' 로 깨진 자리).
    """
    declared = set()
    for f in sorted((_ROOT / 'tools').glob('*.py')):
        src = f.read_text(encoding='utf-8', errors='replace')
        for name in ('MAX_LINEAR_CMD', 'VX_ABS_MAX'):
            if re.search(rf'^{name}\s*=\s*[-\d.eE+]+\s*$', src, re.MULTILINE):
                declared.add((f'tools/{f.name}', name))
    listed = {(rel, py) for rel, py, _i, _fa in _COPIES}
    missing = declared - listed
    assert not missing, (
        f'🔴 목록에 없는 상한 복사본: {sorted(missing)} — '
        f'`_COPIES` 에 추가하고 배수를 적어라')
