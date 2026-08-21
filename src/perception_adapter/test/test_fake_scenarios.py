#!/usr/bin/env python3
"""`fake_detections.py` 시나리오 목록 회귀 — 2026-08-22 신설.

🔴 **왜 필요한가** — `SCENARIOS` 는 그냥 문자열 튜플이고, `tick()` 의 분기와
**아무도 대조하지 않았다.** 이름을 목록에 넣고 분기를 빠뜨리면 조용히 통과했다가
**실제로 그 시나리오를 돌리는 순간** "알 수 없는 scenario" 로 끝난다. 그 순간은
대개 촬영장이나 합류 시험 자리다.

08-22 에 사람 경로 6종을 더하면서 그 구멍이 6배가 됐으므로 여기서 잠근다.
ROS 를 안 띄우고 **원문을 읽어** 대조한다 — 이 검사는 노드를 기동할 이유가 없다.
"""
import os
import re

import perception_adapter.fake_detections as fd

SRC = os.path.join(os.path.dirname(os.path.abspath(fd.__file__)),
                   'fake_detections.py')


def source():
    with open(SRC, encoding='utf-8') as f:
        return f.read()


def handled():
    """`tick()` 안에서 실제로 분기가 달린 시나리오 이름 집합."""
    return set(re.findall(r"s == '([a-z_]+)'", source()))


def test_fs1_every_listed_scenario_has_a_branch():
    """🔴 목록에 있는데 분기가 없으면, 그걸 고르는 순간 노드가 거부한다."""
    missing = sorted(set(fd.SCENARIOS) - handled())
    assert not missing, f'분기가 없는 시나리오: {missing}'


def test_fs2_every_branch_is_listed():
    """반대 방향 — 분기만 있고 목록에 없으면 도움말이 거짓말을 한다."""
    orphan = sorted(handled() - set(fd.SCENARIOS))
    assert not orphan, f'목록에 없는 분기: {orphan}'


def test_fs3_no_duplicate_names():
    assert len(fd.SCENARIOS) == len(set(fd.SCENARIOS)), '시나리오 이름이 중복이다'


def test_fs4_the_person_path_is_present():
    """🆕 08-22 — 사람 경로가 통째로 빠지면 `/person_status` 를 로봇 없이 못 굴린다."""
    need = {'person_ok', 'person_fallen', 'person_none',
            'person_unknown', 'person_flicker'}
    assert need <= set(fd.SCENARIOS), f'빠진 것: {sorted(need - set(fd.SCENARIOS))}'


def test_fs5_none_and_stale_stay_separate_names():
    """🔴 부정 회귀 — "봤는데 없다"(none)와 "못 봤다"(stale)는 다른 시나리오다.

    둘을 한 이름으로 합치면, 아무도 없는 자리에서 유도가 시작되거나(none 을
    stale 로 읽음) 센서가 죽은 채로 신고가 나간다(그 반대). 계약이 `§4.1` 에서
    *"빈 배열과 미발행을 섞지 않는다"* 로 못 박은 그 구분이다.
    """
    assert 'person_none' in fd.SCENARIOS
    assert 'stale' in fd.SCENARIOS
    assert 'person_none' != 'stale'


def test_fs6_class_names_stay_inside_the_contract():
    """🔴 계약 열거 5종 밖의 이름을 **의도치 않게** 쓰면 안 된다.

    ⚠ `bad_class`('human')는 **일부러** 계약을 어기는 시나리오다 — 어댑터의 거부
    경로를 겨눈다. 그래서 예외로 둔다. 그 외에 새 이름이 새어 들어오면 잡는다.
    """
    contract = {'person_fallen', 'person_ok', 'person_unknown', 'fire', 'smoke'}
    used = set(re.findall(r"mk\('([a-z_]+)'", source()))
    strays = sorted(used - contract - {'human'})
    assert not strays, f'계약 밖 class_name: {strays}'
