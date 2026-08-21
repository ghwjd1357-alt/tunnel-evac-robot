# -*- coding: utf-8 -*-
"""정지 실패 출구의 **개수를 계약으로 고정**한다 (08-21, Codex §85.3 패턴 수정).

[왜 개수를 세나]
  §84.2 는 `safety_stop` 의도를 만들고 실패 출구를 **3개만** 바꿨다. 남은 2개
  (`_on_cancel_response` 의 응답 예외 · 빈 `goals_canceling`)는 여전히
  `_on_fault()` 를 직접 불렀고, §85.3 이 그것을 재현했다 —
  `faults=[1] · unconfirmed=[] · stop_pending=True`.

  손으로 "다 바꿨다" 고 세면 또 놓친다. 그래서 **기계가 센다**:
    · `self._stop_failed(` 호출 수 == `GoalManager.STOP_FAILURE_EXITS`
    · 정지 대상(`_is_stop_target`) 블록 안에 `_on_fault()` 직접 호출이 0

⚠ 이 검사가 증명하지 않는 것: 각 출구가 **옳은 사유 문자열**을 넘기는지.
  그건 `test_alarm_boundary.py` 의 전이 검사 몫이다.
"""

import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GM = os.path.join(ROOT, 'src', 'mission_manager', 'mission_manager',
                  'goal_manager.py')


def source():
    return open(GM, encoding='utf-8').read()


def test_declared_exit_count_matches_the_calls():
    """🔴 출구를 하나 더 만들고 상수를 안 고치면 여기서 죽는다."""
    src = source()
    calls = len(re.findall(r'self\._stop_failed\(', src))
    m = re.search(r'STOP_FAILURE_EXITS = (\d+)', src)
    assert m, 'STOP_FAILURE_EXITS 선언이 사라졌다'
    assert calls == int(m.group(1)), (
        f'_stop_failed 호출 {calls}개 vs 선언 {m.group(1)}개 — '
        f'출구를 늘리거나 줄였으면 상수도 같이 고칠 것')


def test_no_direct_fault_inside_stop_target_guards():
    """정지 대상 블록 안에서 `_on_fault()` 를 직접 부르면 의도 분리가 샌다."""
    src = source().split('\n')
    bad = []
    for i, ln in enumerate(src):
        if '_is_stop_target(' not in ln or 'def ' in ln:
            continue
        # 가드 아래 8줄 안에서 직접 호출을 찾는다
        for j in range(i + 1, min(i + 9, len(src))):
            if 'self._on_fault()' in src[j]:
                bad.append((i + 1, j + 1, src[j].strip()))
            if src[j].strip().startswith('def '):
                break
    assert not bad, f'정지 대상 블록 안 직접 FAULT 호출: {bad}'


def test_stop_failed_routes_by_intent():
    """의도 분기가 함수 안에 실제로 있는가 (구조 검사)."""
    src = source()
    m = re.search(r'def _stop_failed\(self, reason\):(.*?)\n    def ', src,
                  re.S)
    assert m, '_stop_failed 를 못 찾았다'
    body = m.group(1)
    assert "_stop_intent == 'safety_stop'" in body, body
    assert '_on_stop_unconfirmed' in body and '_on_fault' in body, body


def test_safety_stop_intent_is_armed_like_guide_stop():
    """`safety_stop` 이 guide_stop 과 같은 직렬화를 무장하는가."""
    src = source()
    assert "intent in ('guide_stop', 'safety_stop')" in src
    assert "if intent in ('guide_stop', 'safety_stop') else None" in src
