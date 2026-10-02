"""bag 구간 자르기 회귀 — 래치 프라임이 빠지면 화면에 지도가 영영 안 온다 (2026-10-02)."""
import importlib.util, pathlib

_p = pathlib.Path(__file__).with_name('bag_slice.py')
_spec = importlib.util.spec_from_file_location('bag_slice', _p)
m = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(m)

T0 = 1_000_000_000_000
S = lambda sec: T0 + int(sec * 1e9)          # noqa: E731
MAP, TF, SCAN = 1, 2, 3                       # topic_id
# (id, topic_id, timestamp) — 시간순
ROWS = [
    (10, MAP,  S(0.0)),      # 래치 — 구간 한참 전
    (11, TF,   S(0.1)),
    (12, SCAN, S(1.0)),
    (13, MAP,  S(2.0)),      # 래치 갱신 — 이게 '구간 앞의 마지막'
    (14, SCAN, S(4.9)),      # 구간 직전
    (15, SCAN, S(5.0)),      # 구간 시작 경계 — 포함
    (16, SCAN, S(7.5)),
    (17, SCAN, S(10.0)),     # 구간 끝 경계 — 포함
    (18, SCAN, S(10.1)),     # 구간 밖
]
WINDOW = dict(windows=[(S(5.0), S(10.0))], prime_topic_ids={MAP, TF})


def test_window_keeps_boundaries_and_drops_outside():
    got = m.plan_messages(ROWS, **WINDOW)
    ids = [i for i, _ in got]
    assert 15 in ids and 17 in ids, '경계(start·end)는 포함한다'
    assert 14 not in ids and 18 not in ids, '구간 밖은 뺀다'
    assert 12 not in ids, '구간 앞 일반 토픽은 안 끌어온다'


def test_latched_topics_are_primed_with_the_last_one_before_start():
    got = dict(m.plan_messages(ROWS, **WINDOW))
    assert 13 in got and 10 not in got, '래치는 구간 앞의 **마지막** 1건만'
    assert 11 in got, '/tf_static 도 프라임한다'
    assert got[13] == S(5.0) and got[11] == S(5.0), '프라임은 구간 시작 시각으로 당긴다'


def test_primed_messages_come_first():
    got = m.plan_messages(ROWS, **WINDOW)
    first_two = {i for i, _ in got[:2]}
    assert first_two == {13, 11}, '지도·정적TF 가 맨 앞이어야 화면이 바로 그린다'


def test_no_prime_when_topic_not_latched():
    got = dict(m.plan_messages(ROWS, **{**WINDOW, 'prime_topic_ids': set()}))
    assert 13 not in got and 11 not in got


def test_empty_window_returns_nothing():
    assert m.plan_messages(ROWS, windows=[(S(20.0), S(21.0))], prime_topic_ids=set()) == []


# ── 여러 구간 이어 붙이기 (10-02) ────────────────────────────────────
#   한 상태가 길어 클립이 늘어질 때 그 가운데를 들어낸다. 뒤 구간은 앞 구간 끝에 붙어야
#   재생이 끊기지 않는다 — 공백이 남으면 화면이 그 시간만큼 멈춰 보인다.
TWO = dict(windows=[(S(1.0), S(2.0)), (S(7.0), S(10.0))], prime_topic_ids=set())


def test_stitched_windows_have_no_gap_between_them():
    got = dict(m.plan_messages(ROWS, **TWO))
    assert got[12] == S(1.0), '첫 구간은 원래 시각 그대로'
    # 두 번째 구간(7~10s)은 공백 5초(2→7)만큼 당겨진다
    assert got[16] == S(7.5) - (S(7.0) - S(2.0)), '뒤 구간이 앞 구간 끝에 붙는다'
    assert got[17] == S(10.0) - (S(7.0) - S(2.0))
    total = max(got.values()) - min(got.values())
    assert total == S(4.0) - S(0.0), f'총 길이 = 구간 길이 합(1+3초)이어야 한다: {total/1e9}s'


def test_stitched_windows_drop_the_gap_messages():
    got = dict(m.plan_messages(ROWS, **TWO))
    assert 14 not in got and 15 not in got, '들어낸 구간(4.9·5.0s)은 안 들어간다'


def test_prime_uses_only_the_first_window():
    got = dict(m.plan_messages(ROWS, windows=[(S(5.0), S(6.0)), (S(7.0), S(8.0))],
                               prime_topic_ids={MAP, TF}))
    assert got[13] == S(5.0) and got[11] == S(5.0), '프라임은 첫 구간 시작으로'


def test_display_topic_list_matches_console_subscriptions():
    """🔴 화면이 구독하는데 목록에 없으면 그 칸이 조용히 빈다 — 둘을 대조한다."""
    import re
    ros_js = (pathlib.Path(__file__).resolve().parents[1] /
              'console' / 'js' / 'ros.js').read_text(encoding='utf-8')
    subbed = set(re.findall(r"sub\('(/[\w/]+)'", ros_js))
    # 디스플레이·관제 지도에 필요한 것들은 반드시 들어 있어야 한다
    need = {'/mission_state', '/map', '/tf', '/tf_static', '/scan', '/plan', '/alarm'}
    assert need <= subbed, 'ros.js 구독 목록이 바뀌었다 — 이 검사의 전제가 깨졌다'
    assert need <= set(m.DISPLAY_TOPICS), f'빠진 토픽: {need - set(m.DISPLAY_TOPICS)}'
    assert set(m.PRIME_TOPICS) <= set(m.DISPLAY_TOPICS)
