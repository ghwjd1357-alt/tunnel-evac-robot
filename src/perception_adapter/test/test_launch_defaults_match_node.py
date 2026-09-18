"""런치 기본값 ↔ 노드 기본값 정합 (2026-09-18).

🔴 왜 있나: `test_p20` 은 노드의 `min_confidence` 기본값(0.60)을 검사해 초록이었는데,
`adapter.launch.py` 가 `default_value='0.40'` 을 넘겨 덮어써서 **실차 어댑터는 0.4 로 돌았다**
(젯슨 `ros2 param get /perception_adapter min_confidence` 실측). 검사가 자기 대상(실제 기동
경로)을 안 지나는 자리다 — 예약 73 과 같은 클래스. 여기서는 런치가 노드에 **넘기는 모든**
인자에 대해 런치 기본값과 노드 기본값이 같은지 대조한다. 런치 쪽에서 값을 바꾸고 싶으면
노드 기본값도 같이 바꾼다(한 곳만 바꾸면 여기서 붉어진다).
"""
import pathlib, re

ROOT = pathlib.Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'launch' / 'adapter.launch.py'
NODE = ROOT / 'perception_adapter' / 'adapter_node.py'


def _launch_passed_defaults():
    src = LAUNCH.read_text(encoding='utf-8')
    declared = dict(re.findall(r"DeclareLaunchArgument\(\s*'(\w+)',\s*default_value='([^']*)'", src))
    passed = re.findall(r"'(\w+)':\s*LaunchConfiguration\('(\w+)'\)", src)
    return {param: declared[arg] for param, arg in passed if arg in declared}


def _node_defaults():
    src = NODE.read_text(encoding='utf-8')
    return dict(re.findall(r"declare_parameter\('(\w+)',\s*([^)]+)\)", src))


def _same(launch_v, node_v):
    node_v = node_v.strip().strip("'\"")
    try:
        return float(launch_v) == float(node_v)
    except ValueError:
        return launch_v.lower() == node_v.lower()


def test_launch_passes_at_least_the_fire_threshold():
    passed = _launch_passed_defaults()
    assert 'min_confidence' in passed, '런치가 min_confidence 를 안 넘기면 이 검사는 대상을 잃는다'


def test_every_launch_default_matches_node_default():
    passed, node = _launch_passed_defaults(), _node_defaults()
    diverged = {k: (v, node.get(k)) for k, v in passed.items()
                if k in node and not _same(v, node[k])}
    assert not diverged, f'런치 기본값 ≠ 노드 기본값: {diverged}'


def test_fire_threshold_in_launch_clears_role_b_false_positives():
    """test_p20 과 같은 판정을 **런치 경로**에서도 — 0.58 오탐이 통과하면 안 된다."""
    assert float(_launch_passed_defaults()['min_confidence']) > 0.58
