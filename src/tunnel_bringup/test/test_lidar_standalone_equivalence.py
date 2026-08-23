"""🆘 복구용 라이다 설정이 런치와 **같은 로봇**인지 정적으로 대조한다 (2026-08-23 §91 P2-1).

왜 있나
-------
`docs/…/0823_촬영_런북.md §10` 은 라이다가 죽으면 `lidar_standalone.yaml` 로 노드만
되살리라고 지시한다. 그런데 그 yaml 은 `real_bringup.launch.py` 의 라이다 파라미터를
**손으로 베낀 사본**이다. 한쪽만 고치면 복구된 라이다가 원래와 다른 설정으로 돌고,
그 중 `frame_id` 가 어긋나면 costmap 이 스캔을 통째로 버린다 — 그런데 노드는 정상으로
보이고 `/scan` 도 나온다. 증상은 *"장애물이 안 보인다"* 로만 나타나 현장에서 못 찾는다.

🔴 그래서 **문서 주석("같아야 한다")에 맡기지 않고 검사로 못박는다.**
   런치를 고치고 yaml 을 안 고치면 여기서 FAIL 한다.

한계 (일부러 이렇게 한다)
-------------------------
런치를 실행하지 않고 **소스를 정적으로 읽는다**(`ast`). 런치 실행은 ROS 환경·장치를
요구해서 회귀에 못 넣는다. 대신 리터럴로 적힌 값만 대조하고, `LaunchConfiguration` 으로
넘어가는 두 키(`serial_port`·`serial_baudrate`)는 **그 인자의 `default_value`** 와 맞춘다.
"""
import ast
import pathlib

import yaml

_PKG = pathlib.Path(__file__).resolve().parents[1]
_LAUNCH = _PKG / 'launch' / 'real_bringup.launch.py'
_YAML = _PKG / 'config' / 'lidar_standalone.yaml'

# 🔴 어긋나면 현장에서 못 찾는 키들. `serial_port`·`serial_baudrate` 는 런치 인자
#   default_value 에서, 나머지는 노드 parameters 딕셔너리 리터럴에서 읽는다.
_FROM_LAUNCH_ARG = {'serial_port': 'lidar_port', 'serial_baudrate': 'lidar_baud'}
_LITERAL_KEYS = ('channel_type', 'frame_id', 'inverted',
                 'angle_compensate', 'scan_mode', 'use_sim_time')


def _launch_tree():
    return ast.parse(_LAUNCH.read_text(encoding='utf-8'))


def _sllidar_params(tree):
    """`package='sllidar_ros2'` 인 Node(...) 의 parameters=[{...}] 리터럴을 뽑는다."""
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        kw = {k.arg: k.value for k in node.keywords if k.arg}
        pkg = kw.get('package')
        if not (isinstance(pkg, ast.Constant) and pkg.value == 'sllidar_ros2'):
            continue
        params = kw.get('parameters')
        assert isinstance(params, ast.List) and len(params.elts) == 1, \
            'sllidar Node 의 parameters 모양이 바뀌었다 — 이 검사를 같이 고쳐라'
        d = params.elts[0]
        assert isinstance(d, ast.Dict)
        out = {}
        for k, v in zip(d.keys, d.values):
            assert isinstance(k, ast.Constant), 'parameters 키가 리터럴이 아니다'
            out[k.value] = v
        return out
    raise AssertionError('real_bringup.launch.py 에서 sllidar_ros2 Node 를 못 찾았다')


def _declared_defaults(tree):
    """DeclareLaunchArgument('name', default_value='...') → {name: default}."""
    out = {}
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'DeclareLaunchArgument'):
            continue
        if not (node.args and isinstance(node.args[0], ast.Constant)):
            continue
        dv = next((k.value for k in node.keywords if k.arg == 'default_value'), None)
        if isinstance(dv, ast.Constant):
            out[node.args[0].value] = dv.value
    return out


def _standalone():
    doc = yaml.safe_load(_YAML.read_text(encoding='utf-8'))
    assert list(doc) == ['/**'], f'네임스페이스 와일드카드 하나여야 한다: {list(doc)}'
    return doc['/**']['ros__parameters']


def test_standalone_matches_launch_literals():
    """리터럴로 적힌 키가 전부 같은가 — 특히 `frame_id`."""
    params, sa = _sllidar_params(_launch_tree()), _standalone()
    for key in _LITERAL_KEYS:
        assert key in params, f'런치에서 {key} 가 사라졌다 — 이 검사를 같이 고쳐라'
        node = params[key]
        assert isinstance(node, ast.Constant), \
            f'{key} 가 더 이상 리터럴이 아니다 — 이 검사를 같이 고쳐라'
        assert key in sa, f'🔴 lidar_standalone.yaml 에 {key} 가 없다'
        assert sa[key] == node.value, \
            f'🔴 {key} 불일치 — 런치 {node.value!r} vs standalone {sa[key]!r}'


def test_standalone_matches_launch_argument_defaults():
    """런치 인자로 넘어가는 두 키는 그 인자의 default_value 와 같아야 한다."""
    defaults = _declared_defaults(_launch_tree())
    sa = _standalone()
    for key, arg in _FROM_LAUNCH_ARG.items():
        assert arg in defaults, f'DeclareLaunchArgument({arg!r}) 가 사라졌다'
        want = defaults[arg]
        assert key in sa, f'🔴 lidar_standalone.yaml 에 {key} 가 없다'
        got = sa[key]
        # `lidar_baud` 는 문자열 default('460800'), yaml 은 int 460800 이다.
        assert str(got) == str(want), \
            f'🔴 {key} 불일치 — 런치 인자 {arg} 기본값 {want!r} vs standalone {got!r}'


def test_key_sets_match_in_both_directions():
    """🔴 08-23 §91(2회차) P1-1 — 구판은 `standalone - launch` **한 방향만** 봤다.

    그래서 **런치에 9번째 키를 추가하면 세 검사가 전부 통과했다**(검토가 주입해 확인).
    그 상태로 복구하면 라이다가 런치에 있는 설정 하나를 **빠뜨린 채** 돈다 — 이 파일이
    막으려던 바로 그 drift 다. 양방향으로 대조한다.
    """
    launch_keys = set(_sllidar_params(_launch_tree()))
    sa_keys = set(_standalone())
    only_sa = sa_keys - launch_keys
    only_launch = launch_keys - sa_keys
    assert not only_sa, \
        f'🔴 standalone 에만 있는 키: {sorted(only_sa)} — 런치에 없는 설정으로 복구된다'
    assert not only_launch, \
        (f'🔴 런치에만 있는 키: {sorted(only_launch)} — 복구용 라이다가 이 설정을 '
         f'빠뜨린 채 돈다. lidar_standalone.yaml 에 같이 넣어라.')


def test_the_pinned_key_list_still_covers_everything():
    """대조 목록(`_LITERAL_KEYS` + `_FROM_LAUNCH_ARG`)이 런치 전체를 덮는지.

    키가 늘었는데 목록을 안 늘리면 위 값 검사가 그 키를 **안 본다** — 집합 검사만
    통과하고 값 불일치는 새어 나간다. 목록 자체를 계약으로 고정한다.
    """
    launch_keys = set(_sllidar_params(_launch_tree()))
    covered = set(_LITERAL_KEYS) | set(_FROM_LAUNCH_ARG)
    missing = launch_keys - covered
    assert not missing, (
        f'🔴 런치에 새 키가 생겼는데 대조 목록에 없다: {sorted(missing)} — '
        f'_LITERAL_KEYS 또는 _FROM_LAUNCH_ARG 에 추가하라')
