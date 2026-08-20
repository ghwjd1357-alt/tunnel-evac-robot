#!/usr/bin/env python3
"""08-21 실측값을 `waypoints_real_H.yaml` 에 넣는다 — 손으로 고치지 않게.

사용:
    python3 tools/apply_measurements.py --dry --guide-speed 0.08
    python3 tools/apply_measurements.py --guide-speed 0.08 --cluster-max-width 0.6 \
        --low-west 0.42,-10.61

왜 이 도구가 있나
-----------------
`waypoints_real_H.yaml` 에는 **잠정값이 6개** 있고 08-21 오전 측정으로 확정된다.
촬영 전날 밤에 사람이 손으로 6군데를 고치면 오타가 난다. 그리고 그 오타는
**미션 한복판에서야** 드러난다.

🔴 **주석을 보존한다.** 이 yaml 은 "왜 이 값인가"가 주석에 있고 그게 정본의 일부다.
`yaml.load` → `yaml.dump` 는 주석을 통째로 날리므로 **줄 단위로 갈아끼운다.**

🔴 쓰고 나면 `validate_waypoints` 로 다시 검사한다 — 넣은 값이 계약을 깨면
그 자리에서 잡는다(NaN·음수·불변조건).
"""

import argparse
import re
import shutil
import sys

sys.path.insert(0, 'src/mission_manager')

WP = 'src/mission_manager/config/waypoints_real_H.yaml'


def set_scalar(lines, key, value, indent=0):
    """`key: 값` 한 줄을 갈아끼운다. 뒤 주석은 살린다."""
    pat = re.compile(r'^(\s{%d}%s:\s*)([-\d.]+)(.*)$' % (indent, re.escape(key)))
    for i, ln in enumerate(lines):
        m = pat.match(ln)
        if m:
            old = m.group(2)
            lines[i] = f'{m.group(1)}{value}{m.group(3)}'
            return old
    return None


def set_node_xy(lines, node, x, y):
    """`corridor_graph.nodes.<node>: {x: .., y: ..}` 를 갈아끼운다."""
    pat = re.compile(r'^(\s+%s:\s*)\{[^}]*\}(.*)$' % re.escape(node))
    for i, ln in enumerate(lines):
        m = pat.match(ln)
        if m:
            old = ln.strip()
            lines[i] = f'{m.group(1)}{{x: {x:6.2f}, y: {y:7.2f}}}{m.group(2)}'
            return old
    return None


def set_escape(lines, x, y, yaw):
    pat = re.compile(r'^(escape:\s*)\{[^}]*\}(.*)$')
    for i, ln in enumerate(lines):
        m = pat.match(ln)
        if m:
            old = ln.strip()
            lines[i] = f'{m.group(1)}{{x: {x:.2f}, y: {y:.2f}, yaw: {yaw:.2f}}}{m.group(2)}'
            return old
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--file', default=WP)
    ap.add_argument('--dry', action='store_true', help='쓰지 않고 바뀔 것만 인쇄')
    # M1
    ap.add_argument('--guide-speed', type=float)
    ap.add_argument('--normal-speed', type=float)
    # M3
    ap.add_argument('--cluster-max-width', type=float)
    ap.add_argument('--detect-range', type=float)
    ap.add_argument('--min-points', type=int)
    # M4 — "x,y"
    ap.add_argument('--low-west', help='아래 복도 서쪽 좌표 "x,y" (그래프 노드 + 탈출구 동시 반영)')
    a = ap.parse_args()

    lines = open(a.file, encoding='utf-8').read().split('\n')
    changes = []

    def rec(what, old, new):
        if old is None:
            print(f'  🔴 {what}: 자리를 못 찾았다 — yaml 구조가 바뀌었나?')
            sys.exit(1)
        changes.append((what, old, new))

    if a.guide_speed is not None:
        rec('guide_speed', set_scalar(lines, 'guide_speed', f'{a.guide_speed:.2f}'),
            f'{a.guide_speed:.2f}')
    if a.normal_speed is not None:
        rec('normal_speed', set_scalar(lines, 'normal_speed', f'{a.normal_speed:.2f}'),
            f'{a.normal_speed:.2f}')
    if a.cluster_max_width is not None:
        rec('cluster_max_width',
            set_scalar(lines, 'cluster_max_width', f'{a.cluster_max_width:.2f}', indent=2),
            f'{a.cluster_max_width:.2f}')
    if a.detect_range is not None:
        rec('detect_range',
            set_scalar(lines, 'detect_range', f'{a.detect_range:.2f}', indent=2),
            f'{a.detect_range:.2f}')
    if a.min_points is not None:
        rec('min_points', set_scalar(lines, 'min_points', str(a.min_points), indent=2),
            str(a.min_points))
    if a.low_west:
        x, y = (float(v) for v in a.low_west.split(','))
        rec('graph.low_west', set_node_xy(lines, 'low_west', x, y), f'({x}, {y})')
        # 🔴 탈출구도 같이 옮긴다 — 둘이 갈리면 유도 목표가 그래프 밖이 된다
        rec('escape', set_escape(lines, x, y, 3.14), f'({x}, {y}, 3.14)')

    if not changes:
        print('바꿀 값이 없다. --help 를 볼 것.')
        return 1

    print(f'{a.file}')
    for what, old, new in changes:
        print(f'  {what:20s} {old}  →  {new}')

    if a.dry:
        print('\n--dry — 쓰지 않았다')
        return 0

    shutil.copy(a.file, a.file + '.bak')
    open(a.file, 'w', encoding='utf-8').write('\n'.join(lines))
    print(f'\n저장 완료 (백업 = {a.file}.bak)')

    # 🔴 넣은 값이 계약을 깨는지 그 자리에서 본다
    import yaml
    from mission_manager.mission_node import validate_waypoints
    wp = yaml.safe_load(open(a.file, encoding='utf-8'))
    bad = validate_waypoints(wp)
    if bad:
        print(f'🔴 검증 실패: {bad}')
        print(f'   되돌리려면: cp {a.file}.bak {a.file}')
        return 1
    print('🟢 validate_waypoints 통과')

    # 불변조건 — yaml 주석이 요구하는 것들
    gd = float(wp.get('gather_dist', 0))
    mfd = float(wp['search_back']['min_fire_dist'])
    if mfd >= gd:
        print(f'🔴 불변조건 위반: min_fire_dist({mfd}) >= gather_dist({gd})')
        return 1
    gs, ns = float(wp['guide_speed']), float(wp['normal_speed'])
    if gs > ns:
        print(f'🔶 guide_speed({gs}) > normal_speed({ns}) — 유도가 평시보다 빠르다')
    if ns > 0.12:
        print(f'🔴 normal_speed({ns}) 가 구동부 명령 상한 0.12 를 넘는다')
        return 1
    print('🟢 불변조건 통과')
    return 0


if __name__ == '__main__':
    sys.exit(main())
