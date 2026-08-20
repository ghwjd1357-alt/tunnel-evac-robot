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

🔴 **검증 전에는 원본을 건드리지 않는다** (08-21 Codex §82.8 재현 반영)
--------------------------------------------------------------------
구판은 `write` → `validate` 순서였다. 재현:

    apply_measurements.py --file t.yaml --normal-speed nan
      → 저장 완료 (백업 = t.yaml.bak) … 그 뒤에 🔴 검증 실패
      → 원본 normal_speed: nan

    이어서 --guide-speed 0.09
      → .bak 을 **이미 nan 인 파일로 덮은** 뒤 다시 실패
      → 원본도 백업도 nan. 도구가 안내하는 `cp .bak` 복구는 오염본을 되돌린다.

그래서 순서를 뒤집었다:

    입력 검사 → 후보 텍스트를 **메모리에서** 만든다 → yaml 파싱 → validate_waypoints
    → 불변조건 → **전부 통과한 뒤에만** 같은 디렉터리 임시파일 + fsync + os.replace

같은 디렉터리에 쓰는 이유 = `os.replace` 가 원자적이려면 같은 파일시스템이어야 한다.
백업은 **원본이 이미 유효할 때만** 만든다 — 깨진 원본으로 last-good 을 덮지 않는다.
"""

import argparse
import math
import os
import re
import sys
import tempfile

sys.path.insert(0, 'src/mission_manager')

WP = 'src/mission_manager/config/waypoints_real_H.yaml'


# ─────────────────────────────────────────────────────────────
# 1. 입력 검사 — 파일을 열기도 전에 막는다
# ─────────────────────────────────────────────────────────────
def finite_positive(name, v):
    """NaN/Inf/음수/0 을 여기서 죽인다. 통과한 값만 텍스트가 된다."""
    if v is None:
        return None
    if not math.isfinite(v):
        return f'{name}: 유한값이 아니다 ({v!r})'
    if v <= 0:
        return f'{name}: 0 이하 ({v!r}) — 속도·폭·거리는 양수여야 한다'
    return None


def check_inputs(a):
    errs = []
    for name, v in (('guide_speed', a.guide_speed),
                    ('normal_speed', a.normal_speed),
                    ('cluster_max_width', a.cluster_max_width),
                    ('detect_range', a.detect_range)):
        e = finite_positive(name, v)
        if e:
            errs.append(e)
    if a.min_points is not None and a.min_points < 1:
        errs.append(f'min_points: 1 미만 ({a.min_points}) — 클러스터가 성립 불가')
    if a.low_west:
        parts = a.low_west.split(',')
        if len(parts) != 2:
            errs.append(f'low_west: "x,y" 형식이 아니다 ({a.low_west!r})')
        else:
            try:
                xy = [float(v) for v in parts]
            except ValueError:
                errs.append(f'low_west: 숫자로 못 읽는다 ({a.low_west!r})')
            else:
                if not all(math.isfinite(v) for v in xy):
                    errs.append(f'low_west: 유한값이 아니다 ({a.low_west!r})')
    return errs


# ─────────────────────────────────────────────────────────────
# 2. 후보 텍스트 생성 — 순수 함수. 파일에 안 쓴다
# ─────────────────────────────────────────────────────────────
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


def build_candidate(text, a):
    """원본 텍스트 → (후보 텍스트, 변경목록). 자리를 못 찾으면 (None, 사유)."""
    lines = text.split('\n')
    changes = []

    def rec(what, old, new):
        if old is None:
            raise KeyError(what)
        changes.append((what, old, new))

    try:
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
    except KeyError as e:
        return None, f'{e.args[0]}: 자리를 못 찾았다 — yaml 구조가 바뀌었나?'
    return '\n'.join(lines), changes


# ─────────────────────────────────────────────────────────────
# 3. 검증 — 텍스트만 받는다. 파일 경로를 안 받는 것이 핵심
# ─────────────────────────────────────────────────────────────
def validate_text(text):
    """(wp, 오류목록). 오류가 하나라도 있으면 이 텍스트는 절대 안 쓴다."""
    import yaml
    from mission_manager.mission_node import validate_waypoints
    try:
        wp = yaml.safe_load(text)
    except yaml.YAMLError as e:
        return None, [f'yaml 파싱 실패: {e}']
    if not isinstance(wp, dict):
        return None, ['yaml 최상위가 매핑이 아니다']

    errs = list(validate_waypoints(wp) or [])
    if errs:
        return wp, [f'validate_waypoints: {errs}']

    # 불변조건 — yaml 주석이 요구하는 것들
    try:
        gd = float(wp.get('gather_dist', 0))
        mfd = float(wp['search_back']['min_fire_dist'])
        gs, ns = float(wp['guide_speed']), float(wp['normal_speed'])
    except (KeyError, TypeError, ValueError) as e:
        return wp, [f'불변조건 검사에 필요한 키를 못 읽었다: {e}']
    if not all(math.isfinite(v) for v in (gd, mfd, gs, ns)):
        return wp, ['불변조건: gather_dist·min_fire_dist·속도 중 유한값 아닌 것이 있다']
    if mfd >= gd:
        return wp, [f'불변조건: min_fire_dist({mfd}) >= gather_dist({gd})']
    if ns > 0.12:
        return wp, [f'불변조건: normal_speed({ns}) 가 구동부 명령 상한 0.12 를 넘는다']
    return wp, []


def no_dispatch_zone(wp, step=0.05):
    """🔵 S1-3 확인 — 그래프 위에서 '자동 출동하지 않는' 구간이 얼마나 되나.

    mission_node 는 계산된 집결지가 화재에서 min_fire_dist 안이면 알람을 거부한다
    (08-21 §82.7). 그 거부 구간은 탈출구 주변에 생기며, **결함이 아니라 정책**이다.
    다만 촬영자가 그 자리를 모르면 '왜 무반응이지'가 되므로 길이를 인쇄한다."""
    from mission_manager.mission_node import compute_gather_point_graph
    g = wp.get('corridor_graph')
    if not g:
        return None
    esc, gd = wp['escape'], float(wp.get('gather_dist', 8.0))
    mfd = float(wp['search_back']['min_fire_dist'])
    nodes = {n: (float(p['x']), float(p['y'])) for n, p in g['nodes'].items()}
    blocked = 0.0
    for a_, b_ in g['edges']:
        if a_ not in nodes or b_ not in nodes:
            continue
        (x0, y0), (x1, y1) = nodes[a_], nodes[b_]
        seg = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(seg / step))
        for i in range(n + 1):
            t = i / n
            fx, fy = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            r = compute_gather_point_graph(fx, fy, float(esc['x']), float(esc['y']), gd, g)
            eff = r if r is not None else wp.get('gather')
            if eff is None:
                continue
            if math.hypot(float(eff['x']) - fx, float(eff['y']) - fy) < mfd:
                blocked += seg / n
    return blocked


# ─────────────────────────────────────────────────────────────
# 4. 원자적 쓰기
# ─────────────────────────────────────────────────────────────
def atomic_write(path, text):
    """같은 디렉터리 임시파일 → fsync → os.replace. 중간에 죽어도 원본은 온전하다."""
    d = os.path.dirname(os.path.abspath(path)) or '.'
    fd, tmp = tempfile.mkstemp(dir=d, prefix='.apply_', suffix='.tmp')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise


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

    # ① 입력 검사 — 파일을 열기 전에
    errs = check_inputs(a)
    if errs:
        for e in errs:
            print(f'🔴 입력 거부 — {e}')
        print('   원본은 건드리지 않았다.')
        return 1

    original = open(a.file, encoding='utf-8').read()

    # ② 후보 생성 (메모리)
    cand, changes = build_candidate(original, a)
    if cand is None:
        print(f'🔴 {changes}')
        print('   원본은 건드리지 않았다.')
        return 1
    if not changes:
        print('바꿀 값이 없다. --help 를 볼 것.')
        return 1

    print(f'{a.file}')
    for what, old, new in changes:
        print(f'  {what:20s} {old}  →  {new}')

    # ③ 검증 (여전히 메모리)
    wp, verrs = validate_text(cand)
    if verrs:
        for e in verrs:
            print(f'🔴 검증 실패 — {e}')
        print('   🟢 원본과 백업은 그대로다 (쓰기 전에 막았다).')
        return 1
    print('🟢 validate_waypoints + 불변조건 통과')

    if a.dry:
        print('\n--dry — 쓰지 않았다')
        return 0

    # ④ 백업은 **원본이 유효할 때만**. 깨진 원본으로 last-good 을 덮지 않는다.
    _, oerrs = validate_text(original)
    bak = a.file + '.bak'
    if oerrs:
        print(f'⚠ 원본이 이미 검증을 통과하지 못한다 {oerrs}')
        print(f'   → 백업을 갱신하지 않는다 ({bak} 의 기존 내용을 보존)')
    else:
        atomic_write(bak, original)
        print(f'   백업 = {bak}')

    # ⑤ 원자적 교체
    atomic_write(a.file, cand)
    print('저장 완료 (원자적 교체)')

    z = no_dispatch_zone(wp)
    if z is not None:
        mfd = float(wp['search_back']['min_fire_dist'])
        print(f'🔵 S1-3 자동 출동 거부 구간 ≈ {z:.2f} m '
              f'(탈출구 주변 · min_fire_dist {mfd} m 정책). '
              f'그 자리 화재는 관제 판단으로 넘어간다 — 결함이 아니다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
