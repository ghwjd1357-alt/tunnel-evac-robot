#!/usr/bin/env python3
"""사람 점뭉치 판별 실측 — `cluster_max_width`·`detect_range`·`min_points` 결정용.

사용:
    # ① 분포만 보기
    python3 tools/cluster_width_report.py <사람bag> --range 2.35

    # ② 🔴 문턱을 정하려면 대조군이 필요하다 (사람 **없는** 같은 자리 bag)
    python3 tools/cluster_width_report.py <사람bag> --compare <대조군bag> \
        --segments 1.2m=0:30,2.0m=30:60,2.35m=60:90

왜 이 도구가 있나
-----------------
`search_back.cluster_max_width: 0.8` 은 **폭 6 m 시뮬 터널** 값이다. 실복도는
**2.35 m**(반폭 1.18 m)라 검출 범위 안에 벽이 들어온다. 벽 조각이 문턱 밑으로
잘리면 **벽이 사람으로** 잡히고, 반대로 사람이 벽과 한 덩어리가 되면
**사람이 벽으로 배제**된다. 둘 다 `SEARCH_BACK` 장면을 깬다.

🔴 08-21 Codex §82.6 재현 반영 — 분모를 바꾸면 결론이 뒤집혔다
--------------------------------------------------------------
구판은 `통과 클러스터 / 전체 클러스터` 를 봤고, 2.5→1.2 m 에서 27→30% 였다.
그래서 *"detect_range 는 지렛대가 아니다"* 라고 적었다. 그런데 같은 자료의
**스캔당 통과 수**는 0.88 → 0.51 로 **42% 줄었다.** 결론이 분모가 만든 것이었다.

그리고 더 근본적으로, 구판은 클러스터를 평평한 목록에 넣어 **어느 스캔에
속했는지 버렸다.** 실제 판정은 그게 아니다:

    스캔마다 "사람 같은 덩어리가 하나라도 있나" (boolean)
      → 그 boolean 이 **seen_sec(1.0초) 연속**이어야 visible() 이 True

같은 총 클러스터 수라도 **몰려 있으면** visible=False, **고르게 이어지면**
visible=True 다. 평균은 그 둘을 구별하지 못한다.

⇒ 이제 이 도구는 통계를 새로 만들지 않는다. **`FollowerMonitor` 를 조합마다
   하나씩 만들어 bag 시계로 `update()` → `visible(zone='any')` 를 그대로
   돌리고, True 가 연속된 구간(run)을 센다.** `SEARCH_BACK` 이 쓰는 zone 이
   'any' 이므로(`mission_node.py:774`) 여기서도 'any' 를 본다.

판정:
    대조군에서 **visible run 0개**  = 벽이 사람으로 재발견되지 않는다
    사람bag 의 **각 거리 구간마다 run ≥ 1** = 세 거리 모두에서 사람이 잡힌다
    둘을 동시에 만족하는 조합이 없으면 **rc=1** — 추천하지 않는다.
"""

import argparse
import statistics
import sys

sys.path.insert(0, 'src/mission_manager')

from rclpy.serialization import deserialize_message                  # noqa: E402
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions  # noqa: E402
from sensor_msgs.msg import LaserScan                                # noqa: E402

from mission_manager.follower_monitor import FollowerMonitor         # noqa: E402


class _Clock:
    """FollowerMonitor 가 요구하는 최소 시계 — bag 재생이라 실시계를 안 쓴다."""
    class _T:
        def __init__(self, ns):
            self.nanoseconds = ns

        def __sub__(self, other):
            return _Clock._T(self.nanoseconds - other.nanoseconds)

    def __init__(self):
        self.ns = 0

    def now(self):
        return _Clock._T(self.ns)


def read_scans(bag):
    """bag 의 /scan 전량을 (시각ns, LaserScan) 으로 한 번만 읽어 둔다.

    ⚠ 조합 sweep 마다 bag 을 다시 열면 수십 번 디스크를 훑는다. 한 번 읽고
      메모리에서 여러 번 재생한다 — 판정은 같고 시간만 줄어든다."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('', ''))
    out = []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == '/scan':
            out.append((t, deserialize_message(data, LaserScan)))
    return out


def clusters_of(scans, max_range):
    """분포표용 — (평균거리, 폭, 점수) 목록. 판정에는 쓰지 않는다."""
    clock = _Clock()
    mon = FollowerMonitor(clock, max_range=max_range)
    rows = []
    for t, scan in scans:
        clock.ns = t
        for c in mon._find_clusters(scan):
            mean_r = sum(rr for _, rr in c) / len(c)
            span = (c[-1][0] - c[0][0]) * scan.angle_increment
            rows.append((mean_r, mean_r * span + 0.05, len(c)))
    return rows


def replay(scans, width, rng, min_pts, zone='any'):
    """🔵 실제 판정 경로를 그대로 재생한다.

    반환 (visible_run 개수, visible 인 스캔 수, 전체 스캔 수, run 구간 목록[초]).
    run = visible() 이 False→True 로 바뀐 뒤 True 로 이어진 구간.
    """
    clock = _Clock()
    mon = FollowerMonitor(clock, max_range=rng, min_points=min_pts,
                          max_cluster_width=width)
    runs, vis_scans, prev = [], 0, False
    t0 = scans[0][0] if scans else 0
    start = None
    for t, scan in scans:
        clock.ns = t
        mon.update(scan)
        v = mon.visible(zone=zone)
        if v:
            vis_scans += 1
            if not prev:
                start = t
        elif prev:
            runs.append(((start - t0) / 1e9, (t - t0) / 1e9))
            start = None
        prev = v
    if prev and start is not None:
        runs.append(((start - t0) / 1e9, (scans[-1][0] - t0) / 1e9))
    return len(runs), vis_scans, len(scans), runs


def parse_segments(text, scans):
    """"라벨=시작:끝,…" (초, bag 시작 기준) → [(라벨, 스캔목록)]."""
    if not text:
        return [('전체', scans)]
    t0 = scans[0][0]
    out = []
    for part in text.split(','):
        label, _, span = part.partition('=')
        lo, _, hi = span.partition(':')
        lo, hi = float(lo), float(hi)
        sel = [(t, s) for t, s in scans if lo <= (t - t0) / 1e9 < hi]
        out.append((label or f'{lo:g}~{hi:g}s', sel))
    return out


def floats(text):
    return [float(v) for v in text.split(',')]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--range', type=float, default=2.35,
                    help='분포표용 검출 범위 [m]')
    ap.add_argument('--compare', metavar='EMPTY_BAG',
                    help='🔴 사람 **없는** 대조군 bag. 주면 실제 판정으로 sweep 한다')
    ap.add_argument('--segments', default='',
                    help='사람 거리 구간 "1.2m=0:30,2.0m=30:60,2.35m=60:90" (초)')
    ap.add_argument('--sweep-width', default='0.4,0.5,0.6,0.7,0.8,0.9,1.0')
    ap.add_argument('--sweep-range', default='1.2,1.8,2.35')
    ap.add_argument('--sweep-min-points', default='3,4,5')
    ap.add_argument('--zone', default='any', choices=('any', 'rear'),
                    help="SEARCH_BACK 이 쓰는 zone 은 'any' 다 (기본값)")
    a = ap.parse_args()

    scans = read_scans(a.bag)
    if not scans:
        print(f'🔴 /scan 이 없다: {a.bag}')
        return 1

    rows = clusters_of(scans, a.range)
    dur = (scans[-1][0] - scans[0][0]) / 1e9
    print(f'{a.bag}\n  스캔 {len(scans)}개 · {dur:.1f}초 · '
          f'클러스터 {len(rows)}개 (검출 범위 {a.range} m)\n')

    if rows:
        print(f'{"거리대(m)":>10} {"개수":>6} {"폭 중앙값":>10} {"폭 p90":>8} {"폭 최대":>8}')
        for lo, hi in [(0.0, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 99.0)]:
            sel = sorted(w for d, w, _ in rows if lo <= d < hi)
            if not sel:
                continue
            p90 = sel[int(len(sel) * 0.9)] if len(sel) > 1 else sel[0]
            print(f'{lo:.1f}~{hi:<5.1f} {len(sel):>6} {statistics.median(sel):>10.3f} '
                  f'{p90:>8.3f} {max(sel):>8.3f}')
        print()

    if not a.compare:
        print('🔴 이 표만으로는 문턱을 못 정한다 (§82.6).')
        print('   같은 자리에서 **사람 없는** 대조군 bag 을 찍어 --compare 로 대조할 것.')
        print('   분포는 참고이고, 판정은 스캔별 연속성이 정한다.')
        return 0

    return sweep(scans, a)


def sweep(scans, a):
    """실제 판정 경로로 (width × range × min_points) 를 훑는다."""
    empty = read_scans(a.compare)
    if not empty:
        print(f'🔴 대조군 bag 에 /scan 이 없다: {a.compare}')
        return 1
    segs = parse_segments(a.segments, scans)
    for label, sel in segs:
        if not sel:
            print(f'🔴 구간 "{label}" 에 스캔이 0개다 — --segments 초 범위를 확인할 것')
            return 1

    edur = (empty[-1][0] - empty[0][0]) / 1e9
    print(f'대조군: {a.compare} — 스캔 {len(empty)}개 · {edur:.1f}초')
    print(f'구간: {", ".join(f"{lb}({len(s)}스캔)" for lb, s in segs)}')
    print(f"판정 = FollowerMonitor.visible(zone='{a.zone}') 재생 · "
          f'seen_sec 1.0초 연속\n')

    hdr = f'{"width":>6} {"range":>6} {"minpt":>6} {"대조군run":>9} ' + \
          ' '.join(f'{lb:>9}' for lb, _ in segs) + '   판정'
    print(hdr)
    print('-' * len(hdr))

    ok = []
    for rng in floats(a.sweep_range):
        for mp in (int(v) for v in floats(a.sweep_min_points)):
            for w in floats(a.sweep_width):
                eruns, _, _, _ = replay(empty, w, rng, mp, a.zone)
                seg_runs = [replay(sel, w, rng, mp, a.zone)[0] for _, sel in segs]
                good = (eruns == 0) and all(r >= 1 for r in seg_runs)
                if good:
                    mark = '🟢 조건 충족'
                    ok.append((w, rng, mp, sum(seg_runs)))
                elif eruns > 0 and all(r >= 1 for r in seg_runs):
                    mark = f'🔴 대조군 오탐 {eruns}회'
                elif eruns == 0:
                    mark = '🔴 사람 미검출 구간 있음'
                else:
                    mark = '🔴 둘 다 실패'
                print(f'{w:>6.2f} {rng:>6.2f} {mp:>6d} {eruns:>9d} ' +
                      ' '.join(f'{r:>9d}' for r in seg_runs) + f'   {mark}')

    print()
    if not ok:
        print('🔴 조건을 동시에 만족하는 조합이 없다.')
        print('   ① 대본에서 사람이 멈추는 자리를 벽에서 더 멀리 옮긴다')
        print('   ② SEARCH_BACK 장면을 도박으로 인정하고 감수 여부를 사람이 정한다')
        print('   ③ 🔴 추천값을 억지로 만들지 않는다 — 그게 §82.6 이 지적한 실패다')
        return 1

    # 사람 쪽 run 이 가장 많은(= 여유가 큰) 조합. 동률이면 width 가 작은 쪽.
    ok.sort(key=lambda r: (-r[3], r[0]))
    w, rng, mp, tot = ok[0]
    print(f'🟢 추천 = cluster_max_width {w:.2f} · detect_range {rng:.2f} · '
          f'min_points {mp}  (대조군 오탐 run 0 · 사람 run 합계 {tot})')
    print('   반영: python3 tools/apply_measurements.py '
          f'--cluster-max-width {w:.2f} --detect-range {rng:.2f} --min-points {mp}')
    print('   ⚠ 이 값은 이 두 bag 이 찍힌 자리에서만 검증됐다. 자리를 옮기면 다시 잰다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
