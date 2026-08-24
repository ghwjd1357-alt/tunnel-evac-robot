"""bag 의 /scan 을 실제 FollowerMonitor 에 먹여 판정을 재현하고, 미션 실제 판정과 대조한다."""
import os
BAGS = os.environ.get('VIZ_BAGS', os.path.expanduser('~/robot_evidence'))
WORK = os.environ.get('VIZ_WORK', os.path.join(BAGS, 'viz', '_work'))
REPO = os.environ.get('VIZ_REPO', os.path.expanduser('~/ros2_ws'))
TAG  = os.environ.get('VIZ_TAG', 'realtake6')
os.makedirs(WORK, exist_ok=True)
import sqlite3, glob, sys, math, yaml, pickle
sys.path.insert(0, os.path.join(REPO,'src/mission_manager'))
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from mission_manager.follower_monitor import FollowerMonitor

TAG = sys.argv[1] if len(sys.argv) > 1 else TAG
DB = glob.glob(f'{BAGS}/{TAG}/*.db3')[0]
WP = yaml.safe_load(open(os.path.join(REPO,'src/mission_manager/config/waypoints_real_H.yaml')))
sb = WP['search_back']

class FakeClock:
    def __init__(self): self._t = Time(nanoseconds=0)
    def set(self, ns): self._t = Time(nanoseconds=int(ns))
    def now(self): return self._t

clock = FakeClock()
mon = FollowerMonitor(
    clock,
    cone_half_deg=float(sb.get('cone_half_deg', 60.0)),
    max_range=float(sb.get('detect_range', 2.5)),
    lost_sec=float(sb.get('lost_sec', 3.0)),
    seen_sec=float(sb.get('seen_sec', 1.0)),
    max_cluster_width=float(sb.get('cluster_max_width', 0.8)),
    min_points=int(sb.get('min_points', 3)),
    range_jump=float(sb.get('range_jump', 0.3)),
    edge_margin=float(sb.get('edge_margin', 0.2)),
    scan_timeout=float(sb.get('scan_timeout', 1.0)))
print(f'FollowerMonitor 인자: cone {sb.get("cone_half_deg")} · range {sb.get("detect_range")} · '
      f'lost {sb.get("lost_sec")} · seen {sb.get("seen_sec")} · width {sb.get("cluster_max_width")} · '
      f'minpts {sb.get("min_points")}')

con = sqlite3.connect(DB)
tp = {r[0]: (r[1], r[2]) for r in con.execute('select id,name,type from topics')}
n2i = {v[0]: k for k, v in tp.items()}
def rows(t):
    tid = n2i[t]; mt = get_message(tp[tid][1])
    for ts, d in con.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
        yield ts, deserialize_message(bytes(d), mt)
def st(h): return h.stamp.sec + h.stamp.nanosec / 1e9

# 기준 시각 = 첫 scan stamp
scans = [(st(m.header), m) for _, m in rows('/scan')]
scans.sort(key=lambda x: x[0])
T0 = scans[0][0]

frames = []      # (t, [(cluster 각/거리/폭/점수/사람여부)], visible, lost, stale)
prev = {'vis': None, 'lost': None}
events = []
for tabs, m in scans:
    clock.set(tabs * 1e9)
    mon.update(m)
    cl = []
    for c in mon._find_clusters(m):
        mean_r = sum(r for _, r in c) / len(c)
        span = (c[-1][0] - c[0][0]) * m.angle_increment
        width = mean_r * span + 0.05
        mid = (c[0][0] + c[-1][0]) / 2.0
        ang = m.angle_min + mid * m.angle_increment
        person = mon._is_person_like(c, m)
        why = ''
        if not person:
            if len(c) < mon.min_points: why = f'{len(c)}점 < 3'
            elif width > mon.max_cluster_width: why = f'폭 {width:.2f} > 0.80'
            else: why = '문턱 경계'
        cl.append(dict(ang=ang, r=mean_r, w=width, n=len(c), person=person, why=why,
                       pts=[(m.angle_min + i * m.angle_increment, r) for i, r in c]))
    v = mon.visible(zone='any'); l = mon.lost(zone='any'); s = mon.scan_stale()
    det = any(c['person'] for c in cl)
    ls = mon._last_seen_t.get('any'); fs = mon._first_seen_t.get('any')
    gap = (clock.now() - ls).nanoseconds / 1e9 if ls is not None else None
    held = (clock.now() - fs).nanoseconds / 1e9 if fs is not None else None
    raw=[]
    for i, r in enumerate(m.ranges):
        if math.isfinite(r) and m.range_min < r < 4.6:
            raw.append((m.angle_min + i * m.angle_increment, r))
    frames.append(dict(t=tabs - T0, cl=cl, vis=v, lost=l, stale=s,
                       det=det, gap=gap, held=held, raw=raw))
    if v != prev['vis']:   events.append((tabs - T0, f'visible → {v}')); prev['vis'] = v
    if l != prev['lost']:  events.append((tabs - T0, f'lost    → {l}')); prev['lost'] = l

print(f'\n스캔 {len(scans)}건 재생 · 기준시각 T0={T0:.3f}')
print('\n=== 내 재현의 전이 (t = 첫 scan 기준) ===')
for t, e in events: print(f'  {t:8.2f}  {e}')

print('\n=== 미션 실제 판정 (/rosout, 같은 시각계) ===')
for bt, m in rows('/rosout'):
    if m.name != 'mission_manager': continue
    if any(k in m.msg for k in ('놓침', '재발견', '역행 시작', 'GUIDE 복귀')):
        t = m.stamp.sec + m.stamp.nanosec / 1e9 - T0
        print(f'  {t:8.2f}  {m.msg[:64]}')

pickle.dump(dict(frames=frames, T0=T0), open(f'{WORK}/{TAG}_cluster.pkl', 'wb'))
print(f'\n저장: {TAG}_cluster.pkl')
