"""bag 의 /scan 을 실제 FollowerMonitor 에 먹여 판정을 재현하고, 미션 실제 판정과 대조한다."""
import os
BAGS = os.environ.get('VIZ_BAGS', os.path.expanduser('~/robot_evidence'))
WORK = os.environ.get('VIZ_WORK', os.path.join(BAGS, 'viz', '_work'))
REPO = os.environ.get('VIZ_REPO', os.path.expanduser('~/ros2_ws'))
TAG  = os.environ.get('VIZ_TAG', 'realtake6')
os.makedirs(WORK, exist_ok=True)
import sqlite3, glob, sys, math, yaml, pickle, bisect
import numpy as np
from scipy import ndimage
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

# ══════════════════════════════════════════════════════════════
#  정적 지도 필터 (예약 80 대응안) — 벽 모서리 오탐 제거
#
#  라이다만 보면 벽 모서리가 폭 0.5~0.7 m 조각으로 잘려 '사람 크기'를
#  통과한다. 실측(realtake6): 사람 판정 8117건 중 **2607건(32.1%)이
#  지도상 벽 자리**였다.
#  기하 특징으로는 안 걸러진다 — 폭<=0.60 은 벽의 71.8% 를 그대로 통과시키고,
#  깊이범위<=0.20 은 벽을 35% 까지 줄이는 대신 사람을 91 프레임에서 놓친다.
#
#  로봇은 이미 지도 안에서 자기 위치를 안다. 덩어리 중심을 map 으로 옮겨
#  **정적 지도의 벽 자리면 사람이 아니다** 로 거른다.
#  실측: 벽 오탐 2607건 전부 제거 · 사람 5510건 전부 유지 · 놓친 프레임 0.
#
#  🔴 전제: 정합(map→odom)이 살아 있어야 한다. 정합이 틀어지면 벽에 붙어 선
#     사람을 지울 수 있다. 팽창 반경 WALL_PAD 가 그 여유다.
#  🔴 아직 운영 FollowerMonitor 에는 안 들어갔다 — 여기서 검증 중인 안이다.
# ══════════════════════════════════════════════════════════════
WALL_PAD = float(os.environ.get('VIZ_WALL_PAD', 0.25))   # [m] 벽 팽창 여유
MPKL = os.path.join(WORK, f'{TAG}.pkl')
_MP = pickle.load(open(MPKL,'rb'))
_g = _MP['map']['g']; _res=_MP['map']['res']
_ox=_MP['map']['ox']; _oy=_MP['map']['oy']
_WALL = ndimage.binary_dilation(_g>=50, ndimage.generate_binary_structure(2,2),
                                iterations=int(round(WALL_PAD/_res)))
_POSE=_MP['pose']; _PT=[p[0] for p in _POSE]; _MT0=_MP['T0']

def _pose_abs(tabs):
    """절대시각 → (x,y,yaw). extract2 pkl 과 같은 시각계로 맞춘다."""
    i=min(max(bisect.bisect_right(_PT, tabs-_MT0)-1,0),len(_POSE)-1)
    return _POSE[i][1],_POSE[i][2],_POSE[i][3]

def _on_static_wall(wx,wy):
    gi=int(round((wx-_ox)/_res)); gj=int(round((wy-_oy)/_res))
    if not (0<=gi<_g.shape[1] and 0<=gj<_g.shape[0]): return False
    return bool(_WALL[gj,gi])

class MapFilteredMonitor(FollowerMonitor):
    """운영 판정 그대로 + 정적 지도 벽이면 배제. 디바운스는 부모 것을 쓴다."""
    _pose=(0.0,0.0,0.0)
    def _is_person_like(self, cluster, scan):
        if not super()._is_person_like(cluster, scan):
            return False
        mean_r = sum(r for _, r in cluster) / len(cluster)
        mid = (cluster[0][0] + cluster[-1][0]) / 2.0
        ang = scan.angle_min + mid * scan.angle_increment
        px,py,pa = self._pose
        return not _on_static_wall(px+mean_r*math.cos(pa+ang),
                                   py+mean_r*math.sin(pa+ang))

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

monf = MapFilteredMonitor(
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
print(f'정적 지도 필터: 벽 팽창 {WALL_PAD:.2f} m')

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
fps_wall=0; fps_keep=0
for tabs, m in scans:
    clock.set(tabs * 1e9)
    monf._pose = _pose_abs(tabs)
    mon.update(m); monf.update(m)
    px,py,pa = monf._pose
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
        wx=px+mean_r*math.cos(pa+ang); wy=py+mean_r*math.sin(pa+ang)
        onwall=_on_static_wall(wx,wy)
        person_f = bool(person and not onwall)      # 지도 필터 적용 후
        if person and onwall: fps_wall+=1
        elif person:          fps_keep+=1
        if person and onwall and not why: why='지도상 벽'
        cl.append(dict(ang=ang, r=mean_r, w=width, n=len(c), person=person,
                       person_f=person_f, onwall=onwall, why=why, wx=wx, wy=wy,
                       pts=[(m.angle_min + i * m.angle_increment, r) for i, r in c]))
    v = monf.visible(zone='any'); l = monf.lost(zone='any'); s = monf.scan_stale()
    v0 = mon.visible(zone='any');  l0 = mon.lost(zone='any')
    det = any(c['person_f'] for c in cl)
    det0 = any(c['person'] for c in cl)
    ls = monf._last_seen_t.get('any'); fs = monf._first_seen_t.get('any')
    gap = (clock.now() - ls).nanoseconds / 1e9 if ls is not None else None
    held = (clock.now() - fs).nanoseconds / 1e9 if fs is not None else None
    raw=[]
    for i, r in enumerate(m.ranges):
        if math.isfinite(r) and m.range_min < r < 4.6:
            raw.append((m.angle_min + i * m.angle_increment, r))
    frames.append(dict(t=tabs - T0, tabs=tabs, cl=cl, vis=v, lost=l, stale=s,
                       det=det, gap=gap, held=held, raw=raw,
                       vis0=v0, lost0=l0, det0=det0))
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

tot=fps_wall+fps_keep
print(f'\n=== 정적 지도 필터 효과 ===')
print(f'  라이다만: 사람 판정 {tot}건')
print(f'  그 중 지도상 벽 자리 = {fps_wall}건 ({100*fps_wall/max(tot,1):.1f}%)  → 배제')
print(f'  남는 사람 판정        = {fps_keep}건')
pickle.dump(dict(frames=frames, T0=T0, MT0=_MT0, wall_pad=WALL_PAD,
                 params={k:sb.get(k) for k in ('detect_range','cluster_max_width','min_points',
                                               'lost_sec','seen_sec','edge_margin','cone_half_deg')}),
            open(f'{WORK}/{TAG}_cluster.pkl', 'wb'))
print(f'\n저장: {TAG}_cluster.pkl')
