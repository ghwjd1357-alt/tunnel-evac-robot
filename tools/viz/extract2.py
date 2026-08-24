import os
BAGS = os.environ.get('VIZ_BAGS', os.path.expanduser('~/robot_evidence'))
WORK = os.environ.get('VIZ_WORK', os.path.join(BAGS, 'viz', '_work'))
REPO = os.environ.get('VIZ_REPO', os.path.expanduser('~/ros2_ws'))
TAG  = os.environ.get('VIZ_TAG', 'realtake6')
os.makedirs(WORK, exist_ok=True)
import sqlite3, math, pickle, sys, glob
import numpy as np
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

TAG = sys.argv[1] if len(sys.argv) > 1 else TAG
S = WORK
DB=glob.glob(f'{BAGS}/{TAG}/*.db3')[0]
con=sqlite3.connect(DB)
tp={r[0]:(r[1],r[2]) for r in con.execute('select id,name,type from topics')}
n2i={v[0]:k for k,v in tp.items()}
def raw(t):
    if t not in n2i: return
    tid=n2i[t]; mt=get_message(tp[tid][1])
    for ts,d in con.execute('select timestamp,data from messages where topic_id=? order by timestamp',(tid,)):
        yield ts/1e9, deserialize_message(bytes(d),mt)
def stamp(h): return h.stamp.sec + h.stamp.nanosec/1e9
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

# ── pose : TF header.stamp 기준 ──────────────────────────────
mo=ob=None; mo_t=ob_t=None; pose=[]; m2o=[]
for _bt,m in raw('/tf'):
    for tr in m.transforms:
        f,c=tr.header.frame_id,tr.child_frame_id
        if f=='map' and c=='odom':   mo,mo_t = tr.transform, stamp(tr.header)
        elif f=='odom' and c=='base_footprint': ob,ob_t = tr.transform, stamp(tr.header)
    if mo is not None and ob is not None:
        t=ob_t                                   # 로봇 자세의 시각 = odom→base 의 stamp
        a=yaw(mo.rotation)
        x=mo.translation.x+math.cos(a)*ob.translation.x-math.sin(a)*ob.translation.y
        y=mo.translation.y+math.sin(a)*ob.translation.x+math.cos(a)*ob.translation.y
        pose.append((t,x,y,a+yaw(ob.rotation)))
        m2o.append((t,mo.translation.x,mo.translation.y,a))
pose.sort(); m2o.sort()
T0=pose[0][0]
pose=[(t-T0,x,y,a) for t,x,y,a in pose]
m2o =[(t-T0,x,y,a) for t,x,y,a in m2o]
pt=[p[0] for p in pose]
import bisect
def pose_at(tt):
    i=min(max(bisect.bisect_right(pt,tt)-1,0),len(pose)-1); return pose[i]

out={}
out['pose']=np.array(pose); out['m2o']=np.array(m2o)

# ── map ──────────────────────────────────────────────────────
for _bt,m in raw('/map'):
    out['map']=dict(g=np.array(m.data,dtype=np.int8).reshape(m.info.height,m.info.width),
                    res=m.info.resolution, ox=m.info.origin.position.x, oy=m.info.origin.position.y)

# ── scan : header.stamp 로 시각, 그 시각의 자세로 map 변환 ────
scans=[]
for _bt,m in raw('/scan'):
    t=stamp(m.header)-T0
    _,px,py,pa=pose_at(t)
    r=np.array(m.ranges,dtype=np.float32)
    ang=m.angle_min+np.arange(len(r))*m.angle_increment
    ok=np.isfinite(r)&(r>m.range_min)&(r<min(m.range_max,12.0))
    r=r[ok]; ang=ang[ok]
    scans.append((t,(px+r*np.cos(ang+pa)).astype(np.float32),
                    (py+r*np.sin(ang+pa)).astype(np.float32)))
out['scan']=scans

# ── plan / local costmap : header.stamp ───────────────────────
out['plan']=[(stamp(m.header)-T0,
              np.array([[p.pose.position.x,p.pose.position.y] for p in m.poses],dtype=np.float32))
             for _bt,m in raw('/plan')]
out['lcost']=[(stamp(m.header)-T0,
               np.array(m.data,dtype=np.int8).reshape(m.info.height,m.info.width),
               m.info.resolution,m.info.origin.position.x,m.info.origin.position.y)
              for _bt,m in raw('/local_costmap/costmap')]
out['alarm']=[(stamp(m.header)-T0,m.pose.position.x,m.pose.position.y) for _bt,m in raw('/alarm')][:1]

# ── mission_state / siren : header 없음 → rosout 지연으로 보정 ─
lags=[bt-(m.stamp.sec+m.stamp.nanosec/1e9) for bt,m in raw('/rosout') if m.name=='mission_manager']
LAG=float(np.median(lags)) if lags else 0.0
print(f'  mission_manager 기록 지연 보정 = {LAG:+.3f}s')
ms=[]; p=None
for bt,m in raw('/mission_state'):
    if m.data!=p: ms.append((bt-LAG-T0,m.data)); p=m.data
out['state']=ms
sir=[]; p=None
for bt,m in raw('/siren'):
    if m.data!=p: sir.append((bt-LAG-T0,m.data)); p=m.data
out['siren']=sir
out['tmax']=pose[-1][0]
out['log']=[(bt-(bt-(m.stamp.sec+m.stamp.nanosec/1e9))-T0, m.msg) for bt,m in raw('/rosout') if m.name=='mission_manager']

pickle.dump(out,open(f'{S}/{TAG}.pkl','wb'))
print(f'  pose {len(pose)} · scan {len(scans)} · plan {len(out["plan"])} · lcost {len(out["lcost"])}')
print('  상태:', ' → '.join(f'{n}({t:.0f})' for t,n in ms))
print(f'  tmax {out["tmax"]:.1f}s')
