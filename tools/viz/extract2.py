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
out['T0']=T0        # 절대 기준시각 — 다른 도구가 같은 시각계로 맞출 수 있게

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

# ── costmap : 반드시 *_raw 를 쓴다 ─────────────────────────────
#   /local_costmap/costmap (OccupancyGrid) 은 rolling window 의 **원점이
#   움직일 때만** 발행된다 (always_send_full_costmap 기본값 false).
#   realtake6 실측: 원점 변경 192회 ↔ full 발행 193회, 매번 0.002초 이내 일치.
#   → 로봇이 제자리면 발행이 멈춘다. SCAN_AREA(제자리 360°)에서 최대 46.6초
#     공백, 그 구간 평균 나이 24초. 사람이 도착해도 costmap 에 안 나타난다.
#   costmap_raw 는 매 주기 무조건 발행된다 — 실측 최대 공백 1.20초.
#   값 규격은 raw(0 자유 · 1~252 완충 · 253 준치명 · 254 치명 · 255 미지)로
#   통일한다. OccupancyGrid 로 되돌아갈 때만 0~100 → 0~254 로 환산한다.
LETHAL, INSCRIBED, NO_INFO = 254, 253, 255

def _raw_costmaps(topic):
    out=[]
    for _bt,m in raw(topic):
        md=m.metadata
        g=np.frombuffer(bytes(m.data),dtype=np.uint8).reshape(md.size_y,md.size_x).copy()
        out.append((stamp(m.header)-T0, g, md.resolution,
                    md.origin.position.x, md.origin.position.y))
    return out

def _occ_to_raw(o):                      # OccupancyGrid(0~100, -1) → raw(0~254, 255)
    o=np.asarray(o,dtype=np.int16)
    g=np.zeros(o.shape,np.uint8)
    g[o<0]        = NO_INFO
    g[o==100]     = LETHAL
    g[o==99]      = INSCRIBED
    mid=(o>=1)&(o<=98)
    g[mid]=(1+(o[mid].astype(np.int32)-1)*251//97).astype(np.uint8)
    return g

def costmaps(ns):
    r=_raw_costmaps(f'/{ns}/costmap_raw')
    if r:
        return r,'raw'
    print(f'  ⚠ /{ns}/costmap_raw 가 없다 → /{ns}/costmap 으로 되돌아간다.'
          f' 제자리 구간에 공백이 생긴다(realtake6 실측 최대 46.6초).')
    return [(stamp(m.header)-T0,
             _occ_to_raw(np.array(m.data,dtype=np.int8).reshape(m.info.height,m.info.width)),
             m.info.resolution,m.info.origin.position.x,m.info.origin.position.y)
            for _bt,m in raw(f'/{ns}/costmap')],'occ'

def _dedupe(cms):                        # 내용이 안 바뀐 연속 발행은 버린다 (pkl 용량)
    out=[]; prev=None
    for c in cms:
        if prev is not None and c[1].shape==prev.shape and np.array_equal(c[1],prev) \
           and c[3]==out[-1][3] and c[4]==out[-1][4]:
            continue
        out.append(c); prev=c[1]
    return out

out['lcost'],out['lcost_src']=costmaps('local_costmap')
_g,out['gcost_src']=costmaps('global_costmap')
out['gcost']=_dedupe(_g)                 # 전역은 map 프레임 · 거의 안 변한다

# ── mission_state / siren : header 없음 → rosout 지연으로 보정 ─
lags=[bt-(m.stamp.sec+m.stamp.nanosec/1e9) for bt,m in raw('/rosout') if m.name=='mission_manager']
LAG=float(np.median(lags)) if lags else 0.0
print(f'  mission_manager 기록 지연 보정 = {LAG:+.3f}s')
ms=[]; p=None; ms_all=[]
for bt,m in raw('/mission_state'):
    ms_all.append((bt-LAG-T0, m.data))          # echo 재생용 — 전량
    if m.data!=p: ms.append((bt-LAG-T0,m.data)); p=m.data
out['state']=ms
out['state_raw']=ms_all                          # ros2 topic echo 재현용
out['mt0']=ms[0][0] if ms else 0.0       # 미션 시작(첫 상태) 시각 = 화면의 T+0
sir=[]; p=None
for bt,m in raw('/siren'):
    if m.data!=p: sir.append((bt-LAG-T0,m.data)); p=m.data
out['siren']=sir

# 🔴 /alarm 은 header.stamp 를 안 채운다 (realtake6 실측: 3건 전부 0.0).
#    /mission_state·/siren 과 같은 처리 — 기록시각에서 지연을 뺀다.
#    예전 코드가 화재 등장 시각 69.0 을 손으로 박아둔 이유가 이것이고,
#    그래서 realtake5(알람 시각이 다르다)로는 못 돌렸다.
_al=[]; _src='header.stamp'
for bt,m in raw('/alarm'):
    st=m.header.stamp.sec+m.header.stamp.nanosec/1e9
    if st<=0: _src='기록시각 보정'
    _al.append(((st-T0) if st>0 else (bt-LAG-T0), m.pose.position.x, m.pose.position.y))
out['alarm']=_al[:1]
if _al: print(f'  화재 알람 t={_al[0][0]:.1f}s @ ({_al[0][1]:.2f}, {_al[0][2]:.2f}) [{_src}]')
out['tmax']=pose[-1][0]
out['log']=[((m.stamp.sec+m.stamp.nanosec/1e9)-T0, m.msg) for bt,m in raw('/rosout') if m.name=='mission_manager']

pickle.dump(out,open(f'{S}/{TAG}.pkl','wb'))
def _gapinfo(cms):
    if len(cms)<2: return 'n/a'
    d=np.diff([c[0] for c in cms]); return f'최대공백 {d.max():.2f}s'
print(f'  pose {len(pose)} · scan {len(scans)} · plan {len(out["plan"])}')
print(f'  lcost {len(out["lcost"]):4d} [{out["lcost_src"]}] {_gapinfo(out["lcost"])}'
      f'  · gcost {len(out["gcost"]):4d} [{out["gcost_src"]}] (중복 제거 후)')
print('  상태:', ' → '.join(f'{n}({t:.0f})' for t,n in ms))
print(f'  tmax {out["tmax"]:.1f}s')
