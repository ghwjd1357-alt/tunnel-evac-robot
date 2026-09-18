# -*- coding: utf-8 -*-
"""관제화면 렌더러 — bag 에서 직접 그린다 (화면 녹화 아님).

08-25 개편:
  A  costmap 출처를 *_raw 로 (extract2.py). 제자리 구간 46.6초 공백 해소
  B  셀 크기만큼 칠한다 — 2x2 px 점찍기(방충망) 폐기
  C  지도 배경 슬라이스를 round() 로 — x축 한 셀(0.05 m) 어긋남 해소
  D  치명 costmap 과 /scan 의 색을 분리
  E  최저 알파 0.28 → 0.05. 완충구역이 창 전체를 덮지 않게
  F  중복 알파 합성 제거 (한 픽셀 = 한 번)
  G  화재 시각·미션 T0 하드코딩 제거 — bag 의 실제 값을 쓴다
     (🔴 /alarm 은 header.stamp 가 0 이라 extract2.py 가 기록시각으로 보정한다)
  +  costmap 을 map 으로 옮길 때 map→odom 을 **렌더 시각 t** 로 조회한다
  08-25b  제목 삭제 · 우측 패널 정리 · 지도 배경은 전역 costmap 없이 (VIZ_GLOBAL=1 로 켬)
  08-26   VIZ_LAYOUT=map 추가 — 패널 없는 지도전용 소스 (편집부 납품용)
"""
import os
BAGS = os.environ.get('VIZ_BAGS', os.path.expanduser('~/robot_evidence'))
WORK = os.environ.get('VIZ_WORK', os.path.join(BAGS, 'viz', '_work'))
REPO = os.environ.get('VIZ_REPO', os.path.expanduser('~/ros2_ws'))
TAG  = os.environ.get('VIZ_TAG', 'realtake6')
os.makedirs(WORK, exist_ok=True)
import pickle, bisect, math, subprocess, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont

S = WORK
D = pickle.load(open(f'{S}/{TAG}.pkl','rb'))

# ---- 화면 ----
#  VIZ_LAYOUT=panel : 지도 + 우측 패널 (1600x1000)  ← 기본. v4 와 동일
#  VIZ_LAYOUT=map   : 지도만. 편집부에 넘길 깨끗한 소스 (1240x1080, 여백·글씨 없음)
LAYOUT = os.environ.get('VIZ_LAYOUT','panel')
X0,X1,Y0,Y1 = -1.5,14.0,-12.0,1.5
if LAYOUT == 'map':
    SC = 80.0                                   # 15.5 x 13.5 m → 1240 x 1080 (둘 다 짝수)
    MW2,MH2 = int((X1-X0)*SC), int((Y1-Y0)*SC)
    W,H, MX,MY = MW2,MH2, 0,0
else:
    W,H = 1600,1000
    MX,MW,MH = 20,1060,960                      # 지도 패널 (제목이 없어 세로를 다 쓴다)
    SC = min(MW/(X1-X0), MH/(Y1-Y0))
    MW2,MH2 = int((X1-X0)*SC), int((Y1-Y0)*SC)
    MY = (H-MH2)//2

# ---- 색 ----
C_BG=(16,20,26); C_UNK=(150,161,175); C_FREE=(246,249,252); C_OCC=(20,26,34)
C_PLAN=(0,224,164); C_TRAIL=(110,168,255)
C_ROB=(255,214,10); C_FIRE=(255,138,0); C_PANEL=(22,27,34)
C_LINE=(46,54,66); C_DIM=(122,136,155); C_TXT=(206,216,228)
C_COST_INFL=(255,176,52)      # 완충 (1~252)
C_COST_INSC=(240,106,36)      # 준치명 253 — 로봇 내접반경 안
C_COST_LETH=(206,38,38)       # 치명 254
C_GCOST    =(104,120,190)     # 전역 costmap (기본 꺼짐)
C_SCAN = (255,64,52) if os.environ.get('VIZ_SCAN')=='red' else (34,214,242)

USE_GLOBAL   = os.environ.get('VIZ_GLOBAL')=='1'
COST_MAX_AGE = float(os.environ.get('VIZ_COST_MAX_AGE', 3.0))
FIRE_LABEL   = os.environ.get('VIZ_FIRE_LABEL','1')!='0'

# 🔴 화재 마커 **표시** 오프셋. 알람 실좌표 (12.50,-0.10) 은 안 바꾼다 — 화면에서만 옮긴다.
#    기준: FollowerMonitor 가 person=True 로 판정한 클러스터의 GATHER 구간 위치
#          = (11.09, +0.02)  (폭 0.40~0.50 m · 51~64점 · 전 구간 안정).
#    그 0.45 m 위에 놓는다 → (11.09, +0.47).
FIRE_DX = float(os.environ.get('VIZ_FIRE_DX', -1.41))
FIRE_DY = float(os.environ.get('VIZ_FIRE_DY',  0.57))

FT='/usr/share/fonts/opentype/noto/NotoSansCJK-{}.ttc'
F=lambda s,b=False: ImageFont.truetype(FT.format('Bold' if b else 'Regular'), s, index=1)
f_state,f_mean,f_time,f_lab,f_row = F(52,True),F(20),F(28,True),F(14),F(17)

def w2p(x,y):
    return (x-X0)*SC, (Y1-y)*SC

# ---- 지도 배경 ----
mp=D['map']; g=mp['g']; res=mp['res']; oxx=mp['ox']; oyy=mp['oy']
ix0=int(round((X0-oxx)/res)); ix1=int(round((X1-oxx)/res))
iy0=int(round((Y0-oyy)/res)); iy1=int(round((Y1-oyy)/res))
gh,gw=g.shape
sub=np.full((iy1-iy0, ix1-ix0), -1, np.int8)
sy0,sy1=max(iy0,0),min(iy1,gh); sx0,sx1=max(ix0,0),min(ix1,gw)
if sy1>sy0 and sx1>sx0:
    sub[sy0-iy0:sy1-iy0, sx0-ix0:sx1-ix0]=g[sy0:sy1, sx0:sx1]
sub=sub[::-1]
base=np.zeros((sub.shape[0],sub.shape[1],3),np.uint8)
base[:]=C_UNK; base[sub==0]=C_FREE; base[sub>=50]=C_OCC
BG=np.array(Image.fromarray(base).resize((MW2,MH2), Image.NEAREST))

pose=D['pose']; pt=pose[:,0]
scans=D['scan']; st=[s[0] for s in scans]
plans=D['plan']; pl=[p[0] for p in plans]
lc=D['lcost'];   lt=[c[0] for c in lc]
gc=D.get('gcost',[]) if USE_GLOBAL else []; gt=[c[0] for c in gc]
m2o=D['m2o']; mot=m2o[:,0]
states=D['state']; sir=D['siren']; sirt=[s[0] for s in sir]; stt_t=[s[0] for s in states]
MT0=D.get('mt0', states[0][0] if states else 0.0)
ALARM=D['alarm'][0] if D.get('alarm') else None

MEAN={'PATROL':'평시 순찰','APPROACH':'화재 감지, 집결지로 출동','SCAN_AREA':'집결지 360° 훑기 · 대피자 탐색',
      'GATHER':'집결 대기','GUIDE':'저속 선행 유도 · 후방 감시','HOLD':'추종자 놓침, 제자리 재수집',
      'SEARCH_BACK':'역행 재탐색','ESCAPED':'탈출 완료'}
def pick(arr,tl,t):
    i=bisect.bisect_right(tl,t)-1
    return arr[i] if i>=0 else None
def idx(tl,t,n):
    return min(max(bisect.bisect_right(tl,t)-1,0),n-1)

# ---- costmap ----
def pal_local(v):
    rgb=np.empty((v.size,3),np.float32); rgb[:]=C_COST_INFL
    rgb[v>=253]=C_COST_INSC
    rgb[v>=254]=C_COST_LETH
    a=np.where(v>=254,0.74,np.where(v>=253,0.56,0.05+(v/252.0)*0.33))
    return rgb, a.astype(np.float32)

def pal_global(v):
    rgb=np.empty((v.size,3),np.float32); rgb[:]=C_GCOST
    a=np.where(v>=253,0.22,0.0)
    return rgb, a.astype(np.float32)

def paint(img, cm, tx,ty,ta, pal, dim=1.0):
    """패널 픽셀 -> 셀 역변환. 구멍도, 중복 알파 합성도 없다."""
    _,cg,cr,cox,coy = cm
    h,wd = cg.shape
    ca,sa = math.cos(ta), math.sin(ta)
    us=[];vs=[]
    for ux in (cox, cox+wd*cr):
        for uy in (coy, coy+h*cr):
            wx=tx+ca*ux-sa*uy; wy=ty+sa*ux+ca*uy
            us.append((wx-X0)*SC); vs.append((Y1-wy)*SC)
    u0=max(int(math.floor(min(us))),0); u1=min(int(math.ceil(max(us)))+1,MW2)
    v0=max(int(math.floor(min(vs))),0); v1=min(int(math.ceil(max(vs)))+1,MH2)
    if u1<=u0 or v1<=v0: return
    uu,vv=np.meshgrid(np.arange(u0,u1,dtype=np.float32),
                      np.arange(v0,v1,dtype=np.float32))
    dx=(X0+(uu+0.5)/SC)-tx; dy=(Y1-(vv+0.5)/SC)-ty
    ux= ca*dx+sa*dy; uy=-sa*dx+ca*dy
    ci=np.floor((ux-cox)/cr).astype(np.int32)
    ri=np.floor((uy-coy)/cr).astype(np.int32)
    ok=(ci>=0)&(ci<wd)&(ri>=0)&(ri<h)
    if not ok.any(): return
    val=np.zeros(ok.shape,np.uint8)
    val[ok]=cg[ri[ok],ci[ok]]
    m=ok&(val>0)&(val<255)
    if not m.any(): return
    rgb,a=pal(val[m].astype(np.float32))
    a=(a*dim)[:,None]
    tgt=img[v0:v1,u0:u1]
    tgt[m]=(tgt[m]*(1-a)+rgb*a).astype(np.uint8)

# ---- 범례용 그라데이션 막대 ----
def _lerp(a,b,f): return tuple(int(a[i]+(b[i]-a[i])*f) for i in range(3))
GBW,GBH=96,12
_gb=Image.new('RGB',(GBW,GBH))
for i in range(GBW):
    f=i/(GBW-1)
    _gb.paste(_lerp(C_COST_INFL,C_COST_INSC,f/0.55) if f<0.55
              else _lerp(C_COST_INSC,C_COST_LETH,(f-0.55)/0.45), (i,0,i+1,GBH))

T_A   = float(os.environ.get('VIZ_T0', 10.0))
T_B   = float(os.environ.get('VIZ_T1', 326.0))
SPEED = float(os.environ.get('VIZ_SPEED', 2.0))
FPS   = int(os.environ.get('VIZ_FPS', 20))
NAME  = os.environ.get('VIZ_NAME', 'overview')
step=SPEED/FPS
frames=int((T_B-T_A)/step)
print(f'{frames} 프레임 · {frames/FPS:.0f}초 · {SPEED}배속 · {W}x{H} [{LAYOUT}] · '
      f'costmap local[{D.get("lcost_src","?")}] · 전역깔기={USE_GLOBAL}', flush=True)

ff=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
    '-s',f'{W}x{H}','-r',str(FPS),'-i','-','-c:v','libx264','-pix_fmt','yuv420p','-crf','20',
    f'{S}/{NAME}.mp4'], stdin=subprocess.PIPE)

PX = MX+MW2+24; PW = W-PX-20
_gi=-1; _gbg=BG
stale_frames=0
trail=[]
for k in range(frames):
    t=T_A+k*step
    pi=idx(pt,t,len(pose)); _,rx,ry,ra=pose[pi]
    trail.append((rx,ry))
    mi=idx(mot,t,len(m2o)); _,tx,ty,ta=m2o[mi]

    if gc:
        j=idx(gt,t,len(gc))
        if j!=_gi:
            _gi=j; _gbg=BG.copy(); paint(_gbg, gc[j], 0.0,0.0,0.0, pal_global)
    img=_gbg.copy()

    c=pick(lc,lt,t); cost_age=float('nan')
    if c is not None:
        cost_age=t-c[0]; stale=cost_age>COST_MAX_AGE; stale_frames+=stale
        paint(img, c, tx,ty,ta, pal_local, dim=0.45 if stale else 1.0)

    s=pick(scans,st,t)
    if s is not None and t-s[0]<1.0:
        px=((s[1]-X0)*SC).astype(int); py=((Y1-s[2])*SC).astype(int)
        ok=(px>=0)&(px<MW2-1)&(py>=0)&(py<MH2-1)
        px,py=px[ok],py[ok]
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                img[np.clip(py+dy,0,MH2-1),np.clip(px+dx,0,MW2-1)]=C_SCAN

    im=Image.fromarray(img); dr=ImageDraw.Draw(im)
    if len(trail)>2:
        dr.line([w2p(*p) for p in trail[::2]],fill=C_TRAIL,width=3)
    p=pick(plans,pl,t)
    if p is not None and t-p[0]<6 and len(p[1])>1:
        dr.line([w2p(x,y) for x,y in p[1][::3]],fill=C_PLAN,width=5)
    if ALARM is not None and t>=ALARM[0]:
        fx,fy=w2p(ALARM[1]+FIRE_DX, ALARM[2]+FIRE_DY)
        dr.ellipse([fx-19,fy-19,fx+19,fy+19],outline=C_FIRE,width=3)
        dr.ellipse([fx-13,fy-13,fx+13,fy+13],fill=C_FIRE)
        if FIRE_LABEL:
            lab='화재 지점'; tw=dr.textlength(lab,font=f_row)
            lx,ly_=fx+34, fy-14             # 마커 오른쪽 (왼쪽엔 로봇이 온다)
            dr.line([fx+16,fy,lx-10,ly_+10],fill=C_FIRE,width=2)
            dr.rounded_rectangle([lx-10,ly_-6,lx+tw+10,ly_+26],6,fill=(26,18,12))
            dr.text((lx,ly_),lab,font=f_row,fill=C_FIRE)
    cx,cy=w2p(rx,ry); R=0.40*SC
    dr.ellipse([cx-R,cy-R,cx+R,cy+R],outline=C_ROB,width=3)
    L=0.55*SC
    dr.polygon([(cx+L*math.cos(-ra),cy+L*math.sin(-ra)),
                (cx+0.45*L*math.cos(-ra+2.5),cy+0.45*L*math.sin(-ra+2.5)),
                (cx+0.45*L*math.cos(-ra-2.5),cy+0.45*L*math.sin(-ra-2.5))],fill=C_ROB)

    # ---- 캔버스 ----
    if LAYOUT=='map':
        cv=im                                   # 지도만. 테두리·패널 없음
    else:
        cv=Image.new('RGB',(W,H),C_BG); cd=ImageDraw.Draw(cv)
        cv.paste(im,(MX,MY))
        cd.rectangle([MX-1,MY-1,MX+MW2,MY+MH2],outline=C_LINE,width=1)
        cd.rectangle([PX,MY,PX+PW,MY+MH2],fill=C_PANEL,outline=C_LINE)

        name=(pick(states,stt_t,t) or (0,'-'))[1]
        y=MY+46
        cd.text((PX+28,y),name,font=f_state,fill=(0,224,164) if name!='HOLD' else C_ROB); y+=66
        cd.text((PX+30,y),MEAN.get(name,''),font=f_mean,fill=C_DIM); y+=52
        cd.text((PX+30,y),f'T + {t-MT0:.1f} s',font=f_time,fill=C_TXT)
        sv=pick(sir,sirt,t)
        if sv and sv[1]:
            bx=PX+PW-132
            cd.rounded_rectangle([bx,y-1,bx+104,y+33],5,fill=(176,38,30))
            cd.ellipse([bx+14,y+12,bx+24,y+22],fill=(255,226,220))
            cd.text((bx+32,y+7),'사이렌',font=f_row,fill=(255,236,232))
        y+=64
        cd.line([PX+30,y,PX+PW-30,y],fill=C_LINE,width=1); y+=30

        cd.text((PX+30,y),'범례',font=f_lab,fill=(96,108,124)); y+=28
        for col,lab in ([(C_SCAN,'라이다 점군')] +
                        ([(C_GCOST,'전역 코스트맵')] if USE_GLOBAL else []) +
                        [(None,'코스트맵'),(C_PLAN,'계획 경로'),(C_TRAIL,'주행 궤적'),(C_ROB,'로봇')]):
            if col is None:
                cv.paste(_gb,(PX+30,y+5))
                cd.text((PX+30+GBW+14,y),lab,font=f_row,fill=C_TXT)
                cd.text((PX+30,y+22),'완충',font=f_lab,fill=(96,108,124))
                tw=cd.textlength('치명',font=f_lab)
                cd.text((PX+30+GBW-tw,y+22),'치명',font=f_lab,fill=(96,108,124))
                y+=44
            else:
                cd.rectangle([PX+30,y+6,PX+30+GBW,y+18],fill=col)
                cd.text((PX+30+GBW+14,y),lab,font=f_row,fill=C_TXT); y+=30
        if c is not None and cost_age>COST_MAX_AGE:
            cd.text((PX+30,y),f'코스트맵 {cost_age:.0f}초 전',font=f_lab,fill=(226,164,58)); y+=24

        y+=14
        cd.line([PX+30,y,PX+PW-30,y],fill=C_LINE,width=1); y+=30
        cd.text((PX+30,y),'진행',font=f_lab,fill=(96,108,124)); y+=30
        for n,(ts_,nm) in enumerate(states):
            done = t>=ts_
            col=(0,224,164) if done else (78,90,106)
            cyc=y+9
            if n: cd.line([PX+38,cyc-28,PX+38,cyc-8],fill=C_LINE,width=2)
            if done: cd.ellipse([PX+33,cyc-5,PX+43,cyc+5],fill=col)
            else:    cd.ellipse([PX+33,cyc-5,PX+43,cyc+5],outline=col,width=2)
            cd.text((PX+58,y),f'{ts_-MT0:6.1f}s',font=f_row,fill=col)
            cd.text((PX+140,y),nm,font=f_row,fill=col)
            y+=28

    ff.stdin.write(np.asarray(cv).tobytes())
    if k%400==0: print(f'  {k}/{frames}',flush=True)

ff.stdin.close(); ff.wait()
print(f'  costmap {COST_MAX_AGE:.0f}초 초과 프레임 = {stale_frames}/{frames}')
print('완료:', f'{S}/{NAME}.mp4')
