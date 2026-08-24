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
D=pickle.load(open(f'{S}/{TAG}.pkl','rb'))

# ---- 화면 ----
W,H = 1600,1000
MX,MY,MW,MH = 20,64,1060,916          # 지도 패널
X0,X1,Y0,Y1 = -1.5,14.0,-12.0,1.5     # 🔴 밟은 범위로 자름
SC = min(MW/(X1-X0), MH/(Y1-Y0))
MW2,MH2 = int((X1-X0)*SC), int((Y1-Y0)*SC)

FIRE_DY = 0.45      # 화재 마커를 지도상 위로 올리는 양 [m]
C_BG=(16,20,26); C_UNK=(150,161,175); C_FREE=(244,247,250); C_OCC=(20,26,34)
C_SCAN=(255,64,52); C_PLAN=(0,224,164); C_TRAIL=(110,168,255)
C_ROB=(255,214,10); C_FIRE=(255,138,0); C_PANEL=(22,27,34)

F=lambda s,b=False: ImageFont.truetype(
    f'/usr/share/fonts/opentype/noto/NotoSansCJK-{"Bold" if b else "Regular"}.ttc', s, index=1)
f_big,f_mid,f_sm,f_xs,f_ttl = F(46,True),F(24,True),F(19),F(16),F(26,True)

def w2p(x,y):   # map 좌표 → 패널 픽셀
    return (x-X0)*SC, (Y1-y)*SC

# ---- 지도 배경 ----
mp=D['map']; g=mp['g']; res=mp['res']; ox=mp['oy'],; oxx=mp['ox']; oyy=mp['oy']
ix0=int((X0-oxx)/res); ix1=int((X1-oxx)/res)
iy0=int((Y0-oyy)/res); iy1=int((Y1-oyy)/res)
sub=g[iy0:iy1, ix0:ix1][::-1]                    # y 뒤집기
base=np.zeros((sub.shape[0],sub.shape[1],3),np.uint8)
base[:]=C_UNK
base[sub==0]=C_FREE
base[sub>=50]=C_OCC
bg=np.array(Image.fromarray(base).resize((MW2,MH2), Image.NEAREST))

pose=D['pose']; pt=pose[:,0]
scans=D['scan']; st=[s[0] for s in scans]
plans=D['plan']; pl=[p[0] for p in plans]
lc=D['lcost'];  lt=[c[0] for c in lc]
m2o=D['m2o']; mot=m2o[:,0]
states=D['state']; sir=D['siren']
MEAN={'PATROL':'평시 순찰','APPROACH':'화재 감지 — 집결지로 출동','SCAN_AREA':'집결지 360° 훑기 — 대피자 탐색',
      'GATHER':'집결 대기','GUIDE':'저속 선행 유도 — 후방 감시','HOLD':'추종자 놓침 — 제자리 재수집',
      'SEARCH_BACK':'역행 재탐색','ESCAPED':'탈출 완료'}
def pick(arr,tl,t):
    i=bisect.bisect_right(tl,t)-1
    return arr[i] if i>=0 else None

T_A   = float(os.environ.get('VIZ_T0', 10.0))
T_B   = float(os.environ.get('VIZ_T1', 326.0))
SPEED = float(os.environ.get('VIZ_SPEED', 2.0))
FPS   = int(os.environ.get('VIZ_FPS', 20))
NAME  = os.environ.get('VIZ_NAME', 'overview')
step=SPEED/FPS
frames=int((T_B-T_A)/step)
print(f'{frames} 프레임 · {frames/FPS:.0f}초 영상 · {SPEED}배속', flush=True)

ff=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
    '-s',f'{W}x{H}','-r',str(FPS),'-i','-','-c:v','libx264','-pix_fmt','yuv420p','-crf','20',
    f'{S}/{NAME}.mp4'], stdin=subprocess.PIPE)

trail=[]
for k in range(frames):
    t=T_A+k*step
    pi=min(max(bisect.bisect_right(pt,t)-1,0),len(pose)-1)
    _,rx,ry,ra=pose[pi]
    trail.append((rx,ry))

    img=bg.copy()
    # --- local costmap ---
    c=pick(lc,lt,t)
    if c is not None:
        _,cg,cr,cox,coy=c
        yy,xx=np.nonzero(cg>0)
        if len(xx):
            ux=cox+(xx+0.5)*cr; uy=coy+(yy+0.5)*cr
            mi=min(max(bisect.bisect_right(mot,c[0])-1,0),len(m2o)-1)
            _,tx,ty,ta=m2o[mi]; ca,sa=math.cos(ta),math.sin(ta)
            wx=tx+ca*ux-sa*uy; wy=ty+sa*ux+ca*uy
            px=((wx-X0)*SC).astype(int); py=((Y1-wy)*SC).astype(int)
            ok=(px>=0)&(px<MW2)&(py>=0)&(py<MH2)
            px,py=px[ok],py[ok]; v=cg[yy,xx][ok].astype(np.float32)
            a=np.clip(0.28+v/100.0*0.52,0.28,0.80)[:,None]
            col=np.where(v[:,None]>=99,np.array([236,52,28]),np.array([255,176,52]))
            for dx in (0,1):
                for dy in (0,1):
                    qx=np.clip(px+dx,0,MW2-1); qy=np.clip(py+dy,0,MH2-1)
                    img[qy,qx]=(img[qy,qx]*(1-a)+col*a).astype(np.uint8)
    # --- scan ---
    s=pick(scans,st,t)
    if s is not None and t-s[0]<1.0:
        px=((s[1]-X0)*SC).astype(int); py=((Y1-s[2])*SC).astype(int)
        ok=(px>=0)&(px<MW2-1)&(py>=0)&(py<MH2-1)
        px,py=px[ok],py[ok]
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                qx=np.clip(px+dx,0,MW2-1); qy=np.clip(py+dy,0,MH2-1)
                img[qy,qx]=C_SCAN

    im=Image.fromarray(img); dr=ImageDraw.Draw(im)
    # --- 지나온 경로 ---
    if len(trail)>2:
        dr.line([w2p(*p) for p in trail[::2]],fill=C_TRAIL,width=3)
    # --- 현재 경로 ---
    p=pick(plans,pl,t)
    if p is not None and t-p[0]<6 and len(p[1])>1:
        dr.line([w2p(x,y) for x,y in p[1][::3]],fill=C_PLAN,width=5)
    # --- 화재 ---
    for _,ax,ay in D['alarm']:
        if t>69.0:
            fx,fy=w2p(ax,ay+FIRE_DY); dr.ellipse([fx-13,fy-13,fx+13,fy+13],fill=C_FIRE)
            dr.ellipse([fx-19,fy-19,fx+19,fy+19],outline=C_FIRE,width=3)
            tw=dr.textlength('화재 지점',font=f_sm)
            dr.text((fx-28-tw,fy-11),'화재 지점',font=f_sm,fill=C_FIRE)
    # --- 로봇 ---
    cx,cy=w2p(rx,ry); R=0.40*SC
    dr.ellipse([cx-R,cy-R,cx+R,cy+R],outline=C_ROB,width=3)
    L=0.55*SC
    tri=[(cx+L*math.cos(-ra),cy+L*math.sin(-ra)),
         (cx+0.45*L*math.cos(-ra+2.5),cy+0.45*L*math.sin(-ra+2.5)),
         (cx+0.45*L*math.cos(-ra-2.5),cy+0.45*L*math.sin(-ra-2.5))]
    dr.polygon(tri,fill=C_ROB)

    # --- 캔버스 ---
    cv=Image.new('RGB',(W,H),C_BG); cd=ImageDraw.Draw(cv)
    cv.paste(im,(MX,MY))
    cd.rectangle([MX-1,MY-1,MX+MW2,MY+MH2],outline=(60,70,84),width=1)
    cd.text((MX,20),'한이음 지하터널 대피로봇 — 실차 자율주행 (2026-08-23)',font=f_ttl,fill=(235,240,245))

    # --- 우측 패널 ---
    PX=MX+MW2+24; PW=W-PX-20
    cd.rectangle([PX,MY,PX+PW,MY+MH2],fill=C_PANEL,outline=(50,58,70))
    stt=pick(states,[s[0] for s in states],t)
    name=stt[1] if stt else '-'
    cd.text((PX+24,MY+26),'MISSION STATE',font=f_xs,fill=(130,145,165))
    cd.text((PX+24,MY+52),name,font=f_big,fill=(0,224,164) if name!='HOLD' else (255,214,10))
    cd.text((PX+24,MY+112),MEAN.get(name,''),font=f_sm,fill=(200,212,225))
    cd.text((PX+24,MY+156),f'T + {t-13.0:6.1f} s',font=f_mid,fill=(150,165,185))
    sv=pick(sir,[s[0] for s in sir],t)
    if sv and sv[1]:
        cd.rectangle([PX+24,MY+196,PX+150,MY+232],fill=(200,40,30))
        cd.text((PX+40,MY+202),'SIREN ON',font=f_sm,fill=(255,255,255))

    ly=MY+264
    cd.text((PX+24,ly),'화면 요소',font=f_xs,fill=(130,145,165)); ly+=30
    for col,lab in [(C_SCAN,'라이다 실시간 점군 (/scan)'),
                    ((255,120,50),'코스트맵 — 장애물 인지'),
                    (C_PLAN,'현재 계획 경로 (/plan)'),
                    (C_TRAIL,'지나온 궤적'),
                    (C_ROB,'로봇 현재 위치·방향')]:
        cd.rectangle([PX+24,ly+6,PX+44,ly+18],fill=col)
        cd.text((PX+54,ly),lab,font=f_xs,fill=(205,215,228)); ly+=28

    ly+=18
    cd.text((PX+24,ly),'상태 진행',font=f_xs,fill=(130,145,165)); ly+=28
    for ts_,nm in states:
        done = t>=ts_
        cd.text((PX+24,ly),('●' if done else '○')+f'  {ts_-13.0:6.1f}s  {nm}',
                font=f_xs,fill=(0,224,164) if done else (95,108,124)); ly+=25

    ff.stdin.write(np.asarray(cv).tobytes())
    if k%400==0: print(f'  {k}/{frames}',flush=True)

ff.stdin.close(); ff.wait()
print('완료:', f'{S}/{NAME}.mp4')
