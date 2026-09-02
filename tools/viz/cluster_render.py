# -*- coding: utf-8 -*-
"""클러스터 판정 시각화 — 로봇 중심 확대 뷰.

VIZ_LAYOUT=panel : 뷰 + 우측 패널 (1920x1080)  ← 기본
VIZ_LAYOUT=view  : 뷰만. 편집부에 넘길 깨끗한 소스 (1080x1080, 패널·자막 없음)
"""
import os
BAGS = os.environ.get('VIZ_BAGS', os.path.expanduser('~/robot_evidence'))
WORK = os.environ.get('VIZ_WORK', os.path.join(BAGS, 'viz', '_work'))
REPO = os.environ.get('VIZ_REPO', os.path.expanduser('~/ros2_ws'))
TAG  = os.environ.get('VIZ_TAG', 'realtake6')
os.makedirs(WORK, exist_ok=True)
import pickle, math, bisect, subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont

S = WORK
D  = pickle.load(open(f'{S}/{TAG}_cluster.pkl','rb'))
MP = pickle.load(open(f'{S}/{TAG}.pkl','rb'))     # 미니맵·미션상태용
F  = D['frames']; FT=[f['t'] for f in F]

# 🔴 시각계 통일: cluster 는 첫 /scan, extract2 는 첫 tf 가 원점이라 서로 다르다.
#    화면 시각은 ①관제화면과 같은 **미션 T+** 로 통일한다 (예전엔 12.7초 어긋났다).
DT_EX    = D['T0'] - D['MT0']
MISSION0 = MP.get('mt0', 0.0)
def ex_t(t):  return t + DT_EX
def mis_t(t): return t + DT_EX - MISSION0

PRM = D.get('params', {})
P_RANGE = float(PRM.get('detect_range', 3.0))
P_WIDTH = float(PRM.get('cluster_max_width', 0.8))
P_MINPT = int(PRM.get('min_points', 3))
P_EDGE  = float(PRM.get('edge_margin', 0.2))
P_LOST  = float(PRM.get('lost_sec', 3.0))
P_SEEN  = float(PRM.get('seen_sec', 3.0))
WALL_PAD= float(D.get('wall_pad', 0.25))

LAYOUT = os.environ.get('VIZ_LAYOUT','panel')
RMAX   = 4.0                            # 화면 반경 [m]
if LAYOUT=='view':
    W=H=VW=1080; VX=VY=0
else:
    W,H = 1920,1080
    VX,VY,VW = 40,24,976
CX,CY = VX+VW//2, VY+VW//2
SC    = (VW/2)/RMAX
PX    = VX+VW+48; PW = W-PX-48

C_BG=(14,17,22); C_VIEW=(20,25,32); C_RING=(48,58,72)
C_ZONE=(38,86,120); C_RAW=(58,68,82)
C_OK=(0,224,140); C_NO=(126,136,150); C_ROB=(255,214,10); C_WALLFP=(176,92,196)
C_LOST=(255,72,58); C_TXT=(226,233,240); C_DIM=(138,150,166); C_LINE=(44,53,66)

NOTO='/usr/share/fonts/opentype/noto/NotoSansCJK-{}.ttc'
MONO='/usr/share/fonts/truetype/dejavu/DejaVuSansMono{}.ttf'
NF=lambda w,s: ImageFont.truetype(NOTO.format(w),s,index=1)
MF=lambda b,s: ImageFont.truetype(MONO.format('-Bold' if b else ''),s)
f_badge = NF('Black',76)
f_sub   = NF('Regular',22)
f_num   = MF(True,30)
f_big   = MF(True,60)
f_lab   = NF('Medium',19)
f_st    = NF('Black',40)
f_t     = MF(True,26)
f_xs    = NF('Regular',16)

ST=[(t,n) for t,n in MP['state']]
def state_at(t):
    n='-'
    for ts,nm in ST:
        if ts<=t: n=nm
        else: break
    return n
POSE=MP['pose']
def pose_at(t):
    i=max(np.searchsorted(POSE[:,0],t)-1,0); return POSE[i]

MX0,MX1,MY0,MY1 = -1.0,13.5,-11.8,1.0
def mini(x,y,ox,oy,w,h):
    return ox+(x-MX0)/(MX1-MX0)*w, oy+(MY1-y)/(MY1-MY0)*h

T_A  = float(os.environ.get('VIZ_T0', 162.0))
T_B  = float(os.environ.get('VIZ_T1', 296.0))
FPS  = int(os.environ.get('VIZ_FPS', 30))
NAME = os.environ.get('VIZ_NAME', 'cluster')
n=int((T_B-T_A)*FPS)
print(f'{n} 프레임 · {n/FPS:.0f}초 · 실속도 1배 · {W}x{H} [{LAYOUT}]',flush=True)
ff=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
    '-s',f'{W}x{H}','-r',str(FPS),'-i','-','-c:v','libx264','-pix_fmt','yuv420p','-crf','19',
    f'{S}/{NAME}.mp4'],stdin=subprocess.PIPE)

def bar(dr,x,y,w,h,frac,col,bgc=(40,48,60)):
    dr.rounded_rectangle([x,y,x+w,y+h],5,fill=bgc)
    if frac>0: dr.rounded_rectangle([x,y,x+max(8,int(w*min(frac,1.0))),y+h],5,fill=col)

for k in range(n):
    t=T_A+k/FPS
    fr=F[min(max(bisect.bisect_right(FT,t)-1,0),len(F)-1)]

    im=Image.new('RGB',(W,H),C_BG); dr=ImageDraw.Draw(im,'RGBA')

    # ── 뷰 ──
    if LAYOUT!='view':
        dr.rounded_rectangle([VX,VY,VX+VW,VY+VW],14,fill=C_VIEW,outline=(50,60,74))
    else:
        dr.rectangle([0,0,W,H],fill=C_VIEW)
    R3=P_RANGE*SC
    dr.ellipse([CX-R3,CY-R3,CX+R3,CY+R3],fill=(*C_ZONE,54),outline=(*C_ZONE,220),width=2)
    for rr in (1,2,3,4):
        R=rr*SC
        dr.ellipse([CX-R,CY-R,CX+R,CY+R],outline=C_RING,width=1)
        dr.text((CX+8,CY-R-20),f'{rr} m',font=f_xs,fill=C_RING)

    for a,r in fr.get('raw',[]):
        if r>RMAX: continue
        px=CX-r*math.sin(a)*SC; py=CY-r*math.cos(a)*SC
        dr.ellipse([px-1,py-1,px+1,py+1],fill=C_RAW)

    people=[]; wallfp=[]
    for c in fr['cl']:
        wfp = c['person'] and not c['person_f']   # 라이다는 통과, 지도 필터가 배제
        col = C_OK if c['person_f'] else (C_WALLFP if wfp else C_NO)
        for a,r in c['pts']:
            if r>RMAX: continue
            px=CX-r*math.sin(a)*SC; py=CY-r*math.cos(a)*SC
            sz = 3 if c['person_f'] else 2
            dr.ellipse([px-sz,py-sz,px+sz,py+sz],fill=col)
        if c['person_f'] and c['r']<=RMAX: people.append(c)
        elif wfp and c['r']<=RMAX:         wallfp.append(c)
    for c in wallfp:                              # 걸러낸 것도 숨기지 않는다
        px=CX-c['r']*math.sin(c['ang'])*SC; py=CY-c['r']*math.cos(c['ang'])*SC
        rad=min(max(12,(c['w']/2)*SC+8),46)
        dr.ellipse([px-rad,py-rad,px+rad,py+rad],outline=(*C_WALLFP,210),width=2)
    people.sort(key=lambda c:c['r'])
    for i,c in enumerate(people,1):
        px=CX-c['r']*math.sin(c['ang'])*SC; py=CY-c['r']*math.cos(c['ang'])*SC
        rad=min(max(15,(c['w']/2)*SC+11),48)
        dr.ellipse([px-rad,py-rad,px+rad,py+rad],outline=C_OK,width=3)
        bx,by=px+rad+5,py-rad-5
        dr.ellipse([bx-14,by-14,bx+14,by+14],fill=C_OK)
        dr.text((bx-6,by-12),str(i),font=f_lab,fill=(10,20,16))

    dr.ellipse([CX-0.28*SC,CY-0.28*SC,CX+0.28*SC,CY+0.28*SC],outline=C_ROB,width=3)
    L=0.36*SC
    dr.polygon([(CX,CY-L),(CX-0.19*SC,CY+0.12*SC),(CX+0.19*SC,CY+0.12*SC)],fill=C_ROB)

    if LAYOUT!='view':
        # ── 우측 패널 ──
        vis,lost=fr['vis'],fr['lost']
        if vis:    btxt,bcol = 'VISIBLE',C_OK
        elif lost: btxt,bcol = 'LOST',C_LOST
        else:      btxt,bcol = '대기',C_DIM
        x=PX; y=VY+30
        dr.text((x,y),btxt,font=f_badge,fill=bcol); y+=100
        dr.text((x+3,y),'이번 스캔 검출 '+('있음' if fr['det'] else '없음'),
                font=f_sub,fill=C_OK if fr['det'] else C_NO); y+=62

        def meter(lab, val, lim, col):
            dr.text((x,y+6),lab,font=f_lab,fill=col)
            tw=dr.textlength(lab,font=f_lab)
            dr.text((x+tw+16,y),f'{min(val,lim):.1f} / {lim:.0f} s',font=f_num,fill=col)
        if not fr['det']:
            g=fr['gap'] if fr['gap'] is not None else 0.0
            meter('놓침', g, P_LOST, C_LOST); y+=44
            bar(dr,x,y,PW,18,g/P_LOST,C_LOST); y+=54
        else:
            h=fr['held'] if fr['held'] is not None else 0.0
            meter('연속 검출', h, P_SEEN, C_OK); y+=44
            bar(dr,x,y,PW,18,h/P_SEEN,C_OK); y+=54

        # 큰 숫자 둘 — 축소해도 읽힌다
        npe=len(people); nwf=len(wallfp)
        colw=PW//2
        dr.text((x,y),'사람',font=f_lab,fill=C_DIM)
        dr.text((x+colw,y),'지도 배제',font=f_lab,fill=C_DIM); y+=26
        dr.text((x,y),str(npe),font=f_big,fill=C_OK if npe else C_NO)
        dr.text((x+colw,y),str(nwf),font=f_big,fill=C_WALLFP if nwf else (60,70,86)); y+=86

        dr.line([x,y,x+PW,y],fill=C_LINE); y+=32
        dr.text((x,y),state_at(ex_t(t)),font=f_st,fill=C_OK); y+=54
        dr.text((x,y),f'T + {mis_t(t):.1f} s',font=f_t,fill=C_TXT); y+=56

        mw,mh=PW,168
        dr.rounded_rectangle([x,y,x+mw,y+mh],10,fill=C_VIEW,outline=(50,60,74))
        for (x0,y0,x1,y1) in [(-0.5,-0.08,13.0,-0.08),(-0.5,-10.65,13.0,-10.65),
                              (8.95,-0.08,8.95,-10.65)]:
            a=mini(x0,y0,x,y,mw,mh); b=mini(x1,y1,x,y,mw,mh)
            dr.line([a,b],fill=(70,82,98),width=2)
        p=pose_at(ex_t(t)); mp_=mini(p[1],p[2],x,y,mw,mh)
        dr.ellipse([mp_[0]-6,mp_[1]-6,mp_[0]+6,mp_[1]+6],fill=C_ROB)

        dr.text((VX,VY+VW+14),
                f'초록 = 사람   ·   보라 = 지도상 벽이라 배제   ·   회색 = 탈락   ·   '
                f'파란 원 = 판정 영역 {P_RANGE:.1f} m',font=f_xs,fill=C_DIM)
        dr.text((VX,VY+VW+38),
                '※ "영역 안에 사람 크기 덩어리가 있는가"만 본다 — 누가 누구인지는 구분하지 않는다.',
                font=f_xs,fill=(190,150,120))

    ff.stdin.write(np.asarray(im).tobytes())
    if k%300==0: print(f'  {k}/{n}',flush=True)

ff.stdin.close(); ff.wait()
print('완료:', f'{S}/{NAME}.mp4')
