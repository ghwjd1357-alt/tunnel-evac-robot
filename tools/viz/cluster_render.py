# -*- coding: utf-8 -*-
import os
BAGS = os.environ.get('VIZ_BAGS', os.path.expanduser('~/robot_evidence'))
WORK = os.environ.get('VIZ_WORK', os.path.join(BAGS, 'viz', '_work'))
REPO = os.environ.get('VIZ_REPO', os.path.expanduser('~/ros2_ws'))
TAG  = os.environ.get('VIZ_TAG', 'realtake6')
os.makedirs(WORK, exist_ok=True)
"""클러스터 판정 시각화 — realtake6. 로봇 중심 확대 뷰."""
import pickle, math, bisect, subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont

S = WORK
D  = pickle.load(open(f'{S}/{TAG}_cluster.pkl','rb'))
MP = pickle.load(open(f'{S}/{TAG}.pkl','rb'))     # 미니맵·미션상태용
F  = D['frames']; FT=[f['t'] for f in F]

W,H = 1920,1080
VX,VY,VW = 40,112,880                 # 좌측 뷰 (정사각)
CX,CY = VX+VW//2, VY+VW//2
RMAX  = 4.0                            # 화면 반경 [m]
SC    = (VW/2)/RMAX
PX    = VX+VW+40; PW = W-PX-40

C_BG=(14,17,22); C_VIEW=(20,25,32); C_RING=(48,58,72)
C_ZONE=(38,86,120); C_PT=(120,132,148); C_RAW=(58,68,82)
C_OK=(0,224,140); C_NO=(126,136,150); C_ROB=(255,214,10)
C_LOST=(255,72,58); C_TXT=(226,233,240); C_DIM=(138,150,166)

def F_(s,b=False): return ImageFont.truetype(
    f'/usr/share/fonts/opentype/noto/NotoSansCJK-{"Bold" if b else "Regular"}.ttc',s,index=1)
f_h,f_b,f_m,f_s,f_xs = F_(30,True),F_(44,True),F_(22,True),F_(18),F_(15)

# 미션 상태 타임라인 (cluster pkl 과 같은 시각계로 맞춘다)
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

# 미니맵 범위
MX0,MX1,MY0,MY1 = -1.0,13.5,-11.8,1.0
def mini(x,y,ox,oy,w,h):
    return ox+(x-MX0)/(MX1-MX0)*w, oy+(MY1-y)/(MY1-MY0)*h

T_A  = float(os.environ.get('VIZ_T0', 162.0))
T_B  = float(os.environ.get('VIZ_T1', 296.0))
FPS  = int(os.environ.get('VIZ_FPS', 30))
NAME = os.environ.get('VIZ_NAME', 'cluster')
n=int((T_B-T_A)*FPS)
print(f'{n} 프레임 · {n/FPS:.0f}초 · 실속도 1배',flush=True)
ff=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
    '-s',f'{W}x{H}','-r',str(FPS),'-i','-','-c:v','libx264','-pix_fmt','yuv420p','-crf','19',
    f'{S}/{NAME}.mp4'],stdin=subprocess.PIPE)

def bar(dr,x,y,w,h,frac,col,bgc=(44,52,64)):
    dr.rounded_rectangle([x,y,x+w,y+h],4,fill=bgc)
    if frac>0: dr.rounded_rectangle([x,y,x+max(6,int(w*min(frac,1.0))),y+h],4,fill=col)

for k in range(n):
    t=T_A+k/FPS
    fr=F[min(max(bisect.bisect_right(FT,t)-1,0),len(F)-1)]

    im=Image.new('RGB',(W,H),C_BG); dr=ImageDraw.Draw(im,'RGBA')
    dr.text((VX,34),'라이다 추종 판정 — 점 클러스터',font=f_h,fill=C_TXT)
    dr.text((VX,74),'realtake6 · 실제 FollowerMonitor 를 bag 의 /scan 에 그대로 돌린 결과',
            font=f_xs,fill=C_DIM)

    # 뷰 배경
    dr.rounded_rectangle([VX,VY,VX+VW,VY+VW],14,fill=C_VIEW,outline=(52,62,76))
    # 판정 영역 3.0 m
    R3=3.0*SC
    dr.ellipse([CX-R3,CY-R3,CX+R3,CY+R3],fill=(*C_ZONE,54),outline=(*C_ZONE,220),width=2)
    for rr in (1,2,3,4):
        R=rr*SC
        dr.ellipse([CX-R,CY-R,CX+R,CY+R],outline=C_RING,width=1)
        dr.text((CX+6,CY-R-18),f'{rr} m',font=f_xs,fill=C_RING)

    # 배경: 판정 영역 밖까지 전체 점군 (복도 형태가 보이게)
    for a,r in fr.get('raw',[]):
        if r>RMAX: continue
        px=CX-r*math.sin(a)*SC; py=CY-r*math.cos(a)*SC
        dr.ellipse([px-1,py-1,px+1,py+1],fill=C_RAW)

    # 점군: 클러스터별로 색칠 (초록=사람 크기 / 회색=탈락)
    people=[]
    for c in fr['cl']:
        col = C_OK if c['person'] else C_NO
        for a,r in c['pts']:
            if r>RMAX: continue
            px=CX-r*math.sin(a)*SC; py=CY-r*math.cos(a)*SC
            sz = 3 if c['person'] else 2
            dr.ellipse([px-sz,py-sz,px+sz,py+sz],fill=col)
        if c['person'] and c['r']<=RMAX:
            people.append(c)
    # 사람 판정에만 링 + 번호 배지
    people.sort(key=lambda c:c['r'])
    for idx,c in enumerate(people,1):
        px=CX-c['r']*math.sin(c['ang'])*SC; py=CY-c['r']*math.cos(c['ang'])*SC
        rad=min(max(14,(c['w']/2)*SC+10),46)
        dr.ellipse([px-rad,py-rad,px+rad,py+rad],outline=C_OK,width=3)
        bx,by=px+rad+4,py-rad-4
        dr.ellipse([bx-13,by-13,bx+13,by+13],fill=C_OK)
        dr.text((bx-6,by-11),str(idx),font=f_s,fill=(10,20,16))

    # 로봇
    dr.ellipse([CX-0.28*SC,CY-0.28*SC,CX+0.28*SC,CY+0.28*SC],outline=C_ROB,width=3)
    L=0.36*SC
    dr.polygon([(CX,CY-L),(CX-0.19*SC,CY+0.12*SC),(CX+0.19*SC,CY+0.12*SC)],fill=C_ROB)
    dr.text((CX-24,CY+0.30*SC+8),'로봇',font=f_xs,fill=C_ROB)

    # ── 우측 패널 ───────────────────────────────────
    y=VY
    vis,lost=fr['vis'],fr['lost']
    badge = ('🟢','VISIBLE',C_OK) if vis else (('🔴','LOST',C_LOST) if lost else ('·','대기',C_DIM))
    dr.text((PX,y),'추종 판정',font=f_xs,fill=C_DIM); y+=26
    dr.text((PX,y),badge[1],font=f_b,fill=badge[2]); y+=62
    dr.text((PX,y),'이번 스캔 검출: '+('있음' if fr['det'] else '없음'),
            font=f_m,fill=C_OK if fr['det'] else C_NO); y+=44

    if not fr['det']:
        g=fr['gap'] if fr['gap'] is not None else 0.0
        dr.text((PX,y),f'놓침 카운트  {min(g,3.0):.1f} / 3.0 초',font=f_s,fill=C_LOST); y+=26
        bar(dr,PX,y,PW-20,16,g/3.0,C_LOST); y+=38
        dr.text((PX,y),'3초를 기다리는 이유 — 한 번 깜빡였다고',font=f_xs,fill=C_DIM); y+=20
        dr.text((PX,y),'역행을 시작하지 않기 위해서다',font=f_xs,fill=C_DIM); y+=34
    else:
        h=fr['held'] if fr['held'] is not None else 0.0
        dr.text((PX,y),f'연속 검출  {min(h,3.0):.1f} / 3.0 초',font=f_s,fill=C_OK); y+=26
        bar(dr,PX,y,PW-20,16,h/3.0,C_OK); y+=38
        dr.text((PX,y),'3초 연속으로 보여야 재발견을 확정한다',font=f_xs,fill=C_DIM); y+=34

    y+=10
    dr.text((PX,y),'사람 판정 기준 (waypoints_real_H.yaml)',font=f_xs,fill=C_DIM); y+=26
    for lab in ['덩어리 폭  ≤ 0.80 m','점 개수    ≥ 3','거리       ≤ 3.00 m','구역       전방위 (zone=any)']:
        dr.text((PX+4,y),lab,font=f_s,fill=C_TXT); y+=26
    y+=14
    npe=len(people); nrej=len(fr['cl'])-npe
    dr.text((PX,y),f'사람 크기 덩어리  {npe}개',font=f_m,fill=C_OK if npe else C_NO); y+=32
    for idx,c in enumerate(people,1):
        dr.ellipse([PX+2,y+3,PX+22,y+23],fill=C_OK)
        dr.text((PX+9,y+4),str(idx),font=f_xs,fill=(10,20,16))
        dr.text((PX+32,y+3),f'폭 {c["w"]:.2f} m · {c["n"]}점 · {c["r"]:.2f} m',font=f_s,fill=C_TXT)
        y+=28
    if npe==0:
        dr.text((PX+4,y),'—',font=f_s,fill=C_NO); y+=28
    dr.text((PX,y+4),f'탈락 {nrej}개 (벽·노이즈)',font=f_xs,fill=C_DIM); y+=40

    dr.text((PX,y),'미션 상태',font=f_xs,fill=C_DIM); y+=24
    dr.text((PX,y),state_at(t),font=f_m,fill=C_OK); y+=40
    dr.text((PX,y),f'T + {t:6.1f} s',font=f_s,fill=C_DIM); y+=44

    # 미니맵
    mw,mh=PW-20,150
    dr.rounded_rectangle([PX,y,PX+mw,y+mh],8,fill=C_VIEW,outline=(52,62,76))
    for (x0,y0,x1,y1) in [(-0.5,-0.08,13.0,-0.08),(-0.5,-10.65,13.0,-10.65),(8.95,-0.08,8.95,-10.65)]:
        a=mini(x0,y0,PX,y,mw,mh); b=mini(x1,y1,PX,y,mw,mh)
        dr.line([a,b],fill=(70,82,98),width=2)
    p=pose_at(t); mp_=mini(p[1],p[2],PX,y,mw,mh)
    dr.ellipse([mp_[0]-5,mp_[1]-5,mp_[0]+5,mp_[1]+5],fill=C_ROB)
    dr.text((PX+8,y+mh-22),'아래 복도 · 서쪽이 탈출구',font=f_xs,fill=C_DIM)

    dr.text((VX,VY+VW+18),'초록 = 사람 크기로 판정   ·   회색 = 덩어리이지만 탈락(벽·노이즈)   ·   '
            '파란 원 = 판정 영역 3.0 m (전방위)',font=f_xs,fill=C_DIM)
    dr.text((VX,VY+VW+42),'※ 이 판정은 "영역 안에 사람 크기 덩어리가 있는가" 다 — '
            '누가 누구인지, 몇 명인지는 구분하지 않는다.',font=f_xs,fill=(190,150,120))
    dr.text((VX,VY+VW+64),'※ 벽 모서리가 폭 0.5~0.7 m 조각으로 잘려 통과하는 경우도 그대로 표시한다 — 남아 있는 오탐이다.',font=f_xs,fill=(190,150,120))

    ff.stdin.write(np.asarray(im).tobytes())
    if k%300==0: print(f'  {k}/{n}',flush=True)

ff.stdin.close(); ff.wait()
print('완료:', f'{S}/{NAME}.mp4')
