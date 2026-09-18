# -*- coding: utf-8 -*-
"""④ HUD · 미션 로그 — 세로 스트립 (기본 300x1080, 1배속).

🔴 화면 녹화가 아니라 **재렌더**다. 데이터·문구는 08-23 실차의 실제 기록
   (`/mission_state` · `/siren` · mission_manager `/rosout`)이지만, 그날 노트북
   터미널을 캡처한 화면은 아니다. *"터미널을 찍었다"* 로 소개하지 않는다.
   원본 계기판 = `tools/mission_hud.py` (터미널 블록문자라 300px 폭에 안 들어간다).
"""
import os
BAGS = os.environ.get('VIZ_BAGS', os.path.expanduser('~/robot_evidence'))
WORK = os.environ.get('VIZ_WORK', os.path.join(BAGS, 'viz', '_work'))
TAG  = os.environ.get('VIZ_TAG', 'realtake6')
os.makedirs(WORK, exist_ok=True)
import pickle, bisect, subprocess, re
import numpy as np
from PIL import Image, ImageDraw, ImageFont

D = pickle.load(open(f'{WORK}/{TAG}.pkl','rb'))
states=D['state']; stt=[s[0] for s in states]
sir=D['siren'];    sirt=[s[0] for s in sir]
LOG=sorted(D['log'], key=lambda r:r[0])
LT=[r[0] for r in LOG]
MT0=D['mt0']

W    = int(os.environ.get('VIZ_W', 300))
H    = int(os.environ.get('VIZ_H', 1080))
T_A  = float(os.environ.get('VIZ_T0', 10.0))
T_B  = float(os.environ.get('VIZ_T1', 326.0))
SPEED= float(os.environ.get('VIZ_SPEED', 1.0))     # ④ 는 기본 1배속
FPS  = int(os.environ.get('VIZ_FPS', 20))
NAME = os.environ.get('VIZ_NAME', 'hud')

C_BG=(14,17,22); C_PANEL=(20,25,32); C_LINE=(42,50,62)
C_TXT=(226,233,240); C_DIM=(128,142,160); C_MUT=(88,100,118)
C_OK=(0,224,140); C_ALERT=(255,120,52); C_LOST=(255,72,58)
C_STAR=(255,214,10); C_FIRE=(255,138,0); C_BLUE=(96,170,255)
ALERT={'SEARCH_BACK','FAULT','BLOCKED'}
HOLDC={'HOLD'}
MEAN={'PATROL':'평시 순찰','APPROACH':'화재 감지 · 집결지로 출동','SCAN_AREA':'집결지 360° 훑기 · 대피자 탐색',
      'GATHER':'집결 대기','GUIDE':'저속 선행 유도 · 후방 감시','HOLD':'추종자 놓침 · 제자리 재수집',
      'SEARCH_BACK':'추종자 놓침 · 역행 재탐색','ESCAPED':'탈출 완료',
      'FAULT':'주행 실패 · 재시도','BLOCKED':'안전한 집결지 없음 · 사람 판단 대기'}

NOTO='/usr/share/fonts/opentype/noto/NotoSansCJK-{}.ttc'
MONO='/usr/share/fonts/truetype/dejavu/DejaVuSansMono{}.ttf'
NF=lambda w,s: ImageFont.truetype(NOTO.format(w),s,index=1)
MF=lambda b,s: ImageFont.truetype(MONO.format('-Bold' if b else ''),s)
f_lab=NF('Medium',12); f_mean=NF('Regular',15); f_t=MF(True,24)
f_hold=NF('Regular',14); f_sir=NF('Medium',14)
f_log=NF('Regular',13); f_logt=MF(False,12)

PAD=16; INNER=W-PAD*2; GUT=36   # 시각 칸 폭

# 🔥🔵🔊 는 컬러 이모지라 흑백 폰트에서 두부가 된다 → 색 점으로 바꾼다
MARK={'🔥':C_FIRE,'🔵':C_BLUE,'★':C_STAR,'🔊':C_OK}
EMO=re.compile('[🔥🔵🔊★]')
def split_mark(msg):
    col=None
    for ch,c in MARK.items():
        if msg.startswith(ch): col=c; break
    return col, EMO.sub('', msg).strip()

def wrap(dr, text, font, width):
    out=[]; cur=''
    for ch in text:
        if dr.textlength(cur+ch, font=font) <= width: cur+=ch
        else: out.append(cur); cur=ch
    if cur: out.append(cur)
    return out

def fit(dr, text, maxw, weight='Black', hi=44, lo=18):
    while hi>lo:
        f=NF(weight,hi)
        if dr.textlength(text,font=f)<=maxw: return f
        hi-=1
    return NF(weight,lo)

def pick(a,tl,t):
    i=bisect.bisect_right(tl,t)-1
    return a[i] if i>=0 else None

step=SPEED/FPS
frames=int((T_B-T_A)/step)
print(f'{frames} 프레임 · {frames/FPS:.0f}초 · {SPEED}배속 · {W}x{H}',flush=True)
ff=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24',
    '-s',f'{W}x{H}','-r',str(FPS),'-i','-','-c:v','libx264','-pix_fmt','yuv420p','-crf','20',
    f'{WORK}/{NAME}.mp4'],stdin=subprocess.PIPE)

# 로그 줄바꿈을 미리 계산 (매 프레임 다시 하면 느리다)
_m=Image.new('RGB',(10,10)); _d=ImageDraw.Draw(_m)
PRE=[]
for t,msg in LOG:
    col,txt=split_mark(msg)
    PRE.append((t,col,wrap(_d,txt,f_log,INNER-GUT)))

for k in range(frames):
    t=T_A+k*step
    im=Image.new('RGB',(W,H),C_BG); dr=ImageDraw.Draw(im)

    st=pick(states,stt,t)
    name=st[1] if st else '대기'
    since=t-st[0] if st else 0.0
    acc = C_LOST if name in ALERT else (C_STAR if name in HOLDC else C_OK)
    sv=pick(sir,sirt,t); on=bool(sv and sv[1])

    y=PAD+14
    dr.text((PAD,y),'MISSION',font=f_lab,fill=C_MUT); y+=20
    fs=fit(dr,name,INNER)
    dr.text((PAD,y),name,font=fs,fill=acc); y+=fs.size+12
    for ln in wrap(dr,MEAN.get(name,''),f_mean,INNER)[:2]:
        dr.text((PAD,y),ln,font=f_mean,fill=C_DIM); y+=21
    y+=16
    dr.text((PAD,y),f'T + {t-MT0:.1f} s',font=f_t,fill=C_TXT); y+=36
    m,s=divmod(int(since),60)
    dr.text((PAD,y),f'이 상태 유지  {m:02d}:{s:02d}',font=f_hold,fill=C_DIM); y+=26
    if on:
        dr.rounded_rectangle([PAD,y,PAD+96,y+30],6,fill=(176,38,30))
        dr.ellipse([PAD+12,y+11,PAD+21,y+20],fill=(255,228,224))
        dr.text((PAD+28,y+6),'사이렌',font=f_sir,fill=(255,238,234))
    y+=44
    dr.line([PAD,y,W-PAD,y],fill=C_LINE); y+=18
    dr.text((PAD,y),'미션 로그',font=f_lab,fill=C_MUT); y+=22

    # 로그: 지나간 것만, 오래된 게 위 — 아래로 흐른다
    top=y; avail=H-PAD-top
    i=bisect.bisect_right(LT,t)
    lines=[]
    for tt,col,ws in PRE[:i]:
        lines.append((tt,col,ws))
    # 아래에서부터 채워 넣어 최신이 항상 보이게
    buf=[]; used=0
    for tt,col,ws in reversed(lines):
        need=len(ws)*18+8
        if used+need>avail: break
        buf.append((tt,col,ws)); used+=need
    buf.reverse()
    yy=top+(avail-used if used<avail else 0)
    for tt,col,ws in buf:
        fade=1.0 if t-tt<3.0 else 0.60
        tc=tuple(int(c*fade) for c in (col or C_TXT))
        sub=tuple(int(c*fade*0.86) for c in (col or C_TXT))
        _v=tt-MT0; ts='0' if -1<_v<1 else f'{_v:.0f}'
        tw=dr.textlength(ts,font=f_logt)
        dr.text((PAD+GUT-10-tw,yy+2),ts,font=f_logt,
                fill=tuple(int(c*fade) for c in C_MUT))
        for j,ln in enumerate(ws):
            dr.text((PAD+GUT,yy+j*18),ln,font=f_log,fill=tc if j==0 else sub)
        yy+=len(ws)*18+8

    ff.stdin.write(np.asarray(im).tobytes())
    if k%800==0: print(f'  {k}/{frames}',flush=True)

ff.stdin.close(); ff.wait()
print('완료:', f'{WORK}/{NAME}.mp4')
