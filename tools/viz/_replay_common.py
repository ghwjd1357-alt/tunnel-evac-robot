# -*- coding: utf-8 -*-
"""④ 재생 도구 공용부 — 기록 적재 · 시각 조회 · CLI.

🔴 여기 있는 것은 전부 08-23 실차의 **실제 기록**이다
   (`/mission_state` · `/siren` · mission_manager `/rosout`).
   다만 그날 터미널을 캡처한 화면은 아니다 — **재생**이다.
"""
import os, sys, time, bisect, pickle, argparse

BAGS = os.environ.get('VIZ_BAGS', os.path.expanduser('~/robot_evidence'))
WORK = os.environ.get('VIZ_WORK', os.path.join(BAGS, 'viz', '_work'))
REPO = os.environ.get('VIZ_REPO', os.path.expanduser('~/ros2_ws'))
TAG  = os.environ.get('VIZ_TAG', 'realtake6')

R='\033[0m'; DIM='\033[2m'; GRAY='\033[90m'
RED='\033[91m'; YEL='\033[93m'; ORA='\033[38;5;208m'; BLU='\033[94m'; GRN='\033[92m'

_D = pickle.load(open(os.path.join(WORK, f'{TAG}.pkl'), 'rb'))
STATES = _D['state']
SIREN  = _D['siren']
LOG    = sorted(_D['log'], key=lambda r: r[0])
RAW    = _D.get('state_raw', _D['state'])
MT0    = _D['mt0']
ST_T=[s[0] for s in STATES]; SI_T=[s[0] for s in SIREN]
LG_T=[r[0] for r in LOG];    RW_T=[r[0] for r in RAW]

def at(arr, tl, t, default=None):
    i = bisect.bisect_right(tl, t) - 1
    return arr[i] if i >= 0 else default

def state_at(t):
    st = at(STATES, ST_T, t)
    return (st[1] if st else '(대기)'), (t - st[0] if st else 0.0)

def siren_at(t):
    sv = at(SIREN, SI_T, t)
    return bool(sv and sv[1])

def key_moments():
    out  = [(s[0]-MT0, f'상태 → {s[1]}') for s in STATES]
    out += [(l[0]-MT0, l[1]) for l in LOG if l[1].startswith(('★','🔥','🔵'))]
    return sorted(out)

def cli(desc):
    ap = argparse.ArgumentParser(description=desc)
    ap.add_argument('--speed', type=float, default=1.0, help='재생 배속 (기본 1.0)')
    ap.add_argument('--at',    type=float, default=None, help='T+초. 그 장면만 띄우고 멈춘다 (스샷용)')
    ap.add_argument('--from',  dest='t0', type=float, default=None, help='T+초부터')
    ap.add_argument('--to',    dest='t1', type=float, default=None, help='T+초까지')
    ap.add_argument('--fps',   type=float, default=4.0, help='다시 그리는 빈도')
    ap.add_argument('--list',  action='store_true', help='스샷 찍을 만한 순간 목록')
    return ap

def show_list():
    print(f'{DIM}스샷 찍을 만한 순간  ( --at 값 ){R}\n')
    for v, m in key_moments():
        print(f'  --at {v:6.1f}   {m}')

def span(a):
    t0 = (a.t0 + MT0) if a.t0 is not None else STATES[0][0] - 3.0
    t1 = (a.t1 + MT0) if a.t1 is not None else LOG[-1][0] + 4.0
    return t0, t1

def loop(a, draw):
    """draw(t) 를 배속에 맞춰 반복 호출한다. --at 이면 한 번만."""
    if a.at is not None:
        draw(a.at + MT0)
        print(f'\n{DIM}  (T+{a.at:.1f}s 정지 — 스샷을 찍으세요. Ctrl+C 로 종료){R}')
        try:
            while True: time.sleep(3600)
        except KeyboardInterrupt: pass
        return 0
    t0, t1 = span(a); t = t0; step = a.speed / a.fps
    try:
        while t <= t1:
            draw(t); time.sleep(1.0 / a.fps); t += step
    except KeyboardInterrupt: pass
    return 0
