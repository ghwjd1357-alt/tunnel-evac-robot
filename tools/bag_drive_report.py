#!/usr/bin/env python3
"""rosbag2 → 구동계 진단. **로봇 없이** 오도메트리가 거짓말하는지 가른다 (08-22 신설).

사용:
    python3 tools/bag_drive_report.py <bag 디렉터리> [<bag> ...]

🔴 **왜 이 도구가 있나** — 08-21 밤에 이 분석을 임시 스크립트로 돌려
`우측 합성 계측 ≈0.52×` 를 확인했다(1채널 dead 는 가설)(`REAL_ROBOT_VALUES §1-m-11`).
그런데 그 스크립트를 남기지 않아 **결론만 있고 방법이 재현 불가**였다.
독립 검토가 그 진단을 확인하려 해도 다시 짜야 한다. 그래서 도구로 되살린다.

★ **판별 원리 — 근거가 둘이어야 확정된다**

  ① **배율** — 직진 배율과 회전 배율을 각각 잰다.
     회전만 틀리면 `ODOM_WHEEL_BASE`(회전에만 들어간다).
     🔴 둘이 **같은 배율**이면 `ODOM_WHEEL_RADIUS` **또는** 좌우 결손이다 —
        반지름은 모든 바퀴 거리에 같은 계수로 곱해지므로 둘 다 똑같이 틀린다.
        08-22 검토에서 이 구분이 빠져 있었음을 확인하고 보강했다.
  ② **비대칭** — 직진 명령 중 `/odom` 이 회전을 보고하는가(phantom ω).
     반지름 오차는 **좌우 대칭**이라 phantom ω 를 못 만든다.
     한쪽 결손·부호반전만 만든다. 🎯 **가르는 것은 이쪽이다.**

★ **`/odom` 하나로 계수가 나온다** (지면 진실도 IMU 도 필요 없다)

    직진:  odom속도 = v(kL+kR)/2 · odomω = v(kR−kL)/BASE
           r = odomω·BASE/(2·odom속도) = (kR−kL)/(kR+kL)      ← v 가 지워진다
    성한 쪽을 1 로 두면  약한 쪽 = (1−|r|)/(1+|r|)
    🔴 0.5 근처면 **1채널 dead 가 선두 가설**이다 — 확정은 개별 바퀴 시험이 한다.

정본 = `REAL_ROBOT_VALUES §1-m-11` · 함정 = `PITFALLS §18`.
짝 도구 = `tools/drive_health.py --straight`(실시간) · `tools/drive_encoder_check.py`(바퀴 특정).
"""
import math
import os
import statistics as st
import sys

# 🔴 odom 계열 상수 — `.ino` ODOM_WHEEL_BASE. 명령 경로 0.62 도 물리 0.49 도 아니다.
#   회귀(`test_bag_drive_report.py`)가 `.ino` 에서 읽어 대조한다.
# 🔴 2026-08-22 재교정 0.829 -> 0.859 (`.ino` 와 한 쌍. 회귀가 `.ino` 에서 읽어 대조한다).
#   근거 = 08-22 실측: 줄자 직진 2회가 반지름을 지지(0.9936·0.9919) · 제자리 회전
#   3회에서 odom/IMU = 1.0431. 정본 = docs/REAL_ROBOT_VALUES.md §1-c.
ODOM_WHEEL_BASE = 0.859
BAL_OK = 0.05        # |r| 이 이 아래면 좌우 균형 정상


# 🔴 08-22 재교정 **이전**에 찍은 bag 은 이 값으로 풀어야 한다 — 그때 펌웨어가 쓰던
#   눈금이기 때문이다. 새 값(0.859)으로 옛 bag 을 풀면 kR 이 조용히 어긋난다
#   (08-21 21:32 리허설이 0.525 -> 0.507 로 읽힌다). `--pre-0822` 로 고른다.
WHEEL_BASE_PRE_0822 = 0.829


def solve_kr(odom_lin, odom_w, base=ODOM_WHEEL_BASE):
    """직진 구간의 (속도, ω) 에서 약한 쪽 계수를 역산한다. 순수 함수 — 회귀 대상.

    🔴 **`k` 는 정규화 비율이지 절대 교정값이 아니다** (§87.3). `kL=1` 은 가정이고,
    bag 이 식별하는 것은 "약한 쪽 합성 계측이 성한 쪽의 k 배" 까지다. 어느 **채널**이
    얼마나인지는 개별 바퀴 시험이 가른다.

    반환 (r, weak, k):
      r    = (kR−kL)/(kR+kL). 0 이면 좌우 같다
      weak = 'right' | 'left' | None — **쪽**이지 채널이 아니다
      k    = 약한 쪽 **합성** 계수 (성한 쪽을 1.0 으로). None 이면 판정 불가
    """
    if odom_lin is None or abs(odom_lin) < 1e-6:
        return None, None, None
    r = odom_w * base / (2.0 * odom_lin)
    if abs(r) <= BAL_OK:
        return r, None, 1.0
    # 🔴 부호: 직진 중 odom 이 '왼쪽으로 휜다'(ω<0)고 말하면 오른쪽이 덜 세는 것이다
    #   (deltaYaw = (dR − dL)/BASE 이므로 dR 이 작으면 음수).
    return r, ('right' if r < 0 else 'left'), (1.0 - abs(r)) / (1.0 + abs(r))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read(path, topics):
    from rclpy.serialization import deserialize_message      # noqa: PLC0415
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions  # noqa: PLC0415, E501
    from rosidl_runtime_py.utilities import get_message      # noqa: PLC0415

    r = SequentialReader()
    r.open(StorageOptions(uri=path, storage_id='sqlite3'),
           ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = {t: get_message(types[t]) for t in topics if t in types}
    out = {t: [] for t in want}
    while r.has_next():
        tn, data, ts = r.read_next()
        if tn in want:
            out[tn].append((ts * 1e-9, deserialize_message(data, want[tn])))
    return out


def segments(cmds, kind, minlen):
    """`/cmd_vel` 에서 직진/회전 구간을 뽑는다. 반환 = [(시작, 끝)]."""
    out, cur = [], None
    for t, m in cmds:
        v, w = m.linear.x, m.angular.z
        hit = ((v > 0.05 and abs(w) < 0.02) if kind == 'straight'
               else (abs(v) < 0.03 and abs(w) >= 0.25))
        if hit:
            cur = (t, t) if cur is None else (cur[0], t)
        else:
            if cur and cur[1] - cur[0] >= minlen:
                out.append(cur)
            cur = None
    if cur and cur[1] - cur[0] >= minlen:
        out.append(cur)
    return out


def report(path, base=ODOM_WHEEL_BASE):
    name = path.rstrip('/').split('/')[-1]
    d = read(path, ['/odom', '/imu/data', '/cmd_vel'])
    if not d.get('/odom') or not d.get('/cmd_vel'):
        print(f'{name:24s} (/odom 또는 /cmd_vel 없음)')
        return 2
    odo = [(t, m.twist.twist.linear.x, m.twist.twist.angular.z)
           for t, m in d['/odom']]
    imu = [(t, m.angular_velocity.z) for t, m in d.get('/imu/data', [])]

    def win(seq, a, b):
        return [x for x in seq if a + 1.0 <= x[0] <= b]

    # ── ② 비대칭 (직진 중 phantom ω) — 확정의 열쇠 ────────────────────
    lin = ow = iw = None
    segs = segments(d['/cmd_vel'], 'straight', 5.0)
    if segs:
        L, W, I = [], [], []
        for a, b in segs:
            O, M = win(odo, a, b), win(imu, a, b)
            if len(O) < 20:
                continue
            L.append(st.median([x[1] for x in O]))
            W.append(st.median([x[2] for x in O]))
            if len(M) >= 20:
                I.append(st.median([x[1] for x in M]))
        if L:
            lin, ow = st.median(L), st.median(W)
            iw = st.median(I) if I else None

    r, weak, k = (solve_kr(lin, ow, base=base) if lin is not None
                  else (None, None, None))
    print(f'\n──── {name} ────')
    if lin is None:
        print('  직진 구간(≥5s)이 없다 — 비대칭 판정 불가')
    else:
        imus = f'{iw:+.4f}' if iw is not None else '없음'
        print(f'  직진 {len(segs)}구간 · odom속도 {lin:.4f} · odom ω {ow:+.4f} · IMU ω {imus}')
        if weak is None:
            print(f'  🟢 좌우 균형 정상 (r={r:+.4f})')
        else:
            side = '오른쪽' if weak == 'right' else '왼쪽'
            print(f'  🔴 **{side}이 {k:.3f} 배로 읽힌다** (r={r:+.4f})')
            if abs(k - 0.5) < 0.12:
                # 🔴 §87.3 — bag 이 식별하는 것은 **합성 비율**까지다. 같은 0.52 를
                #   (1.0, 0.05) 도 (0.525, 0.525) 도 만든다. 확정은 개별 바퀴 시험이다.
                print(f'     → 0.5 에 가깝다. **선두 가설 = {side} 채널 하나가 0**')
                print(f'     🔴 확정 아님 — 두 채널이 함께 낮아도 같은 값이 나온다.')
                print(f'        `drive_encoder_check.py --wheels=FR`/`--wheels=RR` 로 가른다.')
    # ── ① 배율 (참고 — 이것만으로는 반지름과 안 갈린다) ──────────────
    rb = []
    for a, b in segments(d['/cmd_vel'], 'rotation', 1.5):
        O, M = win(odo, a, b), win(imu, a, b)
        if len(O) < 10 or len(M) < 10:
            continue
        o_ = st.median([abs(x[2]) for x in O])
        i_ = st.median([abs(x[1]) for x in M])
        if i_ > 0.05 and o_ > 1e-6:
            rb.append(i_ / o_)
    if rb:
        print(f'  회전 배율 {st.median(rb):.2f} ({len(rb)}구간) '
              f'⚠ 배율만으로는 반지름 오차와 안 갈린다 — 위 비대칭이 판별자다')
    return 0 if weak is None else 1


def main(argv):
    if len(argv) < 2:
        print(__doc__.split('사용:')[1].split('🔴')[0].strip(), file=sys.stderr)
        return 2
    # 🔴 옛 bag 은 옛 윤거로 푼다 (위 WHEEL_BASE_PRE_0822 주석).
    pre = '--pre-0822' in argv
    base = WHEEL_BASE_PRE_0822 if pre else ODOM_WHEEL_BASE
    rc = 0
    for p in [a for a in argv[1:] if not a.startswith('--')]:
        if not os.path.isdir(p):
            print(f'없는 경로: {p}', file=sys.stderr)
            rc = max(rc, 2)
            continue
        rc = max(rc, report(p, base=base))
    return rc


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
