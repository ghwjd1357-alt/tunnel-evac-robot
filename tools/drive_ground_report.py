#!/usr/bin/env python3
"""지면 주행 실측 — `/odom` 으로 거리·속도·횡편차·yaw 를 뽑고 IMU 와 교차한다.

★ **이 도구가 재는 것과 안 재는 것**

  - **잰다** = 명령이 나간 구간의 **경로장·전진/횡 성분·yaw 변화**, 그리고 (주면)
    같은 시행의 **IMU yaw** 와의 차이.
  - **안 잰다** = 실제 이동 거리. 🔴 **그건 줄자다.** `/odom` 은 엔코더 파생값이고
    `twist` 도 같은 엔코더에서 나오므로 **서로를 검증하지 못한다.**

🔴 **줄자가 왜 정본인가** (2026-08-11 실측 두 번이 이걸 증명했다)

  ① R1 1차에서 `/odom` 이 `69.5°` 회전을 주장했는데 육안은 완벽한 직진이었다. 원인은
     좌전륜 엔코더 부호 반전이었고, **줄자가 없었으면 어느 쪽이 거짓말인지 못 갈랐다**.
  ② 부호를 고친 뒤 `/odom` 676mm vs **줄자 685mm** 로 맞았다. 그 순간 08-07부터 열려
     있던 질문("명령보다 빠른 것인가, 오도메트리 스케일이 2배인가")이 닫혔다 —
     **스케일이 아니라 실제로 빨랐다**(명령 `0.12` → 실측 `0.3265 m/s`, 예약 32).

⚠ **평균속도를 정상속도로 읽지 않는다.** 짧은 대조군은 가감속 구간이 커서 평균이 낮게
나온다. `0.12 m/s` 도달 판정은 `§7-c-1` 의 **3m 실측**이 한다.

★ **횡편차를 세계좌표가 아니라 시작 방향 기준으로 가른다** — 세계좌표 그대로 보면
로봇이 어느 방향을 보고 출발했는지에 따라 값이 흔들려 편차가 안 보인다.

사용법:
    python3 tools/drive_ground_report.py <bag> [--tape-mm 685]
    종료코드 0 = 판정 유효 / 1 = 판정 불능 / 2 = 입력 오류

정본 = `docs/JETSON_SETUP.md §7-c-R1`,`§7-c-1` · 짝 도구 = `tools/drive_encoder_check.py`.
"""
import math
import sys

# 🔴 08-13 삭제 (검토 §65.3). 여기 있던 `WHEEL_BASE_M = 0.62` 는 **아무 데서도 안 쓰였다**.
# 그런데 `test_drive_checks` 가 "펌웨어와 같은 값" 이라며 이 죽은 상수를 정답으로 고정해,
# 펌웨어가 명령용 0.62 / odom 용 0.670 으로 갈라진 뒤에도 초록으로 남았다.
# 이 도구는 윤거를 안 쓴다 — 줄자와 `/odom` 거리만 본다. 그래서 상수를 지웠다.
# 명령이 끊긴 뒤 watchdog 이 세울 때까지를 포함해 보는 꼬리. `§7-c-0` 실측 총 정지가
# 약 516ms 이므로 1.5s 면 정지까지 확실히 담는다.
COAST_TAIL_S = 1.5
# 가속 구간을 뺀 "정상 구간"의 시작. 08-11 실측에서 약 1.5s 면 twist 가 평탄해졌다.
CRUISE_START_S = 1.5
# 🔴 "움직이는 중"의 판정선. `/odom` 정지 잡음보다 크고 최저 순항속도(약 0.09)보다 훨씬 작다.
MOVE_EPS_MPS = 0.002
# 움직임 한 덩어리 안에서 허용하는 표본 간격.
# 🔴 08-12 정정 — 시간 공백 하나만으로 "다른 시행"이라고 끊으면 안 된다.
# 실측(`r1_0812_1612`): 주행 도중 기록이 **0.343초 멈췄다가** 밀린 표본 10건을 같은
# 시각으로 한꺼번에 토했다. 속도는 그 공백을 사이에 두고 `0.0513 → 0.0524` 로 명백히
# 이어지는데도 구판은 거기서 뒤로 걷기를 멈췄고, 창이 **2.08초 짧아져** 배율이 `0.818`
# 로 나왔다(줄자 568mm vs odom 464.7mm). 로봇이 아니라 기록이 딸꾹질한 것이다.
# → 판정을 둘로 나눈다: **정지 표본**은 여전히 시행 경계다(그게 진짜 경계다).
#   시간 공백은 **속도가 함께 끊겼을 때만** 경계로 본다.
MOTION_GAP_S = 0.3
# 이보다 긴 공백은 속도가 이어져 보여도 끊는다 — 그 사이에 서 있었는지 알 방법이 없다.
MOTION_GAP_HARD_S = 2.0
# 공백을 사이에 둔 두 표본의 속도차가 이보다 크면 "함께 끊겼다"로 본다.
VEL_CONTINUOUS_MPS = 0.01


def motion_start_before(odoms, anchor_ns):
    """`anchor_ns` 시점에 **이미 움직이고 있었다면** 그 움직임이 시작된 표본을 돌려준다.

    🔴 **왜 필요한가** (08-12 실측): 창을 `/cmd_vel` 첫 비영으로 잡으면, rosbag2 가
    `ros2 topic pub` 의 **발행자를 발견하기까지 걸린 시간**만큼 앞부분 명령이 bag 에
    안 들어왔을 때 창이 늦게 시작한다. 그런데 줄자가 재는 것은 **시작 표시부터 최종
    정지 위치까지**라 창과 줄자가 **다른 구간**이 되고, 스케일이 조용히 틀어진다.
    실측: 앞 1.45초가 빠져 배율이 `0.753` 으로 나왔고, 움직임 구간으로 맞추자
    `0.966` 이었다. 🔴 5% 가드가 이걸 잡아 줬지만 가드는 경보이지 정정이 아니다.

    anchor 에서 로봇이 서 있었으면(정상 순서) `None` — 그때는 창을 옮길 이유가 없다.
    """
    i = None
    for k, o in enumerate(odoms):
        if o[0] > anchor_ns:
            break
        i = k
    if i is None or abs(odoms[i][4]) <= MOVE_EPS_MPS:
        return None
    while i > 0:
        dt_ns = odoms[i][0] - odoms[i - 1][0]
        # 정지 표본 = 진짜 시행 경계. 이건 무조건 끊는다.
        if abs(odoms[i - 1][4]) <= MOVE_EPS_MPS:
            break
        # 너무 긴 공백은 그 사이를 알 수 없으므로 끊는다.
        if dt_ns > MOTION_GAP_HARD_S * 1e9:
            break
        # 짧은 공백은 **속도까지 끊겼을 때만** 경계로 본다 (기록 딸꾹질 면역).
        if dt_ns > MOTION_GAP_S * 1e9 and \
                abs(odoms[i][4] - odoms[i - 1][4]) > VEL_CONTINUOUS_MPS:
            break
        i -= 1
    return i


class UsageError(Exception):
    """입력 계약 위반. rc=2 로 끝난다."""


def load(bag):
    """`/cmd_vel`·`/odom`·`/imu/yaw_deg` 를 읽는다. ROS 의존은 여기에만 둔다."""
    import rosbag2_py                                     # noqa: PLC0415
    from rclpy.serialization import deserialize_message   # noqa: PLC0415
    from rosidl_runtime_py.utilities import get_message   # noqa: PLC0415

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if '/odom' not in tmap or '/cmd_vel' not in tmap:
        raise UsageError(f'bag 에 /odom·/cmd_vel 이 다 있어야 한다: {sorted(tmap)}')

    cmds, odoms, imu = [], [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        m = deserialize_message(data, get_message(tmap[topic]))
        if topic == '/cmd_vel':
            cmds.append((t, m.linear.x, m.angular.z))
        elif topic == '/odom':
            p = m.pose.pose.position
            q = m.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
            odoms.append((t, p.x, p.y, yaw, m.twist.twist.linear.x))
        elif topic == '/imu/yaw_deg':
            imu.append((t, m.data))
    return cmds, odoms, imu


def analyze(cmds, odoms, imu=None, tape_mm=None):
    """순수 함수 — bag I/O 없이 판정한다. 회귀는 여기에 합성 열을 넣는다."""
    nz = [c for c in cmds if abs(c[1]) > 1e-9 or abs(c[2]) > 1e-9]
    if not nz:
        return {'ok': False, 'reason': '비영 /cmd_vel 이 없다 — 무장이 안 됐을 수 있다'}
    t0, t1 = nz[0][0], nz[-1][0]

    # 🔴 창의 시작은 **움직이기 시작한 시점**이다 — 줄자와 같은 구간을 재기 위해서다.
    # 기록된 첫 명령 시점에 이미 굴러가고 있었다면 bag 이 명령 앞부분을 놓친 것이므로
    # 그 움직임의 앞끝까지 되돌아간다(근거 = `motion_start_before` 머리말).
    mi = motion_start_before(odoms, t0)
    w0 = odoms[mi][0] if mi is not None else t0
    lead_s = (t0 - w0) / 1e9

    seg = [o for o in odoms if w0 <= o[0] <= t1 + COAST_TAIL_S * 1e9]
    if len(seg) < 2:
        return {'ok': False, 'reason': f'판정 구간의 /odom 표본이 {len(seg)}개뿐이다'}
    # 꼬리의 정지 표본은 창에서 뺀다 — 안 그러면 관측 시간만 늘어 평균속도가 흐려진다.
    last_move = max((k for k, o in enumerate(seg) if abs(o[4]) > MOVE_EPS_MPS), default=None)
    if last_move is not None and last_move >= 1:
        seg = seg[:last_move + 1]

    x0, y0, yaw0 = seg[0][1], seg[0][2], seg[0][3]
    dx, dy = seg[-1][1] - x0, seg[-1][2] - y0
    # 시작 방향을 x축으로 놓고 종/횡을 가른다.
    fwd = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    lat = -dx * math.sin(yaw0) + dy * math.cos(yaw0)

    path = sum(math.hypot(seg[i][1] - seg[i - 1][1], seg[i][2] - seg[i - 1][2])
               for i in range(1, len(seg)))
    dur_seg = (seg[-1][0] - seg[0][0]) / 1e9
    dyaw = math.degrees((seg[-1][3] - yaw0 + math.pi) % (2 * math.pi) - math.pi)

    # 정상구간도 같은 시작점에서 센다. 🔴 끝은 **명령 종료**다 — 관성 꼬리를 평균에
    # 넣으면 순항속도가 실제보다 낮게 나온다(08-12 에 그 실수를 한 번 했다).
    cruise = [o[4] for o in odoms
              if w0 + CRUISE_START_S * 1e9 <= o[0] <= t1]

    v = {
        'ok': True,
        'cmd_linear': nz[0][1],
        'cmd_angular': nz[0][2],
        'n_nonzero': len(nz),
        'cmd_dur_s': (t1 - t0) / 1e9,
        # 🔴 bag 이 명령 앞부분을 놓친 양. 0 보다 크면 기록 결손이지 로봇 이상이 아니다.
        'cmd_lead_s': lead_s,
        'obs_dur_s': dur_seg,
        'n_odom': len(seg),
        'path_mm': path * 1000.0,
        'fwd_mm': fwd * 1000.0,
        'lat_mm': lat * 1000.0,
        'lat_pct': abs(lat) / max(abs(fwd), 1e-9) * 100.0,
        'dyaw_deg': dyaw,
        'avg_mps': path / dur_seg if dur_seg > 0 else float('nan'),
        'cruise_mps': (sum(cruise) / len(cruise)) if cruise else None,
    }

    if imu:
        pre = [d for t, d in imu if t <= t0]
        post = [d for t, d in imu if t >= t1]
        if pre and post:
            v['imu_dyaw_deg'] = post[-1] - pre[-1]

    # 🔴 줄자가 있으면 그것이 정본이다 — odom 은 여기서 **검증받는 쪽**이다.
    if tape_mm is not None:
        v['tape_mm'] = tape_mm
        v['odom_over'] = path * 1000.0 / tape_mm if tape_mm > 0 else float('nan')
        v['true_mps'] = (tape_mm / 1000.0) / dur_seg if dur_seg > 0 else float('nan')
        # 🔴 08-12 신설 — 평균속도와 순항속도는 **약점이 서로 반대**다.
        #   · 평균(`true_mps`) 은 줄자에 앵커돼 스케일은 믿을 수 있지만, 창에 출발
        #     가속과 관성 꼬리가 같이 들어가 **아래로 희석된다**. 짧은 주행일수록 심하다
        #     (실측 `0.12` 기준 10초 −7.5% · 20초 −3.8% · 25초 −2.9%).
        #   · 순항(`cruise_mps`) 은 희석이 없지만 odom·twist 둘 다 **같은 엔코더 파생**이라
        #     스케일을 스스로 검증하지 못한다.
        # 두 약점은 곱하면 상쇄된다: 순항 × (줄자/odom) = 외부에 앵커된 순항속도.
        # ⚠ 그래도 `cruise_mps` 는 EMA(α=0.10) 파생이라, 순항이 짧으면 이 값도 덜 앉는다.
        #
        # 🔴 08-13 버그 수정. 이 줄은 `odom_over` 를 **곱하고** 있었는데
        #    `odom_over = odom/줄자` 이므로 곱하면 `순항 × (odom/줄자)` — 위 주석이
        #    말하는 것의 정확히 역수다. odom 이 부풀어 있을수록 보정값이 더 부풀었다.
        #    실해: 08-13 직진에서 실제 0.0976 m/s 를 **0.1495 m/s** 로 보고했다.
        #    (= 같은 시행의 평균속도보다 1.62배 빠른 값. 물리적으로 불가능한 수였는데도
        #      부호가 그럴듯해 보여 한 번 지나갔다.)
        #    0.12 상한 판정에 쓰는 수라 방향이 **위험한 쪽**이다 — 실제로는 안 넘었는데
        #    넘었다고 읽거나, 보정 계수가 반대면 넘었는데 안 넘었다고 읽는다.
        if v.get('cruise_mps') is not None and v['odom_over'] == v['odom_over'] \
                and v['odom_over'] > 0:
            v['cruise_true_mps'] = v['cruise_mps'] / v['odom_over']
        else:
            v['cruise_true_mps'] = None
    return v


def report(v, name=''):
    print('=' * 74)
    print('지면 주행 실측:', name)
    if not v['ok']:
        print(f'  🔴 판정 불가 — {v["reason"]}')
        return
    print(f'  명령 linear={v["cmd_linear"]:.3f} angular={v["cmd_angular"]:.3f} · '
          f'비영 {v["n_nonzero"]}건 · 발행 {v["cmd_dur_s"]:.2f}s · '
          f'관측 {v["obs_dur_s"]:.2f}s(정지까지) · /odom {v["n_odom"]}표본')
    if v['cmd_lead_s'] > 0.2:
        print(f'  ⚠ bag 이 명령 앞 {v["cmd_lead_s"]:.2f}s 를 못 받았다 '
              f'(rosbag2 발행자 발견 지연) — 창을 움직임 시작으로 되돌려 잡았다.')
        print('     🔴 로봇 이상이 아니라 기록 결손이다. 그대로 뒀으면 줄자와 다른 '
              '구간을 재서 배율이 틀어진다.')
    print()
    print(f'  경로장(odom)        = {v["path_mm"]:9.1f} mm')
    print(f'  전진 성분           = {v["fwd_mm"]:9.1f} mm')
    print(f'  🔴 횡편차           = {v["lat_mm"]:9.1f} mm   ({v["lat_pct"]:.2f}% of 전진)')
    print(f'  yaw 변화(엔코더)    = {v["dyaw_deg"]:9.2f} °')
    if 'imu_dyaw_deg' in v:
        d = v['imu_dyaw_deg']
        gap = abs(d - v['dyaw_deg'])
        flag = '✅ 교차 일치' if gap < 1.0 else '🔴 두 관측자가 어긋난다'
        print(f'  yaw 변화(IMU 독립)  = {d:9.2f} °   차이 {gap:.2f}° {flag}')
    print()
    print(f'  평균속도(경로장)    = {v["avg_mps"]:9.4f} m/s'
          f'   ⚠ 가감속 포함 — 정상속도가 아니다')
    if v['cruise_mps'] is not None:
        print(f'  twist.x 정상구간    = {v["cruise_mps"]:9.4f} m/s'
              f'   ⚠ EMA(α=0.10) 파생값이라 보조 관측이다')
    if 'tape_mm' in v:
        print()
        print(f'  🔴 줄자(정본)       = {v["tape_mm"]:9.1f} mm')
        print(f'     odom / 줄자      = {v["odom_over"]:9.3f} 배')
        print(f'     실제 평균속도    = {v["true_mps"]:9.4f} m/s'
              f'   ← 명령의 {v["true_mps"] / max(abs(v["cmd_linear"]), 1e-9):.2f}배')
        if v.get('cruise_true_mps') is not None:
            print(f'     줄자앵커 순항    = {v["cruise_true_mps"]:9.4f} m/s'
                  f'   ← 명령의 '
                  f'{v["cruise_true_mps"] / max(abs(v["cmd_linear"]), 1e-9):.2f}배')
            print('     ⚠ 평균속도는 가감속이 섞여 **아래로 희석된다**(짧은 주행일수록 심함).')
            print('       이 줄은 순항 × (줄자/odom) 이라 희석이 없고 스케일도 외부 앵커다.')
        if abs(v['odom_over'] - 1.0) > 0.05:
            print('     🔴 odom 과 줄자가 5% 넘게 어긋난다 — 오도메트리 스케일을 의심한다')
        else:
            print('     ✅ 오도메트리 스케일 정상(5% 이내)')
    print()
    print('  🔴 실제 이동 거리의 정본은 줄자다 — odom·twist 는 같은 엔코더 파생이라')
    print('     서로를 검증하지 못한다. 표시하고 재는 것이 유일한 외부 관측이다.')


def main(argv):
    args = argv[1:]
    tape = None
    if '--tape-mm' in args:
        i = args.index('--tape-mm')
        try:
            tape = float(args[i + 1])
        except (IndexError, ValueError):
            print('입력 오류 — --tape-mm 뒤에 수를 준다', file=sys.stderr)
            return 2
        del args[i:i + 2]
    if len(args) != 1:
        print(__doc__.split('사용법:')[1].strip(), file=sys.stderr)
        return 2

    try:
        cmds, odoms, imu = load(args[0])
    except UsageError as exc:
        print(f'입력 오류 — {exc}', file=sys.stderr)
        return 2
    except Exception as exc:                              # noqa: BLE001
        print(f'판정 불가 — bag 을 읽지 못했다: {exc}', file=sys.stderr)
        return 2

    v = analyze(cmds, odoms, imu, tape)
    report(v, args[0].rstrip('/').split('/')[-1])
    return 0 if v['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
