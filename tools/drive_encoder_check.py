#!/usr/bin/env python3
"""바퀴별 엔코더 기여 분해 — 손으로 한 바퀴씩 굴린 bag 을 정지 구간으로 자른다.

★ **무엇을 가르는 도구인가** (2026-08-11 R1 에서 이 고장을 실제로 잡았다)

`d0_check` 검사 6 은 바퀴를 손으로 굴려 **엔코더 부호**를 본다. 그런데 그 검사는
*부호가 있는가*만 보지 **그 부호가 어느 쪽 바퀴로 적분되는가**는 안 본다. 펌웨어는

    deltaLeft  = 0.5*(dFL + dRL)      deltaRight = 0.5*(dFR + dRR)
    deltaYaw   = (deltaRight - deltaLeft) / WHEEL_BASE

로 좌우를 평균하므로, **좌측 한 바퀴의 부호가 뒤집히면 그 바퀴가 나머지 좌측 바퀴를 지운다.**
`deltaLeft = 0` 이 되어 **거리는 절반이 되고 없던 회전이 생긴다.** 2026-08-11 실측에서
5초 직진에 `69.5°` 가 적분됐는데 **로봇은 눈으로 보기에 완벽히 직진했다** — 네 바퀴가
지면으로 묶여 있어 빠른 바퀴는 미끄러질 뿐이라 **육안으로는 절대 안 잡힌다.**

🔴 **그리고 오도메트리만의 문제가 아니다.** PI 는 `direction * filteredWheelVelocity` 로
오차를 만들기 때문에 부호가 반대면 `error = target - (-v) = target + v` 가 되어 **속도가
붙을수록 오차가 커진다.** 적분기가 감겨 그 바퀴만 PWM 천장으로 간다.

★ **판정 원리** — 바퀴 하나만 굴리면 그 구간에서

    Δdist = 0.25 * d_i          Δyaw = ±0.5 * d_i / WHEEL_BASE   (좌 −, 우 +)

이므로 **`Δyaw` 의 부호가 그 바퀴가 좌/우 어느 쪽으로 계산되는지를 그대로 보여준다.**
회전수를 정확히 셀 필요가 없다 — 부호만 보면 범인이 나온다.

★ **찍는 법** (이걸 어기면 구간이 안 갈린다)

  - 바퀴를 띄우고 **무장 해제**한 뒤 `ros2 bag record /odom -o enc_check_$(date +%m%d_%H%M)`
  - **한 번에 한 바퀴만** 전진 방향으로 굴린다. 손이 다른 바퀴에 닿으면 그 구간은 못 쓴다
  - **사이에 3초 이상 완전 정지.** 이 정지가 유일한 구간 분리 수단이다
  - 순서 = 좌전 → 좌후 → 우전 → 우후 (기대: 앞 둘 음수, 뒤 둘 양수)

사용법:
    python3 tools/drive_encoder_check.py <bag> [--pre-plate] [--wheels=FL,RL,FR,RR]
    종료코드 0 = 네 구간 전부 기대 부호 / 1 = 부호 이상·구간 수 불일치 / 2 = 입력 오류

    `--wheels=` = 굴린 바퀴만 **굴린 순서대로** 준다 (기본 = 네 바퀴 전부).
    🔴 **엔코더가 죽은 바퀴는 굴려도 구간이 안 생긴다** — `/odom` 이 미동도 안 하기
    때문이다. 그러면 네 바퀴를 굴려도 구간은 3개가 되고, 우전·우후는 기대 부호가
    같아 남은 부호만으로 둘을 못 가른다. **한 바퀴만 굴린 bag 을 주면** 그 모호함이
    사라진다 — 구간 0개가 곧 "그 바퀴가 범인" 이다:

        ros2 bag record /odom -o enc_FR_$(date +%m%d_%H%M)   # 우전륜만 굴린다
        python3 tools/drive_encoder_check.py enc_FR_... --wheels=FR

    `--pre-plate` = 08-13 상판 판재를 얹기 **전** 에 찍은 bag 을 그때의 상수
    (윤거 0.62 · 반지름 0.05698)로 환산한다. 옛 증거를 다시 볼 때만 쓴다.
    🔴 부호 판정에는 영향이 없다 — 회전수 숫자만 바뀐다.

정본 = `docs/JETSON_SETUP.md §7-c-R1` · 함정 = `docs/PITFALLS.md §11`.
짝 도구 = `tools/drive_ground_report.py`(지면 주행 실측).
"""
import math
import sys

# ── 정본 상수 ────────────────────────────────────────────────────────────
# 🔴 이 도구는 `/odom` 의 yaw 를 되짚어 "그 바퀴가 몇 바퀴 굴렀나" 를 낸다. 그러니
# 여기 들어갈 값은 **odom 계열**이다 — 명령 경로의 0.62 도, 제어 피드백의 0.05698 도
# 아니다. 08-13 검토 §65.3 이 이 자리를 짚었다: 구판은 두 계열을 구분하지 않고
# "펌웨어와 같은 값" 이라고만 써 두어, 펌웨어가 갈라진 뒤 조용히 약 25% 틀린 바퀴
# 회전수를 보고했다.
#
#   물리 0.49  = 줄자로 잰 실제 간격 (URDF 몫). 여기 안 쓴다.
#   명령 0.62  = cmd_vel -> 바퀴 목표 (`.ino` CMD_WHEEL_BASE). 여기 안 쓴다.
#   odom 0.829 = 엔코더 -> yaw (`.ino` ODOM_WHEEL_BASE). 🔴 이 도구가 쓰는 값.
#     ⚠ 08-21 — 이 줄이 `0.670` 에 멈춰 있었다. 아래 상수는 0.829 로 맞았으니
#       계산은 옳았고 **읽는 사람만 속았다.** 값을 옮길 땐 주석도 같이 옮긴다.
#
# 바꿀 일이 생기면 `.ino` 를 먼저 고치고 여기를 따라 옮긴다. 보드가 실제로 뭘 들고
# 있는지는 `/firmware/info` 의 `odom_wheel_base=` · `odom_wheel_radius=` 로 대조한다.
# 🔴 2026-08-22 재교정 0.829 -> 0.859 (`.ino` 와 한 쌍. 회귀가 `.ino` 에서 읽어 대조한다).
#   근거 = 08-22 실측: 줄자 직진 2회가 반지름을 지지(0.9936·0.9919) · 제자리 회전
#   3회에서 odom/IMU = 1.0431. 정본 = docs/REAL_ROBOT_VALUES.md §1-c.
ODOM_WHEEL_BASE_M = 0.859
ODOM_WHEEL_RADIUS_M = 0.05698

# 🔴 판재 이전(08-12 까지) 증거를 다시 환산할 때 쓰는 옛 값. **현재값이 아니다.**
# 검토 §65.3 "역사 실측 문장을 새 값으로 일괄 덮어쓰지 않는다" 에 따른 자리다.
# `--pre-plate` 로 고른다.
PRE_PLATE_WHEEL_BASE_M = 0.62
# 🔴 08-13 밤 — 구름 반지름은 판재 전후가 같다(예약 32-e). 윤거만 갈린다.
PRE_PLATE_WHEEL_RADIUS_M = ODOM_WHEEL_RADIUS_M

# 굴림 구간을 가르는 기준. 촬영 규격의 "3초 이상 정지"보다 넉넉히 짧게 잡아 사람이
# 조금 빨리 움직여도 구간이 붙지 않게 한다.
QUIET_S = 1.5
# 표본 간 이만큼 넘게 변하면 "굴리는 중"으로 본다. odom 잡음 바닥보다 위다.
MOVE_EPS_M = 0.0005
MOVE_EPS_RAD = 0.002
# 한 구간이 이 각도조차 못 만들면 "반응 없음"이다 — 굴렸는데 안 잡힌 바퀴를 찾는다.
DEAD_DEG = 1.0
# 기대 부호. 좌측 둘은 음수, 우측 둘은 양수여야 한다.
EXPECTED_SIGNS = (-1, -1, +1, +1)
WHEEL_NAMES = ('좌전륜', '좌후륜', '우전륜', '우후륜')
# `--wheels=` 가 쓰는 이름. 펌웨어의 FL/RL/FR/RR 순서와 같다(`.ino` deltaLeft/Right).
WHEEL_KEYS = ('FL', 'RL', 'FR', 'RR')


class UsageError(Exception):
    """입력 계약 위반. 🔴 traceback 이 아니라 원인 + rc=2 로 끝난다."""


def load(bag):
    """bag 에서 `/odom` 의 (t, x, y, yaw) 를 읽는다. ROS 의존은 이 함수 안에만 둔다."""
    import rosbag2_py                                     # noqa: PLC0415
    from rclpy.serialization import deserialize_message   # noqa: PLC0415
    from rosidl_runtime_py.utilities import get_message   # noqa: PLC0415

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if '/odom' not in tmap:
        raise UsageError(f'bag 에 /odom 이 없다: {sorted(tmap)}')

    rows = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != '/odom':
            continue
        m = deserialize_message(data, get_message(tmap[topic]))
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        rows.append((t, p.x, p.y, yaw))
    return rows


def wrap_deg(rad):
    """−180~180 으로 접는다. 한 바퀴를 넘게 굴리면 이 값은 못 쓴다(아래 경고 참조)."""
    return math.degrees((rad + math.pi) % (2 * math.pi) - math.pi)


def segments(rows, quiet_s=QUIET_S):
    """움직인 표본만 남기고 조용한 틈으로 자른다. 반환 = 인덱스 묶음 리스트."""
    moving = []
    for i in range(1, len(rows)):
        d = math.hypot(rows[i][1] - rows[i - 1][1], rows[i][2] - rows[i - 1][2])
        dy = abs(wrap_deg(rows[i][3] - rows[i - 1][3]))
        if d > MOVE_EPS_M or math.radians(dy) > MOVE_EPS_RAD:
            moving.append(i)

    groups = []
    for i in moving:
        if groups and (rows[i][0] - rows[groups[-1][-1]][0]) / 1e9 < quiet_s:
            groups[-1].append(i)
        else:
            groups.append([i])
    return groups


def analyze(rows, groups, wheel_base_m=None, wheel_radius_m=None, wheels=None):
    """순수 함수 — bag I/O 없이 판정한다. 회귀는 여기에 합성 열을 넣는다.

    상수는 인자로 받는다 (검토 §65.3). 모듈 상수를 직접 읽으면 회귀가 그 값을
    정답으로 굳혀, 펌웨어가 바뀐 뒤에도 스스로에게 초록을 준다.
    """
    if wheel_base_m is None:
        wheel_base_m = ODOM_WHEEL_BASE_M
    if wheel_radius_m is None:
        wheel_radius_m = ODOM_WHEEL_RADIUS_M

    if wheels is None:
        wheels = list(range(len(WHEEL_KEYS)))

    if len(rows) < 2:
        return {'ok': False, 'reason': f'/odom 표본이 {len(rows)}개뿐이다',
                'wheels': wheels}
    if len(groups) != len(wheels):
        # 🔵 08-21 — 여기가 막혀 있었다. 엔코더가 **죽으면** 그 바퀴를 굴려도 `/odom` 이
        #   전혀 안 움직여 **구간이 아예 안 생긴다.** 그래서 네 바퀴를 굴려도 구간은 3개가
        #   되고, 도구는 "3개다"라고만 하고 멈췄다 — 정작 알고 싶은 **범인 이름**을 못 냈다.
        #   게다가 우전·우후는 기대 부호가 같아, 남은 부호만으로는 둘을 못 가른다.
        #   한 바퀴만 굴린 bag 이면 그 모호함이 없다. **침묵 자체가 판정이다.**
        if len(wheels) == 1 and not groups:
            return {'ok': False, 'wheels': wheels, 'bad': [wheels[0]],
                    'verdicts': ['dead'],
                    'rows': [{'dur_s': 0.0, 'ddist_mm': 0.0, 'dyaw_deg': 0.0,
                              'wheel_mm': 0.0, 'turns': 0.0}]}
        hint = ('' if len(wheels) == 1 else
                '  🔵 죽은 바퀴를 특정하려면 **한 바퀴씩 따로 찍고** '
                '`--wheels=FR` 처럼 하나만 준다.')
        return {'ok': False, 'wheels': wheels,
                'reason': f'굴림 구간이 {len(groups)}개인데 바퀴는 {len(wheels)}개다 — '
                          f'한 번에 하나씩, 사이에 {QUIET_S}s 이상 정지하며 굴린다.' + hint}

    out = []
    for g in groups:
        a, b = rows[g[0] - 1], rows[g[-1]]
        dur = (b[0] - a[0]) / 1e9
        ddist = math.hypot(b[1] - a[1], b[2] - a[2])
        dyaw = wrap_deg(b[3] - a[3])
        # 한 바퀴만 굴렸다는 전제에서 그 바퀴가 실제로 간 거리
        d_wheel = abs(math.radians(dyaw)) * wheel_base_m / 0.5
        out.append({
            'dur_s': dur,
            'ddist_mm': ddist * 1000.0,
            'dyaw_deg': dyaw,
            'wheel_mm': d_wheel * 1000.0,
            'turns': d_wheel / (2 * math.pi * wheel_radius_m),
        })

    verdicts, bad = [], []
    for row, wi in zip(out, wheels):
        want = EXPECTED_SIGNS[wi]
        if abs(row['dyaw_deg']) < DEAD_DEG:
            v = 'dead'
            bad.append(wi)
        elif (row['dyaw_deg'] < 0) == (want < 0):
            v = 'ok'
        else:
            v = 'flipped'
            bad.append(wi)
        verdicts.append(v)

    return {'ok': not bad, 'rows': out, 'verdicts': verdicts, 'bad': bad,
            'wheels': wheels}


def report(v):
    print('=' * 76)
    if not v['ok'] and 'rows' not in v:
        print(f'  🔴 판정 불가 — {v["reason"]}')
        return
    names = [WHEEL_NAMES[i] for i in v.get('wheels', range(len(WHEEL_NAMES)))]
    print('  #  바퀴      지속(s)  Δdist(mm)  Δyaw(deg)  바퀴이동(mm)  회전수  판정')
    for i, (r, name, verdict) in enumerate(zip(v['rows'], names, v['verdicts']), 0):
        mark = {'ok': '✅ 정상',
                'flipped': '🔴 부호 반전',
                'dead': '🔴 반응 없음'}[verdict]
        print(f'  {i + 1}  {name}  {r["dur_s"]:7.1f} {r["ddist_mm"]:10.1f} '
              f'{r["dyaw_deg"]:10.2f} {r["wheel_mm"]:12.0f} {r["turns"]:7.1f}  {mark}')
    print()
    if v['ok']:
        print('  ✅ 굴린 바퀴 전부가 기대한 쪽으로 적분됐다.'
              if len(v.get('wheels', ())) < len(WHEEL_NAMES) else
              '  ✅ 좌측 2개 음수 · 우측 2개 양수 — 부호 배치가 정상이다.')
    else:
        bad = ', '.join(WHEEL_NAMES[i] for i in v['bad'])
        dead = 'dead' in v['verdicts']
        print(f'  🔴 **{bad}** 가 기대한 값을 내지 않는다.')
        if dead:
            # 🔴 08-21 실차 — 오른쪽 계수가 0.525(≈절반)로 나왔다. 한쪽당 엔코더 2개를
            #   평균하므로(`.ino` deltaRight = 0.5*(dFR+dRR)) **하나가 0 이면 그 쪽이 절반**이 된다.
            #   그 절반은 직진·회전에 같은 배율로 나타나고, 직진 명령만 줘도 없는 회전을 만든다.
            print('     반응 없음 = 그 바퀴의 카운트가 안 들어온다. 굽기 전에 **배선부터** 본다:')
            print('     ① 엔코더 커넥터가 빠졌나 ② A/B·전원·GND 가 헐거운가 '
                  '③ 커플러가 축에서 놀아 엔코더만 안 도나')
            print('     🔵 배선이면 다시 꽂는 것으로 끝난다 — 펌웨어를 굽지 않는다.')
        else:
            print('     부호 반전은 그 바퀴의 엔코더 A/B 배선을 서로 바꿔 고친다 — '
                  '🔴 `ENCODER_POLARITY` 를 고쳐 다시 구우면')
            print('     `§7-c-E` 13행 + `§5-G6` 10회 재시험이 따라온다.')
    print()
    print('  ⚠ 회전수는 굴린 실제 회전수와 맞아야 한다 — 크게 벗어나면 계수 상수를 본다.')
    print('  ⚠ 한 바퀴(360°)를 넘게 굴리면 Δyaw 가 접혀 회전수가 작게 나온다. '
          '부호 판정에는 영향이 없다.')


def main(argv):
    args = [a for a in argv[1:] if not a.startswith('--')]
    flags = [a for a in argv[1:] if a.startswith('--')]
    unknown = [f for f in flags
               if f != '--pre-plate' and not f.startswith('--wheels=')]
    if len(args) != 1 or unknown:
        if unknown:
            print(f'모르는 옵션: {" ".join(unknown)}', file=sys.stderr)
        print(__doc__.split('사용법:')[1].strip(), file=sys.stderr)
        return 2

    # 🔴 판재 이전 bag 은 그때의 상수로 환산해야 그때 본 수가 재현된다 (검토 §65.3).
    wheels = None
    picked = [f for f in flags if f.startswith('--wheels=')]
    if picked:
        keys = [k.strip().upper()
                for k in picked[-1].split('=', 1)[1].split(',') if k.strip()]
        bad = [k for k in keys if k not in WHEEL_KEYS]
        if bad or not keys or len(set(keys)) != len(keys):
            print(f'--wheels 가 잘못됐다: {picked[-1]} — '
                  f'{"/".join(WHEEL_KEYS)} 중에서 중복 없이 고른다', file=sys.stderr)
            return 2
        wheels = [WHEEL_KEYS.index(k) for k in keys]

    pre_plate = '--pre-plate' in flags
    base = PRE_PLATE_WHEEL_BASE_M if pre_plate else ODOM_WHEEL_BASE_M
    radius = PRE_PLATE_WHEEL_RADIUS_M if pre_plate else ODOM_WHEEL_RADIUS_M

    try:
        rows = load(args[0])
    except UsageError as exc:
        print(f'입력 오류 — {exc}', file=sys.stderr)
        return 2
    except Exception as exc:                              # noqa: BLE001
        print(f'판정 불가 — bag 을 읽지 못했다: {exc}', file=sys.stderr)
        return 2

    print('엔코더 바퀴별 분해:', args[0].rstrip('/').split('/')[-1])
    print(f'  /odom {len(rows)}표본')
    print(f'  환산 상수 = odom 윤거 {base:.3f} m · odom 반지름 {radius:.5f} m'
          + ('  🔴 판재 이전(--pre-plate)' if pre_plate else ''))
    if wheels is not None:
        print('  굴린 바퀴 = ' + ', '.join(WHEEL_NAMES[i] for i in wheels)
              + '  (순서가 굴린 순서와 같아야 한다)')
    v = analyze(rows, segments(rows), base, radius, wheels)
    report(v)
    return 0 if v['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
