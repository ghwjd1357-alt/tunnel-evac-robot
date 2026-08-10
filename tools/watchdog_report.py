#!/usr/bin/env python3
"""R0 watchdog(`TODO(D+0) #11`) 판정용 — bag 에서 정지 시각을 기계로 재계산한다.

정본 = `docs/JETSON_SETUP.md §7-c-0`.

★ 이 도구는 §7-c-0 의 **1차 증거(영상)를 대체하지 않는다.** `/odom.pose` 는 펌웨어가
엔코더를 적분해 만든 값이라, 지금 시험 대상인 그 펌웨어와 독립적인 관측이 아니다.
영상이 없을 때 **교차 증거를 기계 타임스탬프로 재현**하는 용도이며, 결과를 쓸 때는
"영상 미분석"을 함께 적는다.

사용법:
    python3 tools/watchdog_report.py <bag 디렉터리> [...]
    python3 tools/watchdog_report.py ~/Desktop/d0_evidence/d0_watchdog_*
    종료코드 0 = 전 bag 유효 / 1 = 하나라도 판정 불능 / 2 = 사용법·입력 오류

🔴 **2026-08-10 검토 §52 보완 — 구판의 두 거짓 판정을 고쳤다.** 구판이 낸
`519.9 / 532.0 / 537.1 ms` 는 그 결함 위의 값이므로 **인용하지 않는다**(`§7-c-0` 정본).

  §52.1 **zero 개입 구간을 판정에서 배제한다.** 구판은 "마지막 비영 명령 뒤 zero 가
     들어왔다"고 **경고만 하고 그 뒤 이동까지 watchdog 결과로 계산**했다. zero 로 멈춘
     것은 *명령을 받아서* 선 것이지 watchdog 이 세운 것이 아니다. 이제 판정 구간을
     `[마지막 비영 명령, min(첫 zero, bag 끝)]` 으로 자르고, 그 안에서 마지막 이동 뒤
     **2초 관찰**을 못 채우면 수치를 내지 않고 **판정 불능**으로 끝낸다.
     ⚠ 출력용 2600ms 절단과 **판정용 구간은 다른 것**이다 — 섞지 않는다.

  §52.2 **정지 판정을 표본 간 증분이 아니라 고정 창 안의 속도로 한다.** 구판은 연속 두
     표본의 거리만 `0.5mm` 와 비교해서 **0.4mm 씩 130 표본(총 52mm)** 을 전부 "정지"로
     읽었다 — watchdog 고장의 대표 증상인 **저속 creep 를 정확히 못 잡는** 판정기였다.
     ⚠ **그렇다고 누적 변위로 바꾸면 반대로 틀린다.** 구현 중 실측에서 확인했다: 1521 bag
     은 정지 **11초 뒤**부터 약 1.5초간 표본당 0.03~0.13mm 씩 **총 1.3mm**(≈1.2 mm/s)
     흐르고, 누적 판정은 그것을 "12.5초에 마지막 이동"이라고 불렀다. 로봇이 아니라 기구
     안착·엔코더 적분이며 같은 구간 `twist.x` 는 0.0018 m/s 다. 그래서 판정선을
     **속도(`MOTION_RATE_MM_S`)** 로 두고, 임계 하나에 수치가 흔들리는 것을 숨기지 않도록
     **민감도 표를 항상 함께 출력**한다.

  🔴 **재산출 결과(2026-08-10)** = `519.9 / 532.0 / 516.2 ms`. 앞의 둘은 구판과 우연히
     같고 **1522 만 537.1 → 516.2 로 바뀌었다.** 구판이 맞아서 같았던 것이 아니라
     **틀린 방법이 그 두 bag 에서 우연히 근처를 짚은 것**이다.

  §52.4 **판정 불능이면 종료코드가 0 이 아니다.** 구판은 `판정 불가` 를 출력하고도
     `rc=0` 을 냈다. 자동화가 성공으로 오인한다.
"""
import glob
import math
import os
import sys

# 엔코더 양자화 잡음과 실이동을 가르는 임계. 바퀴 반경 0.053 m 기준 충분히 작다.
MOVE_EPS_MM = 0.5
# 🔴 **잡음 바닥과 이동 판정선은 다른 값이다** — 검토 §52.2 가 분리하라고 한 자리다.
# 누적 변위만 보면 구판과 **반대 방향**으로 틀린다: 실측 1521 bag 은 정지 11초 뒤부터
# 약 1.5초간 표본당 0.03~0.13mm 씩 **총 1.3mm**(≈1.2 mm/s) 흐른다. 로봇이 움직인 것이
# 아니라 기구 안착·엔코더 적분이며, 같은 구간 `twist.x` 는 0.0018 m/s 다.
# 그래서 판정선을 **속도**로 둔다. 실제 감속은 표본당 2~5mm(=100~250 mm/s)로 압도적이라
# 아래 선과 두 자릿수 떨어져 있다.
MOTION_RATE_MM_S = 5.0     # 관측 주행속도 약 0.1 m/s 의 5%. 이보다 느리면 이동이라 안 한다
MOTION_WINDOW_MS = 200     # 그 속도를 재는 창. 창 안 변위 1.0mm 가 실판정 임계가 된다
# 판정선의 민감도를 항상 함께 낸다 — 임계 하나로 수치가 흔들리는 것을 숨기지 않는다.
SENSITIVITY_RATES_MM_S = (2.0, 5.0, 10.0, 20.0)
WATCHDOG_CONTRACT_MS = 500
# §7-c-0 조건 2 — 마지막 이동 뒤 이만큼은 관찰해야 "섰다"고 말할 수 있다.
REQUIRED_TAIL_MS = 2000
# 출력만 좁히는 값이다. 판정 구간과 혼동하지 않는다(§52.1).
PRINT_WINDOW_MS = (-150, 2600)


def load(bag):
    """bag 에서 `/cmd_vel`·`/odom` 을 읽는다. ROS 의존은 이 함수 안에만 둔다."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cmds, odoms = [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/cmd_vel':
            m = deserialize_message(data, get_message(tmap[topic]))
            cmds.append((t, m.linear.x, m.angular.z))
        elif topic == '/odom':
            m = deserialize_message(data, get_message(tmap[topic]))
            p = m.pose.pose.position
            odoms.append((t, p.x, p.y, m.twist.twist.linear.x))
    return cmds, odoms


def _undecidable(reason):
    return {'ok': False, 'reason': reason}


def last_motion_index(samples, rate_mm_s=MOTION_RATE_MM_S):
    """**고정 시간 창 안의 속도**로 이동을 판정하고, 마지막 이동 표본의 인덱스를 준다.

    구판(§52.2 이전)은 이웃 두 표본의 증분만 봐서 `0.4mm × 130 표본(52mm)` 을 전부
    "정지"로 읽었다. 그렇다고 **누적 변위**만 보면 반대로 틀린다 — 정지 한참 뒤의 느린
    기구 안착이 임계를 넘어 "이동"이 된다(실측 1521 = 11초 뒤 1.3mm).

    그래서 각 표본 `i` 에서 **이후 `MOTION_WINDOW_MS` 안**의 최대 변위를 보고, 그것이
    `rate_mm_s × 창` 을 넘으면 이동으로 센다. 지속되는 creep 는 창 안에서 바로 넘고
    (0.4mm/20ms = 20 mm/s), 느린 안착은 어느 창에서도 못 넘는다(1.2 mm/s).

    반환 = 이동으로 판정된 마지막 인덱스. `None` 이면 창 안에서 이동이 없었다.
    """
    window_ns = MOTION_WINDOW_MS * 1_000_000
    eps_m = rate_mm_s * MOTION_WINDOW_MS / 1000.0 / 1000.0
    n, last = len(samples), None
    for i in range(n):
        t_i, x_i, y_i, _ = samples[i]
        far, k = 0.0, i + 1
        while k < n and samples[k][0] - t_i <= window_ns:
            far = max(far, math.hypot(samples[k][1] - x_i, samples[k][2] - y_i))
            if far > eps_m:
                break
            k += 1
        if far > eps_m:
            last = i
    return last


def analyze(cmds, odoms):
    """순수 함수 — bag I/O 없이 판정한다. 회귀는 여기에 합성 입력을 넣는다."""
    if not cmds or not odoms:
        return _undecidable('필요한 토픽이 비었다')
    for t, x, y, vx in odoms:
        if not all(map(math.isfinite, (x, y))):
            return _undecidable('pose 에 NaN/Inf 가 있다')
    dur_ns = odoms[-1][0] - odoms[0][0]
    if dur_ns <= 0:
        return _undecidable('녹화 길이가 0 이하다')

    nonzero = [c for c in cmds if abs(c[1]) > 1e-9 or abs(c[2]) > 1e-9]
    if not nonzero:
        return _undecidable('비영 명령이 없다')
    last_cmd = nonzero[-1][0]

    # §52.1 — 판정 구간의 끝은 첫 zero 이거나 bag 끝이다. 둘 중 이른 쪽.
    zero_after = [z[0] for z in cmds
                  if abs(z[1]) <= 1e-9 and abs(z[2]) <= 1e-9 and z[0] > last_cmd]
    zero_at = min(zero_after) if zero_after else None
    cutoff = min(zero_at, odoms[-1][0]) if zero_at is not None else odoms[-1][0]

    window = [s for s in odoms if last_cmd <= s[0] <= cutoff]
    if len(window) < 2:
        return _undecidable('판정 구간에 /odom 표본이 2개 미만이다')

    idx = last_motion_index(window)
    if idx is None:
        return _undecidable('판정 구간에서 이동이 관측되지 않았다')
    # 이동 중으로 센 마지막 표본의 **다음** 표본이 정지 시각이다.
    idx = min(idx + 1, len(window) - 1)

    last_move_ms = (window[idx][0] - last_cmd) / 1e6
    tail_ms = (cutoff - window[idx][0]) / 1e6
    if tail_ms < REQUIRED_TAIL_MS:
        return _undecidable(
            f'마지막 이동 뒤 관찰이 {tail_ms:.0f} ms 뿐이다 '
            f'(§7-c-0 조건 2 = {REQUIRED_TAIL_MS}ms)')

    sens = {}
    for rate in SENSITIVITY_RATES_MM_S:
        k = last_motion_index(window, rate)
        sens[rate] = (None if k is None
                      else (window[min(k + 1, len(window) - 1)][0] - last_cmd) / 1e6)

    return {
        'ok': True,
        'sensitivity': sens,
        'last_move_ms': last_move_ms,
        'tail_ms': tail_ms,
        'over_ms': last_move_ms - WATCHDOG_CONTRACT_MS,
        'zero_gap_ms': None if zero_at is None else (zero_at - last_cmd) / 1e6,
        'window_n': len(window),
        'cmd_n': len(cmds),
        'odom_n': len(odoms),
        'duration_s': dur_ns / 1e9,
    }


def report(bag, verbose=True, load_fn=None):
    """한 bag 을 판정하고 사람이 읽을 출력을 낸다. 유효하면 dict, 아니면 None."""
    print('=' * 78)
    print('BAG:', os.path.basename(str(bag).rstrip('/')))
    try:
        cmds, odoms = (load_fn or load)(bag)
    except Exception as exc:                                  # noqa: BLE001
        print(f'  판정 불가 — bag 을 읽지 못했다: {exc}')
        return None
    print(f'  /cmd_vel {len(cmds)}건 · /odom {len(odoms)}건')

    verdict = analyze(cmds, odoms)
    if not verdict['ok']:
        print(f"  🔴 판정 불가 — {verdict['reason']}")
        return None

    if verdict['zero_gap_ms'] is not None:
        print(f"  ⚠ 마지막 비영 명령 뒤 {verdict['zero_gap_ms']:.0f} ms 에 zero 가 들어왔다 "
              f'— 판정 구간을 거기서 잘랐다(§52.1)')
    print(f"  녹화 {verdict['duration_s']:.1f}s · "
          f"/odom 평균 {verdict['odom_n'] / verdict['duration_s']:.1f} Hz · "
          f"판정 구간 표본 {verdict['window_n']}개")

    if verbose:
        last_cmd = [c for c in cmds if abs(c[1]) > 1e-9 or abs(c[2]) > 1e-9][-1][0]
        print('\n   t_rel(ms)      x(m)      y(m)   twist.x')
        for (t, x, y, vx) in odoms:
            rel = (t - last_cmd) / 1e6
            if not PRINT_WINDOW_MS[0] <= rel <= PRINT_WINDOW_MS[1]:
                continue
            mark = '  <-- 마지막 이동' if abs(rel - verdict['last_move_ms']) < 1e-6 else ''
            print(f'  {rel:9.1f}  {x:8.4f}  {y:8.4f}  {vx:8.4f}{mark}')

    print()
    print(f"  >>> 마지막 pose 이동 = +{verdict['last_move_ms']:.1f} ms "
          f"(계약 {WATCHDOG_CONTRACT_MS}ms 대비 {verdict['over_ms']:+.1f} ms)")
    print(f"  >>> 정지 후 관찰 창 = {verdict['tail_ms']:.0f} ms "
          f'(§7-c-0 조건 2 는 {REQUIRED_TAIL_MS}ms 이상)')
    band = ' · '.join(f"{r:g}mm/s→{'?' if v is None else f'{v:.1f}'}"
                      for r, v in sorted(verdict['sensitivity'].items()))
    print(f'  >>> 판정선 민감도: {band}   '
          f'(정본 = {MOTION_RATE_MM_S:g}mm/s. 흔들리면 영상이 1차 증거다)')
    return verdict


def main(argv):
    targets = argv[1:]
    if not targets:
        targets = sorted(glob.glob(os.path.expanduser(
            '~/Desktop/d0_evidence/d0_watchdog_*')))
    if not targets:
        print('bag 디렉터리를 인자로 주거나 ~/Desktop/d0_evidence/ 에 두어라', file=sys.stderr)
        return 2
    undecided = [bag for bag in targets if report(bag) is None]
    print()
    if undecided:
        # §52.4 — 판정 불능은 성공이 아니다. 자동화가 오인하지 않게 nonzero 로 끝낸다.
        print(f'FAIL 판정 불능 {len(undecided)}/{len(targets)} — '
              f"{', '.join(os.path.basename(str(b).rstrip('/')) for b in undecided)}",
              file=sys.stderr)
        return 1
    print(f'OK 전 bag 판정 유효 ({len(targets)}개)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
