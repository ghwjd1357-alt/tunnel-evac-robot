#!/usr/bin/env python3
"""피드포워드 계수 산출 — 지면 시행 여러 건에서 `FEEDFORWARD_PWM_PER_MPS_ABOVE_MIN` 을 뽑는다.

★ **왜 이 도구가 생겼나** (2026-08-12 검토 §60.1·§61.1)

구현자가 손으로 낸 `375` 는 두 번 지적받았다.

  ① **명목 FF 를 실제 PWM 으로 썼다.** `wheelVelocityToFeedforwardPwm()` 의 출력은
     피드포워드 **항**이고, 실제 출력은 `FF + Kp·error + Ki·integral` 을 더 거친다.
     보드가 `appliedPwm` 을 발행하지 않으므로 관측 쌍이 아니었다.
  ② **속도를 어느 창에서 뽑았는지 안 적었다.** `0.123 m/s` 는 bag 에서 유일하게
     재현되는 값이 아니었다 — 같은 bag 이 정상구간 `0.13351`, 줄자 기반 평균 `0.1132` 를
     동시에 갖는다.

→ 그래서 **입력·창·보정을 전부 코드 한 곳에 박고**, 같은 bag 과 줄자에서 **항상 같은 수**가
나오게 한다. 검토자가 이 명령 하나로 계수를 재현할 수 있어야 한다.

★ **무엇을 정본으로 쓰나 — 줄자는 "스케일", odom 은 "모양"**

계수에 필요한 것은 **정상구간 속도**인데, 줄자는 **총 이동거리** 하나뿐이라 그것만으로는
정상속도를 못 만든다(총거리/총시간은 가감속이 섞인 평균이다). 그래서 둘을 이렇게 나눈다:

    스케일  = 줄자 총거리 / odom 총 경로장        ← 🔴 줄자가 판정하는 것은 이것뿐이다
    정상속도 = odom 정상구간 twist × 스케일        ← 계수에 쓰는 값

odom 의 **시간 모양**은 믿고(엔코더가 언제 빨랐는지는 안다) **절대 배율만** 줄자로 고친다.
🔴 **줄자는 시작 표시부터 "최종 정지 위치"까지여야 한다** — 미리 그은 결승선까지 재면
관성 주행분이 빠져 odom 총 경로장과 **다른 구간**이 되고 스케일이 틀어진다.
⚠ 스케일이 `1 ± 5%` 를 벗어나면 오도메트리 스케일 자체를 의심한다(08-11 실측은 `0.987`).

  - **PWM 은 관측이 아니라 재구성**이다. `FF(명령) + Kp·(명령 − 정상속도)` 로 만든다.
    🔴 **적분항은 뺀다** — 근거는 시정수다(아래).

★ 🔴 **적분을 빼는 것이 정당한 이유, 그리고 그 한계**

적분 상태는 `error×dt` 로만 자라고 `INTEGRAL_PWM_LIMIT/WHEEL_KI` 로 잘린다. 출력 기여가
`X PWM` 이 되려면 `X/(WHEEL_KI·|error|)` 초가 걸린다 — 08-11 시행의 오차 크기에서 이는
**70초대**이고 실제 시행은 `5~10초`다. 그래서 그 창에서 적분 기여는 한 자리 PWM 미만이다.
**이 도구는 그 값을 계산해 같이 인쇄한다** — "무시했다"가 아니라 "이만큼이라 무시한다"를
검토자가 보게 한다. 🔴 **시행이 길어지면 이 전제가 깨진다.** `--max-window-s` 를 넘는
시행은 계수 산출에서 **거부**한다.

사용법:
    python3 tools/drive_ff_calibrate.py \\
        --point 0.05:<bag>:685 --point 0.12:<bag>:3000 --point 0.04:<bag>:<mm>
    (형식 = `명령속도:bag경로:줄자mm`)
    종료코드 0 = 계수 산출 / 1 = 산출 불능 / 2 = 입력 오류

정본 = `docs/MASTER_PLAN.md §7` 예약 32 · 짝 도구 = `tools/drive_ground_report.py`.
"""
import math
import sys

# ── 펌웨어 상수 (`.ino` 와 같아야 한다) ──────────────────────────────────
# 🔴 다르면 이 도구가 펌웨어와 다른 물건을 재게 된다. `.ino` 를 먼저 고치고 여기를 옮긴다.
LOW_SPEED_HOLD_PWM = 30.0
MIN_EFFECTIVE_WHEEL_CMD = 0.020
FEEDFORWARD_MAX_PWM = 145.0
WHEEL_KP = 30.0
# 🔴 2026-08-23 §91 P1-3 — **아래 상수 묶음은 08-22 16:20 펌웨어(`60bb3c2`)와 어긋난다.**
#   굽힌 값: WHEEL_KP 30→60 · INTEGRAL_PWM_LIMIT 20→40 · MAX_LINEAR_CMD 0.12→0.20.
#   여기 수치를 **일부러 안 옮겼다** — 이 파일은 *그때 그 보드로 낸 교정 결과를 재현*하는
#   용도라 값을 바꾸면 과거 산출을 못 되돌린다. 대신 규칙을 못박는다:
#   🔴 08-22 이후에 딴 bag 으로 이 도구를 돌리면 **산출이 무효다.** 새 보드로 재교정하려면
#      이 상수들을 `.ino` 현재값으로 갱신한 뒤 `--point` 를 새로 따야 한다.
#   정본 = `docs/REAL_ROBOT_VALUES.md §1-n` · `firmware/…v1_4.ino`.
WHEEL_KI = 5.0
INTEGRAL_PWM_LIMIT = 20.0
# 시행 당시 보드에 있던 값. 🔴 산출은 "그때 무엇이 실려 있었나"에 의존한다.
FF_IN_EFFECT = 1300.0

# 교정 목표와 합격 폭 (`§7-c-1`).
TARGET_MPS = 0.12
TOLERANCE = 0.10
# 🔴 적분 무시 전제가 성립하는 창의 상한. 넘으면 산출을 거부한다.
MAX_WINDOW_S = 20.0
# 계수를 확정하려면 이 개수 이상이 필요하다(§60.1 — 2점은 후보까지다).
MIN_POINTS = 3


class UsageError(Exception):
    """입력 계약 위반. rc=2 로 끝난다."""


def feedforward_pwm(cmd_mps, ff_slope=FF_IN_EFFECT):
    """`wheelVelocityToFeedforwardPwm()` 과 같은 식. 포화까지 그대로 재현한다."""
    excess = max(0.0, abs(cmd_mps) - MIN_EFFECTIVE_WHEEL_CMD)
    return min(LOW_SPEED_HOLD_PWM + excess * ff_slope, FEEDFORWARD_MAX_PWM)


def integral_seconds(pwm_gap, error_mps):
    """적분이 `pwm_gap` 만큼의 출력 기여를 만드는 데 걸리는 시간(초)."""
    if abs(error_mps) < 1e-12:
        return float('inf')
    return abs(pwm_gap) / (WHEEL_KI * abs(error_mps))


def scale_from_tape(tape_mm, odom_path_mm):
    """줄자/odom 배율. 🔴 둘 다 **총 이동**(시작 표시 → 최종 정지)이어야 한다."""
    if odom_path_mm <= 0:
        raise UsageError('odom 경로장이 0 이하다')
    return tape_mm / odom_path_mm


def reconstruct_point(cmd_mps, v_true, window_s, scale=None):
    """한 시행에서 `(재구성 PWM, 정상속도)` 한 점을 만든다.

    🔴 `v_true` 는 **줄자로 배율을 고친 정상구간 속도**다. 총거리/총시간 평균이 아니다.
    """
    ff = feedforward_pwm(cmd_mps)
    error = cmd_mps - v_true
    p_term = WHEEL_KP * error
    # 적분이 이 창에서 만들 수 있는 최대 기여(클램프와 시간 중 작은 쪽).
    i_cap = min(INTEGRAL_PWM_LIMIT, WHEEL_KI * abs(error) * window_s)
    return {
        'cmd': cmd_mps, 'v': v_true, 'window_s': window_s,
        'ff_pwm': ff, 'p_pwm': p_term, 'pwm': ff + p_term,
        'saturated': ff >= FEEDFORWARD_MAX_PWM - 1e-9,
        'i_possible_pwm': i_cap, 'scale': scale,
    }


def fit(points):
    """최소제곱 직선 `v = slope*(pwm - intercept)`. 반환은 판정 딕셔너리."""
    if len(points) < MIN_POINTS:
        return {'ok': False,
                'reason': f'점이 {len(points)}개다 — 계수 확정에는 {MIN_POINTS}개 이상이 '
                          f'필요하다(§60.1: 2점은 후보까지다)'}
    long_runs = [p for p in points if p['window_s'] > MAX_WINDOW_S]
    if long_runs:
        return {'ok': False,
                'reason': f'{long_runs[0]["window_s"]:.1f}s 시행이 있다 — 창이 '
                          f'{MAX_WINDOW_S:.0f}s 를 넘으면 적분 무시 전제가 깨진다'}

    n = len(points)
    xs = [p['pwm'] for p in points]
    ys = [p['v'] for p in points]
    mx, my = sum(xs) / n, sum(ys) / n
    denom = sum((x - mx) ** 2 for x in xs)
    if denom < 1e-9:
        return {'ok': False, 'reason': 'PWM 이 전부 같아 기울기를 못 낸다'}
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / denom
    if slope <= 0:
        return {'ok': False, 'reason': f'기울기가 양수가 아니다({slope:.6f}) — 입력을 본다'}
    intercept = mx - my / slope

    pwm_target = intercept + TARGET_MPS / slope
    ff_slope = (pwm_target - LOW_SPEED_HOLD_PWM) / (TARGET_MPS - MIN_EFFECTIVE_WHEEL_CMD)

    # 잔차 — 직선이 실제로 맞는지. 크면 계수를 못 믿는다.
    resid = [y - slope * (x - intercept) for x, y in zip(xs, ys)]
    rms = math.sqrt(sum(r * r for r in resid) / n)

    # 목표를 사이에 두는 점이 있어야 외삽이 아니다.
    below = [p for p in points if p['v'] <= TARGET_MPS]
    above = [p for p in points if p['v'] >= TARGET_MPS]

    return {
        'ok': True, 'n': n, 'slope': slope, 'intercept': intercept,
        'pwm_target': pwm_target, 'ff_slope': ff_slope,
        'resid_rms': rms, 'resid': resid,
        'brackets_target': bool(below and above),
        'nearest_gap': min(abs(p['v'] - TARGET_MPS) for p in points),
    }


def predict(ff_slope, v):
    """제안 계수로 굽었을 때 FF 만으로 나오는 속도(적분 없이)."""
    ff = feedforward_pwm(TARGET_MPS, ff_slope)
    return ff, v['slope'] * (ff - v['intercept'])


def report(points, v):
    print('=' * 78)
    print('피드포워드 계수 산출')
    print(f'  시행 당시 보드 FF 기울기 = {FF_IN_EFFECT:g} · 목표 {TARGET_MPS} m/s '
          f'±{TOLERANCE * 100:.0f}% = {TARGET_MPS * (1 - TOLERANCE):.4f}'
          f'~{TARGET_MPS * (1 + TOLERANCE):.4f}')
    print()
    print('  명령   정상속도   스케일  창(s)   FF항   Kp항  재구성PWM 포화 적분가능폭')
    for p in points:
        sc = '  —  ' if p['scale'] is None else f'{p["scale"]:.3f}'
        print(f'  {p["cmd"]:.3f} {p["v"]:9.4f}  {sc} {p["window_s"]:6.2f} '
              f'{p["ff_pwm"]:6.1f} {p["p_pwm"]:6.1f} {p["pwm"]:9.1f}  '
              f'{"Y" if p["saturated"] else "-"}  {p["i_possible_pwm"]:6.2f}')
    print()
    print('  🔴 PWM 은 관측이 아니라 재구성이다 — 보드가 appliedPwm 을 발행하지 않는다.')
    print('     적분은 뺐고, 그 창에서 적분이 만들 수 있는 최대 기여를 위에 같이 적었다.')
    print()
    if not v['ok']:
        print(f'  🔴 산출 불능 — {v["reason"]}')
        return
    print(f'  plant  : v = {v["slope"]:.6f} × (PWM − {v["intercept"]:.2f})   '
          f'(점 {v["n"]}개 · 잔차 RMS {v["resid_rms"]:.5f} m/s)')
    print(f'  목표 PWM: {v["pwm_target"]:.1f}')
    print(f'  >>> FEEDFORWARD_PWM_PER_MPS_ABOVE_MIN = {v["ff_slope"]:.0f}')
    ff, v_pred = predict(v['ff_slope'], v)
    print(f'      그 값으로 굽으면 FF 단독 출력 {ff:.1f} PWM → {v_pred:.4f} m/s')
    print()
    if not v['brackets_target']:
        print(f'  ⚠ 목표를 사이에 두는 점이 없다 — 외삽이다. 가장 가까운 점이 '
              f'{v["nearest_gap"]:.4f} m/s 떨어져 있다.')
    else:
        print('  ✅ 목표를 사이에 두는 점이 있다 — 내삽이다.')
    print('  🔴 확정은 굽고 지면에서 실측하는 것이다. 이 값은 굽을 후보다.')


def parse_point(spec):
    """`명령:bag:줄자mm` 한 개."""
    parts = spec.split(':')
    if len(parts) != 3:
        raise UsageError(f'--point 는 "명령:bag:줄자mm" 세 값이다: {spec!r}')
    try:
        cmd, tape = float(parts[0]), float(parts[2])
    except ValueError:
        raise UsageError(f'--point 의 명령·줄자가 수가 아니다: {spec!r}') from None
    if cmd <= 0 or tape <= 0:
        raise UsageError(f'--point 의 명령·줄자는 양수여야 한다: {spec!r}')
    return cmd, parts[1], tape


def main(argv):
    specs = [argv[i + 1] for i, a in enumerate(argv) if a == '--point' and i + 1 < len(argv)]
    if not specs:
        print(__doc__.split('사용법:')[1].strip(), file=sys.stderr)
        return 2
    try:
        parsed = [parse_point(s) for s in specs]
    except UsageError as exc:
        print(f'입력 오류 — {exc}', file=sys.stderr)
        return 2

    sys.path.insert(0, __file__.rsplit('/', 1)[0])
    import drive_ground_report as gr                       # noqa: PLC0415

    points = []
    for cmd, bag, tape in parsed:
        try:
            cmds, odoms, imu = gr.load(bag)
        except Exception as exc:                           # noqa: BLE001
            print(f'판정 불가 — bag 을 못 읽었다 ({bag}): {exc}', file=sys.stderr)
            return 2
        a = gr.analyze(cmds, odoms, imu, tape)
        if not a['ok']:
            print(f'판정 불가 — {bag}: {a["reason"]}', file=sys.stderr)
            return 1
        if a.get('cruise_mps') is None:
            print(f'판정 불가 — {bag}: 정상구간 표본이 없다(시행이 너무 짧다)', file=sys.stderr)
            return 1
        sc = scale_from_tape(tape, a['path_mm'])
        if abs(sc - 1.0) > 0.05:
            print(f'  ⚠ {bag}: 줄자/odom 배율 {sc:.3f} — 5% 를 벗어났다. '
                  f'오도메트리 스케일을 먼저 본다.')
        points.append(reconstruct_point(cmd, a['cruise_mps'] * sc, a['obs_dur_s'], sc))

    points.sort(key=lambda p: p['pwm'])
    v = fit(points)
    report(points, v)
    return 0 if v['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
