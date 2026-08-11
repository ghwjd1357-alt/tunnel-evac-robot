#!/usr/bin/env python3
"""R0 watchdog(`TODO(D+0) #11`) 1차 증거 — 영상에서 바퀴 정지 프레임을 기계로 잰다.

정본 = `docs/JETSON_SETUP.md §7-c-0`. 짝 도구 = `tools/watchdog_report.py`(bag).

★ **이 도구가 재는 것과 안 재는 것을 먼저 갈라 둔다.**

  - **잰다** = 바퀴가 마지막으로 회전한 프레임. 카메라 흔들림을 보정한 뒤의 회전만 센다.
  - **안 잰다** = `T0`(마지막 `publishing #N` 이 화면에 뜬 프레임). 터미널 글자를 읽는
    일이라 사람이 눈으로 확인해 `--t0-frame` 으로 준다.

🔴 **그래서 이 도구는 원리적으로 FAIL 만 증명할 수 있고 PASS 는 증명하지 못한다.**
`T0` 는 *글자가 화면에 뜬 시각*이지 *메시지가 나간 시각*이 아니다. 터미널 렌더는 항상
늦으므로 `측정값 = 참값 - 렌더지연` 이고 **측정값은 참값의 하한**이다. 따라서

    측정값 > 판정선  →  FAIL 확정 (참값은 더 크다)
    측정값 ≤ 판정선  →  🔴 **판정 불능** (렌더지연을 독립적으로 못 묶으면 PASS 라고 못 한다)

2026-08-11 에 이 자리에서 거짓 PASS 가 날 뻔했다: 영상 실측 `500.0ms` 가 구 판정선
`500.0ms` 아래로 보였지만, 같은 시행의 bag 은 `516.2ms` 였다. 차이 `16.2ms`(1 프레임)가
정확히 렌더 지연이다. **총 정지 시간의 정본 측정은 bag 이고**(시작점이 실제 메시지
타임스탬프라 편향이 없다), 영상은 *펌웨어와 독립된* 관측이라는 다른 가치를 갖는다.
`--bag-ms` 를 주면 둘의 차이를 렌더 지연으로 환산해 타당 범위인지 같이 검사한다.

★ **채택한 측정 방법과, 먼저 실패한 두 방법** (2026-08-11 실측. 다시 밟지 않도록 남긴다)

  ① **폴라 상호상관** — 바퀴 중심을 배경 phase correlation 으로 추적하고 극좌표를 펴서
     상관을 봤다. **중심이 2px 틀리면 반경 150px 에서 0.76° 의 가짜 회전**이 생긴다.
     주행 신호가 프레임당 1.5° 라 잡음이 신호와 같은 자릿수다. 주행 구간 실측이
     `0.482°/f ± 0.331` 로 진동해서 폐기했다.
  ② **아핀 고유값** — 프레임 간 아핀의 2×2 부분은 `M·R(θ)·M⁻¹` 이라 복소 고유값의 편각이
     곧 회전각이고 중심·타원비와 무관하다. 원리는 옳은데 **회전이 작아지면 고유값이
     실수로 떨어져 `NaN`** 이 된다 — 정지 근처, 즉 판정이 필요한 바로 그 자리에서 못 쓴다.
  ③ **채택** — 배경 특징점으로 아핀을 구해 바퀴 위 점들에 적용하고 그 **잔차**를 본다.
     잔차를 바퀴 평면의 **접선 성분(=회전)** 과 **반경 성분(=추적 잡음)** 으로 분해한다.
     중심 오차가 만드는 가짜 성분은 원 둘레를 돌며 부호가 뒤집혀 **중앙값에서 상쇄**된다.
     실측 대비: 주행 접선 `1.490°/f` vs 정지 후 `0.0009°/f`, 접선/반경 비가 주행 `1.6` →
     정지 `0.01`. **남은 흔들림이 회전이 아님을 이 비가 증명한다.**

사용법:
    python3 tools/watchdog_video.py <영상> --t0-frame 670 --preset 0807-1522
    python3 tools/watchdog_video.py <영상> --t0-frame 670 \\
        --center 480,446 --axes 112,178 --range 610,895 --bag-ms 516.2
    종료코드 0 = 판정 유효 / 1 = 판정 불능·FAIL / 2 = 사용법·입력 오류
"""
import argparse
import math
import os
import sys

# ── 정본 상수 ────────────────────────────────────────────────────────────
# 펌웨어 `.ino:128` 의 롤링 반경. 회전각을 bag 과 같은 mm/s 로 환산할 때 쓴다.
WHEEL_RADIUS_M = 0.05698
# 🔴 `watchdog_report.py` 와 **같은 판정선**을 쓴다. 두 도구가 다른 선을 쓰면 서로를
# 교차 검증할 수 없다. 관측 주행속도 약 0.1 m/s 의 5%.
MOTION_RATE_MM_S = 5.0
# 그 속도를 재는 창. 🔴 짝 도구와 **같은 200ms** 다 — 창이 다르면 두 수치가 같은 뜻이 아니다.
MOTION_WINDOW_MS = 200
# 임계 하나로 수치가 흔들리는 것을 숨기지 않는다(짝 도구 §52.2 와 같은 규율).
SENSITIVITY_RATES_MM_S = (2.0, 5.0, 10.0, 20.0)
# §7-c-0 조건 2 — 마지막 회전 뒤 이만큼은 관찰해야 "섰다"고 말할 수 있다.
REQUIRED_TAIL_MS = 2000
# 잡음 바닥을 재는 구간. **정지 지점을 보고 고르지 않는다**(순환 논증) — 항상 열의 끝에서
# 잘라 쓴다. 그 구간이 정지 뒤인지는 `REQUIRED_TAIL_MS` 검사가 따로 보증한다.
NOISE_TAIL_FRAMES = 90
# 구 판정선(§7-c-0 조건 1 원본) = fps × 0.5. 상수가 아니라 fps 에서 나온다.
LEGACY_TOTAL_RATIO = 0.5
# 🔴 결정 1-ⓐ(2026-08-11 사용자 결정) 초안 = 총 정지 상한. **검토자 확인 대기**다.
PROPOSED_TOTAL_MS = 600
# 렌더 지연이 이 범위 밖이면 T0 를 잘못 읽었거나 두 측정이 다른 시행이다.
PLAUSIBLE_RENDER_LAG_MS = (0.0, 60.0)


def deg_per_frame_to_mm_s(deg, fps):
    """회전각(°/프레임) → 바퀴 표면 속도(mm/s). bag 판정선과 같은 단위로 만든다."""
    return math.radians(abs(deg)) * fps * WHEEL_RADIUS_M * 1000.0


def mm_s_to_deg_per_frame(mm_s, fps):
    return math.degrees(mm_s / 1000.0 / WHEEL_RADIUS_M / fps)


def rotation_series(path, center, axes, frame_range, progress=None):
    """영상에서 프레임당 (접선=회전, 반경=잡음) 을 뽑는다. cv2 의존은 여기에만 둔다.

    반환 = `[(n, rot_deg, rad_deg, n_points), ...]`. 추적 실패 프레임은 `nan`.

    ⚠ `center` 는 **`frame_range[0]` 시점의** 바퀴 타원 중심이다. 카메라가 흐르므로
    (실측 285 프레임에 198px) 중심을 고정하면 마스크가 바퀴를 벗어난다. 매 프레임쌍에서
    구한 **배경 아핀을 중심에 그대로 적용**해 따라가게 한다 — 로봇은 세계에 대해 서 있고
    움직이는 것은 카메라뿐이므로, 배경의 운동이 곧 바퀴 중심의 화면상 운동이다.
    """
    import cv2                      # noqa: PLC0415  — 회귀는 이 함수를 안 부른다
    import numpy as np              # noqa: PLC0415

    ax, by = axes
    lo, hi = frame_range
    pad = 2.6                       # 바퀴 주변만 본다. 먼 배경은 시차를 만든다.
    lk = dict(winSize=(31, 31), maxLevel=4,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 50, 5e-4))

    def flow(a, b, pts):
        """전방·후방 추적이 일치하는 점만 남긴다(forward-backward check)."""
        p1, st, _ = cv2.calcOpticalFlowPyrLK(a, b, pts, None, **lk)
        pb, st2, _ = cv2.calcOpticalFlowPyrLK(b, a, p1, None, **lk)
        ok = ((st.ravel() == 1) & (st2.ravel() == 1)
              & (np.linalg.norm(pb - pts, axis=2).ravel() < 0.3))
        return pts[ok].reshape(-1, 2), p1[ok].reshape(-1, 2)

    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise RuntimeError(f'영상을 열지 못했다: {path}')
    # `CAP_PROP_POS_FRAMES` 는 HEVC 에서 부정확할 수 있다 — grab() 으로 정확히 센다.
    for _ in range(lo):
        if not cap.grab():
            raise RuntimeError(f'프레임 {lo} 에 못 닿았다')
    ok, frame = cap.read()
    if not ok:
        raise RuntimeError(f'프레임 {lo} 을 읽지 못했다')
    prev = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = prev.shape

    cx, cy = float(center[0]), float(center[1])
    out = []
    for n in range(lo, hi):
        ok, frame = cap.read()
        if not ok:
            break
        cur = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if progress and (n - lo) % 25 == 0:
            progress(n)

        x0, y0 = max(int(cx - ax * pad), 0), max(int(cy - by * pad), 0)
        x1, y1 = min(int(cx + ax * pad), w), min(int(cy + by * pad), h)
        a, b = prev[y0:y1, x0:x1], cur[y0:y1, x0:x1]
        prev = cur
        yy, xx = np.mgrid[y0:y1, x0:x1]
        ell = ((xx - cx) / (ax * 1.28)) ** 2 + ((yy - cy) / (by * 1.28)) ** 2
        m_wheel = (ell < 1.0).astype('uint8') * 255
        m_back = (ell > 2.8).astype('uint8') * 255

        q0 = cv2.goodFeaturesToTrack(a, 900, 0.006, 6, mask=m_back, blockSize=7)
        p0 = cv2.goodFeaturesToTrack(a, 900, 0.006, 6, mask=m_wheel, blockSize=7)
        if q0 is None or p0 is None:
            out.append((n, float('nan'), float('nan'), 0))
            continue
        Q0, Q1 = flow(a, b, q0)
        P0, P1 = flow(a, b, p0)
        if len(Q0) < 25 or len(P0) < 40:
            out.append((n, float('nan'), float('nan'), len(P0)))
            continue
        Mb, _ = cv2.estimateAffine2D(Q0, Q1, method=cv2.RANSAC,
                                     ransacReprojThreshold=0.5, maxIters=8000,
                                     confidence=0.999)
        if Mb is None:
            out.append((n, float('nan'), float('nan'), len(P0)))
            continue

        # 바퀴 위 점에서 배경(=카메라) 운동을 뺀 나머지가 바퀴 자신의 움직임이다.
        res = P1 - ((Mb[:, :2] @ P0.T).T + Mb[:, 2])
        # 타원을 단위원으로 펴면 화면 각도가 곧 바퀴 각도다(약 정사영 가정).
        u = (P0[:, 0] + x0 - cx) / ax
        v = (P0[:, 1] + y0 - cy) / by
        r = np.hypot(u, v)
        ph = np.arctan2(v, u)
        keep = r > 0.35                      # 중심 근처는 반경이 작아 각도 잡음이 폭발한다
        if keep.sum() < 30:
            out.append((n, float('nan'), float('nan'), int(keep.sum())))
        else:
            du, dv = res[keep, 0] / ax, res[keep, 1] / by
            rk, pk = r[keep], ph[keep]
            dphi = (-du * np.sin(pk) + dv * np.cos(pk)) / rk    # 접선 = 회전
            drad = (du * np.cos(pk) + dv * np.sin(pk)) / rk     # 반경 = 추적 잡음
            out.append((n, float(np.degrees(np.median(dphi))),
                        float(np.degrees(np.median(np.abs(drad)))), int(keep.sum())))

        # 다음 쌍을 위해 중심을 배경 운동만큼 옮긴다(크롭은 순수 평행이동이라 이렇게 된다).
        p = np.array([cx - x0, cy - y0])
        cx, cy = (Mb[:, :2] @ p + Mb[:, 2]) + np.array([x0, y0])
    cap.release()
    return out


def analyze(series, t0_frame, fps, bag_ms=None):
    """순수 함수 — 영상 I/O 없이 판정한다. 회귀는 여기에 합성 열을 넣는다.

    `series` = `[(n, rot_deg, rad_deg, n_points), ...]`.
    """
    rows = [r for r in series if r[0] is not None]
    if len(rows) < NOISE_TAIL_FRAMES + 10:
        return {'ok': False, 'reason': f'프레임이 {len(rows)}개뿐이라 잡음 바닥을 못 잡는다'}
    if fps <= 0:
        return {'ok': False, 'reason': 'fps 가 0 이하다'}

    frames = [r[0] for r in rows]
    if not frames[0] <= t0_frame <= frames[-1]:
        return {'ok': False, 'reason': f'T0={t0_frame} 이 분석 구간 밖이다'}

    window_frames = max(int(round(MOTION_WINDOW_MS / 1000.0 * fps)), 2)
    rot = [r[1] if math.isfinite(r[1]) else 0.0 for r in rows]

    def swept(i):
        """프레임 `i` 이후 창 안에서 누적 회전이 가장 크게 벌어진 값(부호 무시)."""
        acc, far = 0.0, 0.0
        for k in range(i, min(i + window_frames, len(rot))):
            acc += rot[k]
            far = max(far, abs(acc))
        return far

    # 🔴 **프레임당 증분으로 판정하지 않는다** — 짝 도구가 검토 §52.2 에서 배운 자리와 같다.
    # 실측: 프레임당 잡음 최대가 0.146°/f 로 정본 판정선(5mm/s = 0.084°/f)보다 커서
    # 증분 판정은 정지 뒤 잡음을 계속 "이동"으로 읽는다. 창 안 누적으로 보면 무작위
    # 잡음은 상쇄되고(창 12프레임에 약 0.2°) 실회전만 남는다(주행 17.9°).
    tail_start = len(rows) - NOISE_TAIL_FRAMES
    noise_swept = [swept(i) for i in range(tail_start, len(rows) - window_frames)]
    if len(noise_swept) < NOISE_TAIL_FRAMES // 2:
        return {'ok': False, 'reason': '잡음 구간이 창 하나를 채우지 못한다'}
    noise_mean = sum(noise_swept) / len(noise_swept)
    noise_max = max(noise_swept)

    def last_motion(rate_mm_s):
        """임계를 넘은 마지막 프레임의 **다음** 프레임 = 바퀴가 최종 위치에 선 프레임."""
        limit = mm_s_to_deg_per_frame(rate_mm_s, fps) * window_frames
        hit = [rows[i][0] for i in range(len(rows))
               if rows[i][0] >= t0_frame and swept(i) > limit]
        return None if not hit else hit[-1] + 1

    canon_deg = mm_s_to_deg_per_frame(MOTION_RATE_MM_S, fps) * window_frames
    if canon_deg <= noise_max:
        return {'ok': False,
                'reason': f'정본 판정선({MOTION_RATE_MM_S:g}mm/s = 창 {window_frames}프레임에 '
                          f'{canon_deg:.3f}°)이 잡음 최대({noise_max:.3f}°) 아래다 — '
                          f'영상이 이 시험을 못 가른다'}

    stop = last_motion(MOTION_RATE_MM_S)
    if stop is None:
        return {'ok': False, 'reason': 'T0 이후 회전이 관측되지 않았다 — T0 를 잘못 읽었을 수 있다'}

    tail_frames = frames[-1] - stop
    tail_ms = tail_frames / fps * 1000.0
    if tail_ms < REQUIRED_TAIL_MS:
        return {'ok': False,
                'reason': f'마지막 회전 뒤 관찰이 {tail_ms:.0f} ms 뿐이다 '
                          f'(§7-c-0 조건 2 = {REQUIRED_TAIL_MS}ms)'}

    # 조건 2 — 정지 뒤 누적 회전이 판정선 아래인가. 부호를 살려 더해야 creep 를 잡는다.
    after = [r[1] for r in rows if r[0] >= stop and math.isfinite(r[1])]
    drift_deg = sum(after)
    drift_mm_s = deg_per_frame_to_mm_s(drift_deg / max(len(after), 1), fps)

    sens = {}
    for rate in SENSITIVITY_RATES_MM_S:
        k = last_motion(rate)
        sens[rate] = None if k is None else k - t0_frame

    n_frames = stop - t0_frame
    measured_ms = n_frames / fps * 1000.0
    legacy_limit = fps * LEGACY_TOTAL_RATIO

    def verdict(limit_frames):
        """🔴 영상은 하한만 준다 — 넘으면 FAIL 확정, 밑이면 판정 불능이다."""
        return 'FAIL' if n_frames > limit_frames else '판정 불능'

    lag_ms = None if bag_ms is None else bag_ms - measured_ms
    lag_ok = None if lag_ms is None else (
        PLAUSIBLE_RENDER_LAG_MS[0] <= lag_ms <= PLAUSIBLE_RENDER_LAG_MS[1])

    return {
        'ok': True,
        't0_frame': t0_frame,
        'stop_frame': stop,
        'n_frames': n_frames,
        'measured_ms': measured_ms,
        'fps': fps,
        'noise_mean_deg': noise_mean,
        'noise_max_deg': noise_max,
        'canon_line_deg': canon_deg,
        'window_frames': window_frames,
        'sensitivity': sens,
        'tail_ms': tail_ms,
        'drift_deg': drift_deg,
        'drift_mm_s': drift_mm_s,
        'cond2_ok': drift_mm_s < MOTION_RATE_MM_S,
        'legacy_limit_frames': legacy_limit,
        'legacy_verdict': verdict(legacy_limit),
        'proposed_limit_frames': PROPOSED_TOTAL_MS / 1000.0 * fps,
        'proposed_verdict': verdict(PROPOSED_TOTAL_MS / 1000.0 * fps),
        'bag_ms': bag_ms,
        'render_lag_ms': lag_ms,
        'render_lag_plausible': lag_ok,
    }


def report(v):
    """사람이 읽을 출력. 🔴 편향 경고를 수치와 **같은 화면**에 낸다."""
    if not v['ok']:
        print(f"  🔴 판정 불가 — {v['reason']}")
        return
    print(f"  fps {v['fps']:.4f} · 1 프레임 = {1000.0 / v['fps']:.3f} ms · "
          f"판정 창 {v['window_frames']}프레임({MOTION_WINDOW_MS}ms)")
    print(f"  잡음 바닥(창 누적): 평균 {v['noise_mean_deg']:.3f}° · 최대 {v['noise_max_deg']:.3f}°")
    print(f"  정본 판정선 {MOTION_RATE_MM_S:g} mm/s = 창당 {v['canon_line_deg']:.3f}°")
    print()
    print(f"  T0(발행 표시) n={v['t0_frame']}  →  마지막 회전 n={v['stop_frame']}")
    print(f"  >>> {v['n_frames']} 프레임 = {v['measured_ms']:.1f} ms  "
          f"🔴 **이 값은 하한이다**(렌더 지연만큼 짧게 나온다)")
    band = ' · '.join(f"{r:g}mm/s→{'?' if f is None else f'{f}f'}"
                      for r, f in sorted(v['sensitivity'].items()))
    print(f'  >>> 판정선 민감도: {band}')
    print()
    print(f"  구 기준(총 정지 ≤ fps×0.5 = {v['legacy_limit_frames']:.3f}f): "
          f"**{v['legacy_verdict']}**")
    print(f"  ⓐ 초안(총 정지 ≤ {PROPOSED_TOTAL_MS}ms = {v['proposed_limit_frames']:.1f}f): "
          f"**{v['proposed_verdict']}**  ⚠ 검토자 확인 대기")
    print(f"  [조건 2] 정지 후 {v['tail_ms']:.0f}ms 누적 회전 {v['drift_deg']:+.3f}° "
          f"= {v['drift_mm_s']:.4f} mm/s → "
          f"{'충족' if v['cond2_ok'] else '🔴 미충족'} (판정선 {MOTION_RATE_MM_S:g} mm/s)")
    if v['bag_ms'] is not None:
        mark = '타당' if v['render_lag_plausible'] else '🔴 범위 밖 — T0 오독이나 다른 시행'
        print(f"  [교차] bag {v['bag_ms']:.1f}ms - 영상 {v['measured_ms']:.1f}ms "
              f"= 렌더 지연 {v['render_lag_ms']:+.1f}ms ({mark})")


PRESETS = {
    # 2026-08-07 15:22:58 촬영(`IMG_3461.mov`) · bag `d0_watchdog_0807_1522` 와 같은 시행.
    # 중심은 **분석 시작 프레임(610)** 시점의 크롬 림 타원 중심, 4K 원본 좌표계. 2026-08-11 실측.
    # T0=670 · 축은 림 외곽(반축 112 × 178, 수직축이 긴 것은 바퀴가 비스듬히 보이기 때문).
    '0807-1522': dict(center=(439.3, 531.1), axes=(112.0, 178.0), frame_range=(610, 895)),
}


def main(argv, series_fn=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('video')
    ap.add_argument('--t0-frame', type=int, required=True,
                    help='마지막 publishing #N 이 화면에 뜬 프레임(사람이 눈으로 확인)')
    ap.add_argument('--preset', choices=sorted(PRESETS))
    ap.add_argument('--center', help='바퀴 타원 중심 "x,y" (원본 좌표계)')
    ap.add_argument('--axes', help='바퀴 타원 반축 "a,b"')
    ap.add_argument('--range', help='분석 프레임 "lo,hi"')
    ap.add_argument('--fps', type=float, help='생략하면 영상에서 읽는다')
    ap.add_argument('--bag-ms', type=float, help='같은 시행의 bag 값 — 렌더 지연 교차검사')
    a = ap.parse_args(argv[1:])

    cfg = dict(PRESETS.get(a.preset, {}))
    for key, raw in (('center', a.center), ('axes', a.axes), ('frame_range', a.range)):
        if raw:
            cfg[key] = tuple(float(v) for v in raw.split(','))
    missing = [k for k in ('center', 'axes', 'frame_range') if k not in cfg]
    if missing:
        print(f"--preset 이나 {'/'.join('--' + m for m in missing)} 가 필요하다", file=sys.stderr)
        return 2
    cfg['frame_range'] = tuple(int(v) for v in cfg['frame_range'])

    fps = a.fps
    if fps is None:
        try:
            import cv2                                       # noqa: PLC0415
            cap = cv2.VideoCapture(a.video)
            fps = cap.get(cv2.CAP_PROP_FPS)
            cap.release()
        except Exception as exc:                             # noqa: BLE001
            print(f'fps 를 읽지 못했다 — --fps 로 준다: {exc}', file=sys.stderr)
            return 2
    if not fps or fps <= 0:
        print('fps 를 읽지 못했다 — --fps 로 준다', file=sys.stderr)
        return 2

    print('=' * 78)
    print('VIDEO:', os.path.basename(a.video))
    try:
        series = (series_fn or rotation_series)(
            a.video, cfg['center'], cfg['axes'], cfg['frame_range'],
            progress=lambda n: print(f'  … {n}', end='\r', flush=True))
    except Exception as exc:                                 # noqa: BLE001
        print(f'  판정 불가 — 영상을 읽지 못했다: {exc}')
        return 1
    print(' ' * 30, end='\r')

    v = analyze(series, a.t0_frame, fps, bag_ms=a.bag_ms)
    report(v)
    print()
    if not v['ok']:
        return 1
    # 🔴 판정 불능은 성공이 아니다 — 자동화가 PASS 로 오인하지 않게 nonzero 로 끝낸다.
    if v['legacy_verdict'] != 'PASS' and v['proposed_verdict'] != 'PASS':
        print('판정 유효 · 🔴 영상 단독으로는 PASS 를 못 만든다 (하한만 준다)')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
