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

하한 성질은 **교차검사 없이 혼자 성립한다** — 시작점은 렌더만큼 늦고, 끝점은 판정선
`{motion_rate:g}mm/s` 를 밑도는 순간 검출돼 참값보다 이르다. 양쪽 모두 구간을 짧게 만든다.

2026-08-11 에 이 자리에서 거짓 PASS 가 날 뻔했다: 영상 실측 `{measured_ms}ms`
({n_frames}프레임)가 구 판정선 `500ms` 아래로 보였지만, 같은 시행의 bag 은 `{bag_ms}ms` 였다.

🔴 **그 차이 `{delta_ms}ms` 를 한 가지 원인으로 부르지 않는다**(검토 §57.2 P1). bag 의
시작점은 `Twist` 가 *발행된* 시각이 아니라 rosbag 이 *저장한* 시각이고(`watchdog_report.py`),
끝점도 `/odom` 이 저장된 시각이다. 영상은 *화면에 뜬* 프레임에서 *회전이 검출된* 프레임까지다.
둘은 **서로 다른 관측계**라 차이 안에는 렌더 지연 · 두 토픽의 DDS/record 지연 · odom 발행
주기 · 두 T1 검출기의 차이가 **함께** 들어 있다. 그래서 `--bag-ms` 는 차이를 **적어 둘 뿐,
타당/부당을 판정하지 않는다.** 총 정지의 정본 측정은 bag 이다(사람 눈과 화면 렌더가 끼지
않는다) — 그러나 그 시각도 *저장* 시각이라 무편향은 아니다.

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

★ **못 본 프레임은 "안 돌았다" 가 아니라 "모른다" 다** (검토 §57.1 P1)

구판은 추적 실패(`NaN`)를 `0.0` 회전으로 바꿔 넣었다. 렌즈가 가려지거나 디코드가 끊긴
구간이 전부 **"정지 증거"** 로 둔갑해, 꼬리 전체를 못 봐도 `조건 2 충족` 이 나왔다.
이 도구의 조건 2 는 R0→R1 지면 주행 허가의 입력이므로 그 방향의 실패는 위험하다.
→ 이제 **관측이 완전할 때만 수치를 낸다**: ⓵ 프레임 번호가 처음부터 끝까지 1씩 연속이고
(결측·중복·역순 없음) ⓶ 요청 구간이 조기 EOF 로 잘리지 않았고 ⓷ `T0` 이후 모든 프레임의
회전값이 유한하며 유효점이 `{min_points}`개 이상일 때. 하나라도 어긋나면 **판정 불능**이다.

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
# 관측이 유효하다고 인정할 최소 유효점 수. 생산자의 `keep.sum() < 30` 과 **같은 선**을
# 소비자도 독립으로 본다 — 생산자가 바뀌어도 소비 쪽 계약이 남는다(검토 §57.1).
MIN_VALID_POINTS = 30
# 🔴 결정 1-ⓐ(2026-08-11 사용자 결정) 초안 = 총 정지 상한. **검토자 확인 대기**다.
# 🔴 이 값은 **최악 지연의 증명이 아니라 사용자가 수용한 운용 상한**이다(검토 §57.2).
PROPOSED_TOTAL_MS = 600

# ── 기록된 실측 (2026-08-11 · `PRESETS['0807-1522']`) ────────────────────────
# 🔴 **설명·출력·회귀가 전부 이 한 곳에서 나온다.** 여기 숫자 하나를 바꾸면 docstring 과
# 회귀가 **같이** 깨진다 — 도구 설명만 옛 수치로 남는 사고(검토 §57.3 P2)를 구조로 막는다.
RECORDED = dict(
    fps=59.9955, t0_frame=670, stop_frame=698, n_frames=28,
    measured_ms=466.7,      # = n_frames / fps
    bag_ms=516.2,           # 같은 시행 `d0_watchdog_0807_1522`
    delta_ms=49.5,          # 🔴 관측계 차이. 렌더 지연 '확정' 이 아니다.
    drift_mm_s=0.5945,      # 정지 뒤 누적 — 조건 2(영상 전용 관측)
)
if __doc__:                      # `python -OO` 로 돌리면 docstring 이 없다
    __doc__ = __doc__.format(motion_rate=MOTION_RATE_MM_S,
                             min_points=MIN_VALID_POINTS, **RECORDED)


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


def analyze(series, t0_frame, fps, bag_ms=None, expected_range=None):
    """순수 함수 — 영상 I/O 없이 판정한다. 회귀는 여기에 합성 열을 넣는다.

    `series` = `[(n, rot_deg, rad_deg, n_points), ...]`.
    `expected_range` = 생산자에게 **요청한** `(lo, hi)`. 주면 조기 EOF 를 잡는다.
    """
    rows = [r for r in series if r[0] is not None]
    if len(rows) < NOISE_TAIL_FRAMES + 10:
        return {'ok': False, 'reason': f'프레임이 {len(rows)}개뿐이라 잡음 바닥을 못 잡는다'}
    if not math.isfinite(fps) or fps <= 0:
        return {'ok': False, 'reason': f'fps 가 유한한 양수가 아니다: {fps!r}'}

    frames = [r[0] for r in rows]
    if not frames[0] <= t0_frame <= frames[-1]:
        return {'ok': False, 'reason': f'T0={t0_frame} 이 분석 구간 밖이다'}

    # ── 🔴 관측 완전성 (검토 §57.1) ──────────────────────────────────────
    # 여기를 통과해야 아래의 "몇 프레임"·"몇 초 관찰" 이 **실제로 본 것**이 된다.
    gaps = [(frames[i - 1], frames[i]) for i in range(1, len(frames))
            if frames[i] != frames[i - 1] + 1]
    if gaps:
        return {'ok': False,
                'reason': f'프레임이 연속이 아니다 — {gaps[0][0]}→{gaps[0][1]} '
                          f'(결측·중복·역순 {len(gaps)}곳). 안 본 구간을 관찰로 셀 수 없다'}
    if expected_range is not None:
        lo, hi = expected_range
        if frames[0] != lo or frames[-1] != hi - 1:
            return {'ok': False,
                    'reason': f'요청 구간 {lo}~{hi - 1} 중 {frames[0]}~{frames[-1]} 만 '
                              f'관측됐다 — 조기 EOF·디코드 중단'}

    t0_idx = frames.index(t0_frame)          # 연속성이 보장돼 항상 존재한다
    # 🔴 **`NaN` 을 `0.0`(=안 돌았다)으로 바꾸지 않는다.** 못 본 것은 "모른다" 이고,
    # 모르는 구간은 정지 증거가 될 수 없다. 판정 구간 전체를 fail-closed 로 막는다.
    bad = [rows[i][0] for i in range(t0_idx, len(rows))
           if not math.isfinite(rows[i][1]) or rows[i][3] < MIN_VALID_POINTS]
    if bad:
        return {'ok': False,
                'reason': f'T0 이후 관측 실패 {len(bad)}프레임(첫 n={bad[0]}, '
                          f'유효점 {MIN_VALID_POINTS} 미만이거나 회전값이 유한하지 않다) '
                          f'— 못 본 구간은 정지가 아니다'}

    window_frames = max(int(round(MOTION_WINDOW_MS / 1000.0 * fps)), 2)
    # T0 앞은 판정에 쓰이지 않는다. 값이 유효하지 않으면 `nan` 그대로 둔다 —
    # 0 으로 채우면 위에서 막은 바로 그 거짓 정지가 뒷문으로 돌아온다.
    rot = [r[1] if math.isfinite(r[1]) else float('nan') for r in rows]

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
    # 🔴 `t0_idx` 아래로는 내려가지 않는다 — 그 앞은 유효성을 보증하지 않은 구간이다.
    tail_start = max(len(rows) - NOISE_TAIL_FRAMES, t0_idx)
    noise_swept = [swept(i) for i in range(tail_start, len(rows) - window_frames)]
    if len(noise_swept) < NOISE_TAIL_FRAMES // 2:
        return {'ok': False, 'reason': '잡음 구간이 창 하나를 채우지 못한다'}
    noise_mean = sum(noise_swept) / len(noise_swept)
    noise_max = max(noise_swept)

    def last_motion(rate_mm_s):
        """임계를 넘은 마지막 프레임의 **다음** 프레임 = 바퀴가 최종 위치에 선 프레임."""
        limit = mm_s_to_deg_per_frame(rate_mm_s, fps) * window_frames
        # 🔴 `t0_idx` 부터만 훑는다 — 유효성을 보증한 구간이 정확히 여기다.
        hit = [rows[i][0] for i in range(t0_idx, len(rows)) if swept(i) > limit]
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
    # 🔴 여기서 비유한값을 **거르지 않는다**. 걸러 낼 것이 있었다면 위 fail-closed 에서
    # 이미 판정 불능으로 끝났어야 한다 — 거르는 순간 "못 본 꼬리"가 조건 2 를 통과한다.
    after = [r[1] for r in rows if r[0] >= stop]
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

    # 🔴 bag 과의 차이는 **서로 다른 관측계의 차이**일 뿐이다(검토 §57.2). 이름도 그렇게
    # 부르고, "이 범위면 타당" 같은 판정을 붙이지 않는다 — 성분을 못 가르기 때문이다.
    delta_ms = None if bag_ms is None else bag_ms - measured_ms

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
        'cross_observer_delta_ms': delta_ms,
        # 부호만 본다. 음수면 "영상이 bag 보다 길다" 는 뜻이라 같은 시행인지·T0 를 옳게
        # 읽었는지 되묻게 한다. 🔴 양수라고 해서 무엇이 타당하다는 뜻은 아니다.
        'delta_negative': None if delta_ms is None else delta_ms < 0.0,
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
          f"🔴 **이 값은 하한이다**(시작점은 렌더만큼 늦고 끝점은 이르게 잡힌다)")
    band = ' · '.join(f"{r:g}mm/s→{'?' if f is None else f'{f}f'}"
                      for r, f in sorted(v['sensitivity'].items()))
    print(f'  >>> 판정선 민감도: {band}')
    print()
    print(f"  구 기준(총 정지 ≤ fps×0.5 = {v['legacy_limit_frames']:.3f}f): "
          f"**{v['legacy_verdict']}**")
    print(f"  ⓐ 초안(총 정지 ≤ {PROPOSED_TOTAL_MS}ms = {v['proposed_limit_frames']:.1f}f): "
          f"**{v['proposed_verdict']}**  ⚠ 검토자 확인 대기")
    print(f"     🔴 {PROPOSED_TOTAL_MS}ms 는 **최악 지연의 증명이 아니라 사용자가 수용한 "
          f"운용 상한**이다")
    print(f"  [조건 2] 정지 후 {v['tail_ms']:.0f}ms 누적 회전 {v['drift_deg']:+.3f}° "
          f"= {v['drift_mm_s']:.4f} mm/s → "
          f"{'충족' if v['cond2_ok'] else '🔴 미충족'} (판정선 {MOTION_RATE_MM_S:g} mm/s)")
    if v['bag_ms'] is not None:
        print(f"  [교차] bag {v['bag_ms']:.1f}ms - 영상 {v['measured_ms']:.1f}ms "
              f"= 관측계 차이 {v['cross_observer_delta_ms']:+.1f}ms")
        print("         🔴 이 차이를 한 원인으로 특정하지 않는다 — bag 은 *저장* 시각, "
              "영상은 *화면 표시*→*회전 검출* 이라")
        print("            DDS·record 지연, odom 발행 주기, 두 T1 검출기 차이가 "
              "함께 들어 있다 (검토 §57.2)")
        if v['delta_negative']:
            print("         🔴 부호가 음수다 — 같은 시행인지, T0 를 옳게 읽었는지 "
                  "먼저 확인한다")


PRESETS = {
    # 2026-08-07 15:22:58 촬영(`IMG_3461.mov`) · bag `d0_watchdog_0807_1522` 와 같은 시행.
    # 중심은 **분석 시작 프레임(610)** 시점의 크롬 림 타원 중심, 4K 원본 좌표계. 2026-08-11 실측.
    # T0=670 · 축은 림 외곽(반축 112 × 178, 수직축이 긴 것은 바퀴가 비스듬히 보이기 때문).
    '0807-1522': dict(center=(439.3, 531.1), axes=(112.0, 178.0), frame_range=(610, 895)),
}


class UsageError(Exception):
    """입력 계약 위반. 🔴 traceback 이 아니라 **원인 + rc=2** 로 끝난다(검토 §57.4)."""


def _finite(raw, name):
    """외부에서 들어온 수 하나 — 수인지, 유한한지까지 본다(`nan`·`inf` 는 수다)."""
    try:
        value = float(str(raw).strip())
    except (TypeError, ValueError):
        raise UsageError(f'{name} 이 수가 아니다: {raw!r}') from None
    if not math.isfinite(value):
        raise UsageError(f'{name} 이 유한하지 않다: {raw!r}')
    return value


def _pair(raw, name, positive=False):
    """`"a,b"` — 개수까지 센다. 3개를 주면 조용히 앞 2개를 쓰지 않는다."""
    parts = str(raw).split(',')
    if len(parts) != 2:
        raise UsageError(f'{name} 은 "a,b" 두 값이다 — {len(parts)}개를 받았다: {raw!r}')
    values = tuple(_finite(p, name) for p in parts)
    if positive and not all(v > 0 for v in values):
        raise UsageError(f'{name} 은 두 값 모두 양수여야 한다: {raw!r}')
    return values


def parse_inputs(a, fps_probe=None):
    """🔴 **모든 외부 수치를 여기 한곳에서** 계약대로 검사한다.

    한곳에 모으는 이유: 검사가 흩어지면 새 인자가 늘 때마다 조용히 뚫린다
    (`scan_unbounded_cli.py` 가 화이트리스트에서 배운 것과 같은 교훈).
    """
    cfg = dict(PRESETS.get(a.preset, {}))
    if a.center:
        cfg['center'] = _pair(a.center, '--center')
    if a.axes:
        cfg['axes'] = _pair(a.axes, '--axes', positive=True)
    if a.range:
        cfg['frame_range'] = _pair(a.range, '--range')
    missing = [k for k in ('center', 'axes', 'frame_range') if k not in cfg]
    if missing:
        raise UsageError(f"--preset 이나 {'/'.join('--' + m for m in missing)} 가 필요하다")

    # preset 에서 온 값도 같은 검사를 받는다 — 계약은 출처가 아니라 값에 붙는다.
    cfg['center'] = _pair(','.join(map(repr, cfg['center'])), '--center')
    cfg['axes'] = _pair(','.join(map(repr, cfg['axes'])), '--axes', positive=True)
    lo, hi = _pair(','.join(map(repr, cfg['frame_range'])), '--range')
    if lo != int(lo) or hi != int(hi):
        raise UsageError(f'--range 는 정수 프레임 번호다: {cfg["frame_range"]!r}')
    if not 0 <= lo < hi:
        raise UsageError(f'--range 는 0 ≤ lo < hi 여야 한다: {int(lo)},{int(hi)}')
    cfg['frame_range'] = (int(lo), int(hi))

    t0 = _finite(a.t0_frame, '--t0-frame')
    if t0 != int(t0) or t0 < 0:
        raise UsageError(f'--t0-frame 은 0 이상 정수다: {a.t0_frame!r}')
    t0 = int(t0)

    fps = None if a.fps is None else _finite(a.fps, '--fps')
    if fps is None and fps_probe is not None:
        fps = fps_probe()
    if fps is None or not math.isfinite(fps) or fps <= 0:
        raise UsageError('fps 를 읽지 못했다 — --fps 로 양수를 준다')

    bag_ms = None if a.bag_ms is None else _finite(a.bag_ms, '--bag-ms')
    return cfg, t0, fps, bag_ms


def main(argv, series_fn=None):
    ap = argparse.ArgumentParser(
        description=(__doc__ or 'R0 watchdog 영상 판정기').splitlines()[0])
    ap.add_argument('video')
    # 🔴 수치는 전부 문자열로 받는다 — 검사를 `parse_inputs` 한곳에 모으기 위해서다.
    ap.add_argument('--t0-frame', required=True,
                    help='마지막 publishing #N 이 화면에 뜬 프레임(사람이 눈으로 확인)')
    ap.add_argument('--preset', choices=sorted(PRESETS))
    ap.add_argument('--center', help='바퀴 타원 중심 "x,y" (원본 좌표계)')
    ap.add_argument('--axes', help='바퀴 타원 반축 "a,b" (둘 다 양수)')
    ap.add_argument('--range', help='분석 프레임 "lo,hi" (0 ≤ lo < hi)')
    ap.add_argument('--fps', help='생략하면 영상에서 읽는다')
    ap.add_argument('--bag-ms', help='같은 시행의 bag 값 — 관측계 차이를 적어 둔다')
    a = ap.parse_args(argv[1:])

    def probe():
        """영상에서 fps 를 읽는다. 실패는 값이 아니라 `UsageError` 로 돌려준다."""
        try:
            import cv2                                       # noqa: PLC0415
            cap = cv2.VideoCapture(a.video)
            fps = cap.get(cv2.CAP_PROP_FPS)
            cap.release()
        except Exception as exc:                             # noqa: BLE001
            raise UsageError(f'fps 를 읽지 못했다 — --fps 로 준다: {exc}') from None
        return fps

    try:
        cfg, t0_frame, fps, bag_ms = parse_inputs(a, fps_probe=probe)
    except UsageError as exc:
        print(f'입력 오류 — {exc}', file=sys.stderr)
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

    # 🔴 `expected_range` 를 함께 준다 — 요청한 만큼 못 읽었으면(조기 EOF) 판정 불능이다.
    v = analyze(series, t0_frame, fps, bag_ms=bag_ms,
                expected_range=cfg['frame_range'])
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
