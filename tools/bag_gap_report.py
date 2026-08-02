#!/usr/bin/env python3
"""rosbag2 한 토픽의 '메시지 간격 분포'를 뽑는다 — R3 판정 도구 (2026-08-02 신설, S6-5).

사용:
    python3 tools/bag_gap_report.py <bag 디렉터리> <토픽> [<토픽> ...]
    예)  python3 tools/bag_gap_report.py ~/r3_bags/r3_0803 /odom /imu/data /scan

왜 이 도구가 필요한가
---------------------
구동부가 준 주기값은 세 번 다 **평균과 짧은 창의 최대**뿐이었다
(2차 회신 '41.63Hz · max 30ms' → 3차 회신 '46.4Hz · 20~23ms'). 두 번 다 **관측 창의
크기와 표본 수가 없어서** 그 최대를 계약상 상한으로 쓸 수 없었다
(`docs/REAL_ROBOT_VALUES.md §1`). 그래서 우리가 직접 분포를 뽑기로 했고, 그 재개방
조건이 *"R3 rosbag 에서 최대 간격이 33.33ms 를 넘는가"* 다.

★ 평균이 아니라 **최대 간격**을 보는 이유: EKF 를 무너뜨리는 것은 평균이 아니다.
  EKF 가 30Hz(33.33ms)로 도는데 입력 간격이 한 번이라도 그보다 길면, 그 주기는
  **입력 없이** 지나간다. 평균 46Hz 여도 가끔 40ms 가 섞이면 그때마다 구멍이 난다.

⚠ 이 도구는 **수신 시각**(bag 이 기록한 시각)으로 잰다. 메시지 안의 `header.stamp`
  가 아니다. 두 값은 다를 수 있고, 어느 쪽이 문제인지는 R3~R4 에서 가른다:
    · 수신 간격만 튄다 → 전송(USB·agent) 구간의 문제
    · header.stamp 도 같이 튄다 → 펌웨어 루프 자체의 문제
  TODO(R3): stamp 기준 비교가 필요해지면 이 도구에 한 갈래를 더한다.

[검증] 2026-08-02 노트북에서 가짜 46.5Hz BEST_EFFORT 퍼블리셔를 11.8초 녹화해
  실행 확인했다 — 549개 · 평균 46.50Hz · 최대 24.39ms · 33.33ms 초과 0건.
  (실차 데이터로는 아직 못 돌려 봤다 — 그건 R3 에서 한다)
"""
import sys

try:
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
except ImportError:                                    # pragma: no cover
    sys.exit('rosbag2_py 를 못 찾았다 — ROS 2 환경을 source 했는지 확인한다\n'
             '  source /opt/ros/humble/setup.bash')

GAP_LIMIT_MS = 33.33          # EKF 한 주기 (config/ekf_real.yaml frequency: 30.0)


def read_stamps(bag, topics):
    """bag 을 한 번만 훑어 토픽별 수신 시각(초) 목록을 만든다."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id='sqlite3'),
                ConverterOptions('', ''))
    out = {t: [] for t in topics}
    while reader.has_next():
        name, _data, t_ns = reader.read_next()
        if name in out:
            out[name].append(t_ns * 1e-9)
    return out


def report(topic, stamps):
    """한 토픽의 간격 분포를 찍고, 상한 초과가 있으면 True 를 돌려준다."""
    # ★ fail-closed: 표본이 없으면 '문제 없음'이 아니라 **판독 실패**로 말한다.
    #   빈 결과를 조용히 통과시키는 것이 이 저장소가 07-31 에 당한 사고다.
    if len(stamps) < 2:
        print('  %-12s ❌ 메시지 %d개 — 판독 실패 (토픽 이름·녹화 대상 확인)'
              % (topic, len(stamps)))
        return True

    gaps = sorted((b - a) * 1000.0 for a, b in zip(stamps, stamps[1:]))
    n = len(gaps)
    span = stamps[-1] - stamps[0]
    over = [g for g in gaps if g > GAP_LIMIT_MS]

    print('  %s  메시지 %d개 · 구간 %.1f초 · 평균 %.2f Hz'
          % (topic, len(stamps), span, n / span if span > 0 else 0.0))
    print('    간격(ms)  최소 %.2f · 중앙 %.2f · p95 %.2f · p99 %.2f · 최대 %.2f'
          % (gaps[0], gaps[n // 2], gaps[int(n * 0.95)], gaps[int(n * 0.99)], gaps[-1]))
    if over:
        print('    ❌ %.2fms 초과 %d건 (%.2f%%) — 최대 %.2fms'
              % (GAP_LIMIT_MS, len(over), 100.0 * len(over) / n, gaps[-1]))
        return True
    print('    ✅ %.2fms 초과 0건' % GAP_LIMIT_MS)
    return False


def main(argv):
    if len(argv) < 2:
        sys.exit(__doc__)
    bag, topics = argv[0], argv[1:]
    print('bag: %s' % bag)
    stamps = read_stamps(bag, topics)
    bad = [report(t, stamps[t]) for t in topics]
    print()
    if any(bad):
        print('❌ 상한 초과 또는 판독 실패가 있다.')
        print('   → docs/REAL_ROBOT_VALUES.md §1 의 IMU 주기 **재개방 조건이 걸렸다.**')
        print('     그 절과 src/tunnel_bringup/test/gate_fakes.py 의 주기 정본을 함께 다시 판단한다.')
        return 1
    print('✅ 전 토픽이 EKF 한 주기 안에 들어온다.')
    print('   ⚠ 단, 이것은 **이 녹화 구간에 대한** 사실이다. 더 긴 구간·다른 부하에서는')
    print('     달라질 수 있다 — 구동부 회신의 짧은 창을 우리가 비판한 것과 같은 한계다.')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
