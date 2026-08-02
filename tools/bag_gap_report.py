#!/usr/bin/env python3
"""rosbag2 한 토픽의 '메시지 간격 분포'와 'stamp 단조성'을 판정한다 — R3 도구 (S6-5).

사용:
    python3 tools/bag_gap_report.py <bag 디렉터리> <토픽> [<토픽> ...]
    예)  python3 tools/bag_gap_report.py ~/r3_bags/r3_0803 /odom /imu/data /scan

왜 이 도구가 필요한가
---------------------
구동부가 준 주기값은 세 번 다 **평균과 짧은 창의 최대**뿐이었고, 관측 창의 크기·표본 수가
없어서 그 최대를 계약상 상한으로 쓸 수 없었다 (`docs/REAL_ROBOT_VALUES.md §1`).
그래서 우리가 직접 분포를 뽑기로 했고, IMU 주기 재개방 조건이
*"R3 rosbag 에서 최대 간격이 33.33ms 를 넘는가"* 다.

★ 평균이 아니라 **최대 간격**을 보는 이유: EKF 를 무너뜨리는 것은 평균이 아니다.
  EKF 가 30Hz(33.33ms)로 도는데 입력 간격이 한 번이라도 그보다 길면, 그 주기는
  **입력 없이** 지나간다. 평균 46Hz 여도 가끔 40ms 가 섞이면 그때마다 구멍이 난다.

무엇을 보는가 (3종) — ★ 08-02 검토 §29.4 로 두 가지가 추가됐다
--------------------------------------------------------------
1. **내부 간격** — 연속한 두 메시지 사이.
2. **bag 양끝 공백** ★신설 — 구판은 각 토픽의 *첫~마지막 메시지 사이만* 봤다. 그래서
   bag 시작 뒤 늦게 살아났거나 종료 훨씬 전에 **영구 정지한** 구간이 간격 목록에 아예
   없었다(= 150초 bag 중 10초만 살아 있어도 "전 토픽이 EKF 한 주기 안" 이라는 녹색).
   → bag 메타데이터의 시작·종료와 각 토픽의 첫·마지막 수신을 **결합**해 앞뒤 공백을
     간격으로 포함한다.
   ※ `starting_time` 은 녹화 명령 시각이 아니라 **bag 안 첫 메시지 시각**이고
     `starting_time + duration` 이 마지막 메시지 시각이다(08-02 실측 확인). 그래서
     녹화 기동 지연 때문에 거짓 FAIL 이 나지 않는다.
3. **`header.stamp` 엄격 단조** ★신설 — 구판은 수신시각만 읽고 `_data` 를 버려서
   R3 완료조건인 "timestamp 단조"를 **원리상 검사하지 않았다**(검토자 실측: 동일 시각열·
   역행 시각열 모두 녹색). stamp 가 같거나 뒤로 가면 EKF 는 `dt<=0` 을 받아 멈추거나 발산한다.
   → 메시지를 실제로 역직렬화해 `header.stamp` 를 검사한다.
     `header` 가 없는 타입은 **N/A** 로 표시하고, 아래 STAMP_REQUIRED 토픽에는 필수로 건다.

⚠ 수신시각과 `header.stamp` 는 다른 것을 말한다. 둘을 함께 보면 원인이 갈린다:
    · 수신 간격만 튄다        → 전송(USB·agent) 구간의 문제
    · `header.stamp` 도 튄다  → 펌웨어 루프 자체의 문제

[검증] 2026-08-02 노트북에서 합성 bag 6종(정상·늦은 시작·이른 종료·동일 stamp·역행 stamp·
  내부 40ms)으로 실행 확인했다. 실차 데이터로는 아직 못 돌려 봤다 — 그건 R3 에서 한다.
"""
import sys

try:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, Info, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:                             # pragma: no cover
    sys.exit('ROS 2 모듈을 못 찾았다 (%s) — 환경을 source 했는지 확인한다\n'
             '  source /opt/ros/humble/setup.bash' % exc)

GAP_LIMIT_MS = 33.33          # EKF 한 주기 (config/ekf_real.yaml frequency: 30.0)
STAMP_REQUIRED = ('/odom', '/imu/data', '/scan')   # stamp 단조를 반드시 확인할 토픽
STORAGE = 'sqlite3'


def read_bag(bag, topics):
    """bag 을 한 번 훑어 토픽별 (수신시각[s], header.stamp[s]) 를 모은다.

    stamp 가 없는 타입이면 stamps 자리에 None 을 남긴다(빈 목록과 구분하기 위해서다 —
    '헤더가 없다'와 '메시지가 없다'는 다른 사실이다).
    """
    meta = Info().read_metadata(bag, STORAGE)
    types = {t.topic_metadata.name: t.topic_metadata.type
             for t in meta.topics_with_message_count}
    bag_start = meta.starting_time.nanoseconds * 1e-9
    bag_end = bag_start + meta.duration.nanoseconds * 1e-9

    classes, recv, stamps = {}, {}, {}
    for t in topics:
        recv[t] = []
        cls = None
        if t in types:
            try:
                cls = get_message(types[t])
            except (ImportError, ValueError, AttributeError):
                cls = None                     # 타입을 못 불러오면 stamp 검사는 N/A
        classes[t] = cls
        stamps[t] = [] if (cls is not None and hasattr(cls(), 'header')) else None

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id=STORAGE), ConverterOptions('', ''))
    while reader.has_next():
        name, data, t_ns = reader.read_next()
        if name not in recv:
            continue
        recv[name].append(t_ns * 1e-9)
        if stamps[name] is not None:
            msg = deserialize_message(data, classes[name])
            st = msg.header.stamp
            stamps[name].append(st.sec + st.nanosec * 1e-9)
    return bag_start, bag_end, recv, stamps


def check_stamps(topic, stamps):
    """header.stamp 가 **엄격 단조 증가**인가. 결함이면 True."""
    if stamps is None:
        if topic in STAMP_REQUIRED:
            print('    ❌ header 가 없는 타입이다 — 이 토픽은 stamp 단조가 필수다')
            return True
        print('    –  stamp 검사 N/A (header 없는 타입)')
        return False

    dup = [i for i in range(1, len(stamps)) if stamps[i] == stamps[i - 1]]
    back = [i for i in range(1, len(stamps)) if stamps[i] < stamps[i - 1]]
    if dup or back:
        if dup:
            print('    ❌ stamp 중복 %d건 (첫 자리 #%d) — dt=0 이면 EKF 가 시간이 멈춘 것으로 본다'
                  % (len(dup), dup[0]))
        if back:
            worst = min((stamps[i] - stamps[i - 1]) for i in back) * 1000.0
            print('    ❌ stamp 역행 %d건 (최대 %.2fms 뒤로) — TF·EKF 가 통째로 무너진다'
                  % (len(back), worst))
        return True

    sgaps = sorted((b - a) * 1000.0 for a, b in zip(stamps, stamps[1:]))
    print('    ✅ stamp 엄격 단조 (stamp 간격 최대 %.2fms)' % (sgaps[-1] if sgaps else 0.0))
    return False


def report(topic, recv, stamps, bag_start, bag_end):
    """한 토픽을 판정하고, 결함이 있으면 True 를 돌려준다."""
    # ★ fail-closed: 표본이 없으면 '문제 없음'이 아니라 **판독 실패**로 말한다.
    if len(recv) < 2:
        print('  %-12s ❌ 메시지 %d개 — 판독 실패 (토픽 이름·녹화 대상 확인)'
              % (topic, len(recv)))
        return True

    # 앞·뒤 공백은 정의상 음수가 될 수 없다(bag 양끝 = 전체 메시지의 첫·마지막).
    # 부동소수 오차로 -0.000000001 이 찍히면 읽는 사람이 헷갈리므로 0 으로 죈다.
    # ⚠ **내부** 간격은 죄지 않는다 — 거기서 음수가 나오면 수신 순서가 뒤집힌 것이고
    #   그건 숨기면 안 되는 사실이다.
    lead = max(0.0, (recv[0] - bag_start) * 1000.0)     # bag 시작 → 첫 수신 (앞 공백)
    trail = max(0.0, (bag_end - recv[-1]) * 1000.0)     # 마지막 수신 → bag 종료 (뒤 공백)
    inner = [(b - a) * 1000.0 for a, b in zip(recv, recv[1:])]
    gaps = sorted(inner + [lead, trail])         # ★ 양끝을 간격에 **포함**한다
    n = len(gaps)
    span = bag_end - bag_start

    print('  %s  메시지 %d개 · bag 구간 %.1f초 · 평균 %.2f Hz'
          % (topic, len(recv), span, len(recv) / span if span > 0 else 0.0))
    print('    간격(ms)  최소 %.2f · 중앙 %.2f · p95 %.2f · p99 %.2f · 최대 %.2f'
          % (gaps[0], gaps[n // 2], gaps[int(n * 0.95)], gaps[int(n * 0.99)], gaps[-1]))
    print('    양끝      bag 시작→첫 수신 %.2fms · 마지막 수신→bag 종료 %.2fms' % (lead, trail))

    bad = False
    over = [g for g in gaps if g > GAP_LIMIT_MS]
    if over:
        where = []
        if lead > GAP_LIMIT_MS:
            where.append('앞 공백')
        if trail > GAP_LIMIT_MS:
            where.append('뒤 공백')
        if [g for g in inner if g > GAP_LIMIT_MS]:
            where.append('내부')
        print('    ❌ %.2fms 초과 %d건 (%.2f%%) — 최대 %.2fms · 위치: %s'
              % (GAP_LIMIT_MS, len(over), 100.0 * len(over) / n, gaps[-1], ', '.join(where)))
        bad = True
    else:
        print('    ✅ %.2fms 초과 0건 (양끝 공백 포함)' % GAP_LIMIT_MS)

    return check_stamps(topic, stamps) or bad


def main(argv):
    if len(argv) < 2:
        sys.exit(__doc__)
    bag, topics = argv[0], argv[1:]
    print('bag: %s' % bag)
    bag_start, bag_end, recv, stamps = read_bag(bag, topics)
    bad = [report(t, recv[t], stamps[t], bag_start, bag_end) for t in topics]
    print()
    if any(bad):
        print('❌ 상한 초과 · stamp 결함 · 판독 실패 중 하나가 있다.')
        print('   → 간격이면 `REAL_ROBOT_VALUES.md §1` 의 IMU 주기 **재개방 조건이 걸렸다.**')
        print('     그 절과 src/tunnel_bringup/test/gate_fakes.py 의 주기 정본을 함께 다시 판단한다.')
        print('   → stamp 결함이면 **펌웨어 쪽**이다. 우리 yaml 로 못 고친다.')
        return 1
    print('✅ 전 토픽이 bag 전 구간에서 EKF 한 주기 안에 있고 stamp 도 엄격 단조다.')
    print('   ⚠ 단, 이것은 **이 녹화 구간에 대한** 사실이다. 더 긴 구간·다른 부하에서는')
    print('     달라질 수 있다 — 구동부 회신의 짧은 창을 우리가 비판한 것과 같은 한계다.')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
