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

★★ 08-03 검토 §30.2 — **토픽마다 계약이 다르다. 하나의 상수로 전부 재면 정상을 거부한다.**
  구판은 `GAP_LIMIT_MS = 33.33` **하나를 모든 토픽에** 적용했다. 그런데 33.33ms 는
  *EKF 한 주기*이고 EKF 입력은 `/odom`·`/imu/data` 뿐이다. `/scan` 은 RPLIDAR C1
  **사양 10Hz = 100ms** 라(주기 정본 = `src/tunnel_bringup/test/gate_fakes.py` 의
  `SCAN_PERIOD_US = 100_000`), **완벽하게 정상인 스캔이 전부 상한 초과**로 찍혔다.
  검토자 실측: bag 전 구간을 채운 정상 10Hz scan 11개 → `33.33ms 초과 10건 · 최대 100.00ms`.
  → 실차 R3 에서 멀쩡한 라이다 데이터가 "IMU 주기 결함"으로 읽히고, 이 도구의 안내대로
    IMU 주기 정본과 **동결된 하네스를 불필요하게 재개방**시켰을 것이다.
    현장 데이터의 결함이 아니라 **서로 다른 센서 계약을 하나로 합친 오경보**다.
  → 그래서 토픽별 계약을 아래 **`TOPIC_POLICY` 한 곳**에 두고, 판정기도 런북
    (`docs/D1_FIRST_STEP.md §5-b`)도 같은 표를 본다. 계약이 두 곳에 적히면 갈라진다.

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
     `header` 가 없는 타입은 **N/A** 로 표시하고, `TOPIC_POLICY` 의 `stamp: True`
     토픽에는 필수로 건다.

⚠ 수신시각과 `header.stamp` 는 다른 것을 말한다. 둘을 함께 보면 원인이 갈린다:
    · 수신 간격만 튄다        → 전송(USB·agent) 구간의 문제
    · `header.stamp` 도 튄다  → 펌웨어 루프 자체의 문제

[검증] 2026-08-02 노트북에서 합성 bag 6종(정상·늦은 시작·이른 종료·동일 stamp·역행 stamp·
  내부 40ms)으로 실행 확인했다. 2026-08-03 검토 §30.2 보완으로 **정상 10Hz `/scan` 역회귀와
  토픽별 경계값**을 합성 bag 으로 추가 확인했다(증거 = `docs/FREEZE_MANIFEST.md §10.6`).
  실차 데이터로는 아직 못 돌려 봤다 — 그건 R3 에서 한다.
"""
import sys

try:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, Info, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:                                        # pragma: no cover
    sys.exit('ROS 2 모듈을 못 찾았다 (%s) — 환경을 source 했는지 확인한다\n'
             '  source /opt/ros/humble/setup.bash' % exc)

# ── 토픽별 계약 = 이 표 한 곳이 정본이다 (★ 08-03 검토 §30.2) ────────────────
#
# EKF_PERIOD_MS  : EKF 한 주기. **EKF 입력 토픽에만** 적용되는 계약이다.
# LIVENESS_MISS  : "몇 주기가 통째로 비면 그 구간엔 이 토픽이 없었다"고 볼 것인가.
#
# ★ LIVENESS_MISS 가 왜 2 인가 (임의의 숫자가 아니다):
#   bag 의 시작·끝은 토픽의 발행 **위상**과 맞춰져 있지 않다. 그래서 완벽하게 건강한
#   토픽도 양끝에서 **최대 한 주기까지는** 비어 있는 것이 정상이다(녹화가 주기 중간에
#   시작·종료되므로). 두 주기가 통째로 비면 그건 위상 문제가 아니라 **결측**이다.
#   ※ EKF 입력 2종은 이 규칙보다 계약이 더 빡빡하다(50Hz odom 한 주기 20ms < 33.33ms)
#     — 그래서 그쪽은 계약이 이기고, 판정은 구판과 똑같이 33.33ms 로 유지된다.
EKF_PERIOD_MS = 33.33     # config/ekf_real.yaml `frequency: 30.0` → 1/30s
LIVENESS_MISS = 2

# gap_ms      : 간격 상한 계약. **None = 아직 계약이 없다**(실측 전이라 발명하지 않는다).
# nominal_ms  : 공칭 주기. 계약이 없을 때 liveness 한계(= nominal × LIVENESS_MISS)의 근거.
# stamp       : header.stamp 엄격 단조를 **필수**로 볼 것인가.
TOPIC_POLICY = {
    '/odom': {
        'gap_ms': EKF_PERIOD_MS, 'nominal_ms': 20.0, 'stamp': True,
        'why': 'EKF 입력 — 간격이 EKF 한 주기를 넘으면 그 주기는 입력 없이 지나간다',
    },
    '/imu/data': {
        'gap_ms': EKF_PERIOD_MS, 'nominal_ms': 21.5, 'stamp': True,
        'why': 'EKF 입력 — 위와 같다 (구동부 3차 회신 실측 46.4Hz ≈ 21.5ms)',
    },
    '/scan': {
        'gap_ms': None, 'nominal_ms': 100.0, 'stamp': True,
        'why': 'EKF 입력이 아니다. RPLIDAR C1 **사양** 10Hz=100ms 이고 실측은 미확보라 '
               '간격 계약을 아직 두지 않는다(R3 가 정한다). liveness 와 stamp 만 판정한다',
    },
}
STORAGE = 'sqlite3'


def policy_for(topic, median_ms):
    """이 토픽에 적용할 계약을 돌려준다 — (한계값 ms, 종류, 근거 문장).

    종류 'contract' = 간격 상한 계약(위반 = IMU 주기 재개방 조건)
    종류 'liveness' = 계약이 없다. **살아 있었는가**만 본다(위반 = 그 구간에 토픽이 없었다)

    표에 없는 토픽은 **그 토픽 자신의 관측 중앙 간격**을 공칭 주기로 삼는다.
    모르는 토픽에 임의의 숫자를 발명하지 않기 위해서다 — 대신 자기 리듬으로 결측만 잡는다.
    """
    pol = TOPIC_POLICY.get(topic)
    if pol is None:
        base = median_ms if median_ms > 0 else 0.0
        return (base * LIVENESS_MISS, 'liveness',
                '계약 표에 없는 토픽 — 관측 중앙 간격 %.2fms × %d 로 결측만 본다'
                % (median_ms, LIVENESS_MISS))
    if pol['gap_ms'] is not None:
        return pol['gap_ms'], 'contract', pol['why']
    return pol['nominal_ms'] * LIVENESS_MISS, 'liveness', pol['why']


def stamp_required(topic):
    """이 토픽에 header.stamp 단조를 **필수**로 걸 것인가."""
    pol = TOPIC_POLICY.get(topic)
    return bool(pol and pol['stamp'])


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
        if stamp_required(topic):
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
    """한 토픽을 자기 계약으로 판정하고, 무엇이 깨졌는지 종류별로 돌려준다.

    돌려주는 dict — 'bad' 하나만 보고 종료코드를 정하고, 나머지는 **안내문을 고르는** 데 쓴다.
    (같은 FAIL 이라도 '재개방 조건이 걸렸다'와 '그 구간에 토픽이 없었다'는 다음 행동이 다르다)
    """
    out = {'bad': False, 'contract': False, 'liveness': False, 'stamp': False, 'read': False}

    # ★ fail-closed: 표본이 없으면 '문제 없음'이 아니라 **판독 실패**로 말한다.
    if len(recv) < 2:
        print('  %-12s ❌ 메시지 %d개 — 판독 실패 (토픽 이름·녹화 대상 확인)'
              % (topic, len(recv)))
        out['bad'] = out['read'] = True
        return out

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

    inner_sorted = sorted(inner)
    median_inner = inner_sorted[len(inner_sorted) // 2] if inner_sorted else 0.0
    limit_ms, kind, why = policy_for(topic, median_inner)

    print('  %s  메시지 %d개 · bag 구간 %.1f초 · 평균 %.2f Hz'
          % (topic, len(recv), span, len(recv) / span if span > 0 else 0.0))
    print('    간격(ms)  최소 %.2f · 중앙 %.2f · p95 %.2f · p99 %.2f · 최대 %.2f'
          % (gaps[0], gaps[n // 2], gaps[int(n * 0.95)], gaps[int(n * 0.99)], gaps[-1]))
    print('    양끝      bag 시작→첫 수신 %.2fms · 마지막 수신→bag 종료 %.2fms' % (lead, trail))
    print('    계약      %s %.2fms — %s'
          % ('간격 상한' if kind == 'contract' else 'liveness 한계', limit_ms, why))

    over = [g for g in gaps if g > limit_ms]
    if over:
        where = []
        if lead > limit_ms:
            where.append('앞 공백')
        if trail > limit_ms:
            where.append('뒤 공백')
        if [g for g in inner if g > limit_ms]:
            where.append('내부')
        wtxt = ', '.join(where)
        if kind == 'contract':
            print('    ❌ %.2fms 초과 %d건 (%.2f%%) — 최대 %.2fms · 위치: %s'
                  % (limit_ms, len(over), 100.0 * len(over) / n, gaps[-1], wtxt))
            out['contract'] = True
        else:
            print('    ❌ 이 토픽이 최대 %.2fms 동안 비어 있었다 (liveness 한계 %.2fms · %d건)'
                  % (gaps[-1], limit_ms, len(over)))
            print('       위치: %s — **간격 계약 위반이 아니라 그 구간에 토픽이 없었다**는 뜻이다'
                  % wtxt)
            out['liveness'] = True
        out['bad'] = True
    elif kind == 'contract':
        print('    ✅ %.2fms 초과 0건 (양끝 공백 포함)' % limit_ms)
    else:
        print('    ✅ bag 전 구간에서 끊긴 적 없음 (최대 %.2fms ≤ liveness 한계 %.2fms)'
              % (gaps[-1], limit_ms))
        print('       ※ 이 토픽의 간격 **분포는 계약이 아니라 관측값**이다 — R3 가 정본을 만든다')

    if check_stamps(topic, stamps):
        out['bad'] = out['stamp'] = True
    return out


def main(argv):
    if len(argv) < 2:
        sys.exit(__doc__)
    bag, topics = argv[0], argv[1:]
    print('bag: %s' % bag)
    bag_start, bag_end, recv, stamps = read_bag(bag, topics)
    results = [report(t, recv[t], stamps[t], bag_start, bag_end) for t in topics]
    print()
    if not any(r['bad'] for r in results):
        print('✅ 전 토픽이 bag 전 구간에서 **자기 계약** 안에 있고 stamp 도 엄격 단조다.')
        print('   ⚠ 단, 이것은 **이 녹화 구간에 대한** 사실이다. 더 긴 구간·다른 부하에서는')
        print('     달라질 수 있다 — 구동부 회신의 짧은 창을 우리가 비판한 것과 같은 한계다.')
        return 0

    print('❌ 계약 초과 · 결측 · stamp 결함 · 판독 실패 중 하나가 있다.')
    if any(r['contract'] for r in results):
        print('   → **간격 계약**(EKF 한 주기)을 넘었다 = `REAL_ROBOT_VALUES.md §1` 의')
        print('     IMU 주기 **재개방 조건이 걸렸다.** 그 절과')
        print('     src/tunnel_bringup/test/gate_fakes.py 의 주기 정본을 함께 다시 판단한다.')
    if any(r['liveness'] for r in results):
        print('   → **결측**(liveness)은 재개방 조건이 아니다. 그 구간에 토픽이 아예 없었던 것이라')
        print('     센서·드라이버가 늦게 떴거나 도중에 죽은 것부터 본다(녹화를 다시 하면 된다).')
    if any(r['stamp'] for r in results):
        print('   → **stamp 결함**이면 펌웨어(또는 Jetson 시계) 쪽이다. 우리 yaml 로 못 고친다.')
    if any(r['read'] for r in results):
        print('   → **판독 실패**는 "간격이 0" 이 아니다. 토픽 이름·녹화 대상을 먼저 확인한다.')
    return 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
