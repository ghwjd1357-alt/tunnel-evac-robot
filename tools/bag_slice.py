#!/usr/bin/env python3
"""rosbag2(sqlite3) 에서 **구간 + 토픽**만 잘라 새 bag 을 만든다 (2026-10-02).

왜 필요한가 — 젯슨에 원본 bag 이 없고 회선이 느리다
---------------------------------------------------
디스플레이 촬영은 패널이 달린 **젯슨에서** 재생해야 하는데 `realtake6` 는 노트북에만
있고 324 MB 다. 폰 핫스팟(실측 ≈1 Mbps · `console/README.md`)으로 통째 복사는 45분이
걸린다. 필요한 것은 **몇 초 구간 + 화면이 쓰는 토픽**뿐이므로 그만 잘라 보낸다.

    python3 tools/bag_slice.py ~/robot_evidence/realtake6 OUT --start 125 --end 150
    python3 tools/bag_slice.py ... --window 131.5:135.5 --window 145:148.5   # 여러 구간을 이어 붙인다

🔴 **래치 토픽 되살리기** — `/map` 과 `/tf_static` 은 bag **맨 앞에 한 번만** 들어 있다.
구간만 자르면 지도가 영영 안 온다(화면이 "지도 수신 대기"로 남는다). 그래서 구간 시작
**이전의 마지막 메시지**를 찾아 구간 시작 시각으로 **다시 찍어** 맨 앞에 넣는다.
내용은 그대로이고 바뀌는 것은 '재생될 시각'뿐이다. 정적 변환·격자지도는 시각에
의존하지 않으므로 안전하다 (움직이는 `/tf` 에는 쓰지 않는다 — PRIME_TOPICS 고정).

🔵 **여러 구간 이어 붙이기** — `--window A:B` 를 여러 번 주면 구간들이 **연속으로** 재생되도록
뒤 구간의 시각을 앞으로 당겨 붙인다. 한 상태가 길어서(예: `GATHER` 12.5초) 클립이 늘어질 때
그 가운데를 들어낸다. ⚠ **로봇이 서 있는 구간에서만** 쓴다 — 움직이는 중에 자르면 지도 위
로봇이 순간이동한다. 이음매가 보이는지는 만든 뒤 눈으로 확인한다.

⚠ 이 도구는 **증거를 만들지 않는다.** 산출물은 촬영·시연용 재생 재료이고, 측정·분석은
원본 bag 으로 한다 (잘린 bag 으로 잰 수치를 인용하지 않는다).
"""
import argparse, pathlib, shutil, sqlite3, sys

# 화면(관제·디스플레이)이 실제로 구독하는 토픽 — `console/js/ros.js` 의 sub() 목록.
DISPLAY_TOPICS = [
    '/mission_state', '/siren', '/person_status', '/victim',
    '/map', '/tf', '/tf_static', '/plan', '/alarm', '/scan',
    '/local_costmap/costmap', '/odometry/filtered',
    '/drive/enabled', '/drive/diag', '/estop/state',
]
# 맨 앞에 한 번만 오는 래치 토픽 — 구간 앞의 마지막 1건을 끌어온다.
PRIME_TOPICS = ['/map', '/tf_static']


def plan_messages(rows, windows, prime_topic_ids):
    """어떤 메시지를 가져갈지 정한다 — 순수 함수(테스트가 잠근다).

    rows    = (id, topic_id, timestamp) 의 **시간순** 목록.
    windows = [(start_ns, end_ns)] — 시간순. 둘 이상이면 뒤 구간을 앞 구간 끝에 **이어 붙인다**.
    반환    = [(id, 새 timestamp)]. 래치 프라임은 첫 구간 시작 시각으로 당긴다.
    """
    first_start = windows[0][0]
    out, prime, shift, prev_end = [], {}, 0, None
    for wi, (ws, we) in enumerate(windows):
        if wi:                                  # 앞 구간 끝과 이 구간 시작 사이의 공백을 없앤다
            shift += ws - prev_end
        for mid, tid, ts in rows:
            if ts < ws:
                if wi == 0 and tid in prime_topic_ids:
                    prime[tid] = mid            # 첫 구간 앞의 **마지막** 1건만
                continue
            if ts > we:
                break
            out.append((mid, ts - shift))
        prev_end = we
    primed = [(mid, first_start) for mid in prime.values()]
    return primed + out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('src', help='원본 bag 디렉터리')
    ap.add_argument('dst', help='만들 bag 디렉터리 (있으면 지운다)')
    ap.add_argument('--start', type=float, help='시작 [s, bag 기록시각 기준]')
    ap.add_argument('--end', type=float, help='끝 [s]')
    ap.add_argument('--window', action='append', default=[], metavar='A:B',
                    help='구간 [s] — 여러 번 주면 이어 붙인다 (--start/--end 대신)')
    ap.add_argument('--topics', nargs='*', default=DISPLAY_TOPICS,
                    help='가져갈 토픽 (기본 = 화면이 구독하는 15종)')
    a = ap.parse_args()

    src = pathlib.Path(a.src).expanduser()
    dst = pathlib.Path(a.dst).expanduser()
    db_in = sorted(src.glob('*.db3'))
    if not db_in:
        sys.exit(f'bag 에 .db3 가 없다: {src}')
    spans = []
    for w in a.window:
        try:
            lo, hi = (float(x) for x in w.split(':'))
        except ValueError:
            sys.exit(f'--window 형식은 A:B 다: {w}')
        spans.append((lo, hi))
    if a.start is not None and a.end is not None:
        spans.append((a.start, a.end))
    if not spans:
        sys.exit('--start/--end 또는 --window 가 필요하다')
    spans.sort()
    for lo, hi in spans:
        if hi <= lo:
            sys.exit(f'구간 끝이 시작보다 뒤여야 한다: {lo}:{hi}')
    for (p_lo, p_hi), (n_lo, _) in zip(spans, spans[1:]):
        if n_lo < p_hi:
            sys.exit('구간이 겹친다 — 겹치면 같은 메시지가 두 번 들어간다')
    if dst.exists():
        shutil.rmtree(dst)
    dst.mkdir(parents=True)

    con = sqlite3.connect(str(db_in[0]))
    t0 = con.execute('select min(timestamp) from messages').fetchone()[0]
    windows = [(t0 + int(lo * 1e9), t0 + int(hi * 1e9)) for lo, hi in spans]

    topics = {name: (tid, typ, ser) for tid, name, typ, ser in
              con.execute('select id,name,type,serialization_format from topics')}
    want = {n: v for n, v in topics.items() if n in a.topics}
    missing = [t for t in a.topics if t not in want]
    if missing:
        print(f'⚠ 원본에 없는 토픽(건너뜀): {" ".join(missing)}')
    want_ids = {v[0] for v in want.values()}
    prime_ids = {want[n][0] for n in PRIME_TOPICS if n in want}

    rows = [r for r in con.execute(
        'select id,topic_id,timestamp from messages order by timestamp') if r[1] in want_ids]
    picked = plan_messages(rows, windows, prime_ids)
    if not picked:
        sys.exit('구간에 메시지가 없다 — --start/--end 를 확인하라')

    out_db = dst / f'{dst.name}_0.db3'
    oc = sqlite3.connect(str(out_db))
    oc.executescript("""
        CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL,
            type TEXT NOT NULL, serialization_format TEXT NOT NULL,
            offered_qos_profiles TEXT NOT NULL);
        CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL,
            timestamp INTEGER NOT NULL, data BLOB NOT NULL);
        CREATE INDEX timestamp_idx ON messages (timestamp ASC);
    """)
    # QoS 프로필은 원본 그대로 옮긴다 — 래치(transient_local) 설정이 여기 들어 있다
    qos = {tid: q for tid, q in con.execute('select id,offered_qos_profiles from topics')}
    for name, (tid, typ, ser) in want.items():
        oc.execute('INSERT INTO topics VALUES (?,?,?,?,?)', (tid, name, typ, ser, qos[tid]))

    picked.sort(key=lambda x: x[1])      # 이어 붙인 뒤에는 새 시각 순서로 기록한다
    counts, new_id = {}, 0
    for mid, ts in picked:
        tid, data = con.execute(
            'select topic_id,data from messages where id=?', (mid,)).fetchone()
        new_id += 1
        oc.execute('INSERT INTO messages VALUES (?,?,?,?)', (new_id, tid, ts, data))
        counts[tid] = counts.get(tid, 0) + 1
    oc.commit()
    lo = oc.execute('select min(timestamp),max(timestamp) from messages').fetchone()
    oc.close(); con.close()

    dur_ns = lo[1] - lo[0]
    by_name = {n: counts.get(v[0], 0) for n, v in want.items()}

    # 🔴 metadata.yaml 은 **원본을 읽어 고친다** — 손으로 쓰면 QoS 프로필 문자열(줄바꿈이
    #   들어 있는 YAML 스칼라)에서 깨진다. 10-02 에 실제로 깨졌고 `ros2 bag info` 는
    #   통과하는데 `ros2 bag play` 만 "illegal map value" 로 거부해 원인이 늦게 보였다.
    import yaml                                               # noqa: PLC0415
    meta = yaml.safe_load((src / 'metadata.yaml').read_text(encoding='utf-8'))
    info = meta['rosbag2_bagfile_information']
    info['relative_file_paths'] = [out_db.name]
    info['duration'] = {'nanoseconds': dur_ns}
    info['starting_time'] = {'nanoseconds_since_epoch': lo[0]}
    info['message_count'] = new_id
    info['topics_with_message_count'] = [
        t for t in info['topics_with_message_count']
        if t['topic_metadata']['name'] in want]
    for t in info['topics_with_message_count']:
        t['message_count'] = by_name[t['topic_metadata']['name']]
    info['files'] = [{'path': out_db.name, 'message_count': new_id,
                      'starting_time': {'nanoseconds_since_epoch': lo[0]},
                      'duration': {'nanoseconds': dur_ns}}]
    (dst / 'metadata.yaml').write_text(
        yaml.safe_dump(meta, allow_unicode=True, sort_keys=False), encoding='utf-8')

    size = sum(f.stat().st_size for f in dst.iterdir())
    print(f'🟢 {dst}  {size/1e6:.1f} MB · {new_id} 건 · {dur_ns/1e9:.1f} s')
    for name in sorted(by_name, key=lambda n: -by_name[n]):
        if by_name[name]:
            print(f'   {by_name[name]:6d}  {name}' +
                  ('   ← 래치 프라임' if name in PRIME_TOPICS else ''))


if __name__ == '__main__':
    main()
