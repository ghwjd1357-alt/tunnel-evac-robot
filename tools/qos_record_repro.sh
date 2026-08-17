#!/usr/bin/env bash
# qos_record_repro.sh — 예약 43 ② 의 결함과 보완을 **로봇 없이** 재현한다.
# =====================================================================
# [무엇을 증명하나]
#   `ros2 bag record` 가 TRANSIENT_LOCAL 발행자만 보고 구독을 만들면, 나중에 뜬
#   VOLATILE 발행자(= `teleop_twist_keyboard`)의 메시지가 **에러 없이 0건**이 된다.
#   그리고 `--qos-profile-overrides-path` 로 구독을 VOLATILE 로 고정하면 전부 회수된다.
#
# [왜 도메인 77 인가]
#   🔴 실로봇과 절대 만나지 않게 격리한다. 이 스크립트는 `/cmd_vel` 을 쓰지 않고
#   `/qos_probe` 라는 더미 토픽만 쓴다 — 실수로도 로봇이 움직이지 않는다.
#
# [실행]
#   bash tools/qos_record_repro.sh
#   기대: ① 비영 0건 (결함 재현)   ② 비영 60건 (보완 성립)
#   rc=0 이면 둘 다 기대대로다. rc=1 이면 둘 중 하나가 어긋난 것이고,
#   그때는 `MASTER_PLAN §7` 예약 43-b 의 결론을 다시 판단한다.
#
# ⚠ `set -u` 를 쓰지 않는다 — ROS `setup.bash` 가 미설정 변수를 참조해 즉시 죽는다.

export ROS_DOMAIN_ID=77
source /opt/ros/humble/setup.bash

OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

cat > "$OUT/override.yaml" <<'YAML'
/qos_probe:
  history: keep_all
  depth: 0
  reliability: reliable
  durability: volatile
YAML

# teleop 과 같은 QoS(rclpy 기본 = RELIABLE + VOLATILE)로 6초간 발행한다.
# linear.x = 1.0 으로 보내 `topic pub` 이 보낸 0 과 구분한다.
cat > "$OUT/vol_pub.py" <<'PY'
import sys, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
rclpy.init()
n = Node('vol_pub')
p = n.create_publisher(Twist, '/qos_probe', 10)
m = Twist(); m.linear.x = 1.0
end = time.time() + float(sys.argv[1])
while time.time() < end:
    p.publish(m)
    time.sleep(0.1)
n.destroy_node(); rclpy.shutdown()
PY

# 결과를 파일로 받아 상위에서 판정한다 (파이프 서브셸에 갇히지 않게).
run_case () {
  local name="$1"; shift
  local bagdir="$OUT/$name"

  # ① TRANSIENT_LOCAL 발행자만 먼저 존재한다 (`ros2 topic pub` 의 기본값이다)
  ros2 topic pub -r 10 /qos_probe geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 &
  local TLPID=$!
  sleep 3

  # ② rosbag2 가 **여기서** 구독을 만든다 — 이 순간의 발행자만 협상에 낀다
  ros2 bag record /qos_probe -o "$bagdir" "$@" >/dev/null 2>&1 &
  local BAGPID=$!
  sleep 4

  # ③ 그 다음에야 VOLATILE 발행자가 뜬다 (= 런북에서 teleop 이 뜨는 자리)
  python3 "$OUT/vol_pub.py" 6
  sleep 2

  kill -INT "$BAGPID" 2>/dev/null; wait "$BAGPID" 2>/dev/null
  kill      "$TLPID"  2>/dev/null; wait "$TLPID"  2>/dev/null
  sleep 1

  python3 - "$bagdir" > "$OUT/$name.count" <<'PY'
import sqlite3, glob, sys, os
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Twist
dbs = glob.glob(os.path.join(sys.argv[1], '*.db3'))
if not dbs:
    print('-1 -1'); raise SystemExit
c = sqlite3.connect(dbs[0])
rows = c.execute("SELECT m.data FROM messages m JOIN topics t ON m.topic_id=t.id "
                 "WHERE t.name='/qos_probe'").fetchall()
nz = sum(1 for (b,) in rows if deserialize_message(bytes(b), Twist).linear.x != 0.0)
print(f'{len(rows)} {nz}')
c.close()
PY
}

echo "=== ① 보완 없음 — 08-14 지도 세션과 같은 순서 ==="
run_case bug
read -r TOT_BUG NZ_BUG < "$OUT/bug.count"
echo "   총 ${TOT_BUG}건 · VOLATILE 발행자분(비영) = ${NZ_BUG}건"

echo "=== ② --qos-profile-overrides-path 로 구독을 VOLATILE 로 고정 ==="
run_case fix --qos-profile-overrides-path "$OUT/override.yaml"
read -r TOT_FIX NZ_FIX < "$OUT/fix.count"
echo "   총 ${TOT_FIX}건 · VOLATILE 발행자분(비영) = ${NZ_FIX}건"

echo
RC=0
# 🔴 결함이 재현되지 않으면 결론을 못 쓴다 — "고쳐졌다" 가 아니라 "판정 불능" 이다.
if [ "$NZ_BUG" -ne 0 ]; then
  echo "❌ ① 이 0건이 아니다 (${NZ_BUG}) — 결함이 재현되지 않았다. 판정 불능."
  RC=1
else
  echo "✅ ① 결함 재현 — VOLATILE 발행자분이 통째로 사라졌다."
fi
# 60 = 6초 × 10Hz. 전송 지터를 감안해 하한만 본다.
if [ "$NZ_FIX" -lt 50 ]; then
  echo "❌ ② 회수분이 ${NZ_FIX}건 뿐이다 (기대 ≥50) — 보완이 성립하지 않는다."
  RC=1
else
  echo "✅ ② 보완 성립 — ${NZ_FIX}건 회수 (기대 60 = 6초 × 10Hz)."
fi
exit $RC
