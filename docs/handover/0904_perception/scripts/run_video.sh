#!/bin/bash
# 시연 영상용: person_bag_0904 를 pn_FINAL 로 재생하며 debug_image 를 jpg 로 덤프
# 원본 파일 무수정 — /tmp 안에서만 동작
set +u
LOG=/tmp/vidrun
mkdir -p "$LOG"
rm -rf /tmp/vidframes
mkdir -p /tmp/vidframes

source /opt/ros/humble/setup.bash
source ~/percep_ws/install/setup.bash
source ~/yolo_env/bin/activate

# .bashrc 는 비대화형 ssh 에서 안 읽힘 → torch 가 libcudss.so.0 을 못 찾는다
export LD_LIBRARY_PATH=/usr/local/cuda/targets/aarch64-linux/lib:/home/hanhan/cudss_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

echo "[1/4] perception (pn_FINAL, rotate_180=false) 기동..."
python3 /tmp/perftest/pn_FINAL.py --ros-args -p rotate_180:=false -p debug_pose:=false -p pose_model_path:=/home/hanhan/yolo11n-pose_meta.engine > "$LOG/pn.log" 2>&1 &
PN=$!

READY=0
for i in $(seq 1 120); do
    if ros2 topic list 2>/dev/null | grep -q "/camera/debug_image"; then
        echo "  준비됨 (${i}초)"
        READY=1
        break
    fi
    sleep 1
done
if [ "$READY" = "0" ]; then
    echo "!! perception 기동 실패 — $LOG/pn.log 확인"
    tail -20 "$LOG/pn.log"
    kill -9 $PN 2>/dev/null
    exit 1
fi
sleep 3

echo "[2/4] 프레임 덤퍼 기동..."
python3 /tmp/dump_frames.py > "$LOG/dump.log" 2>&1 &
DP=$!
sleep 2

echo "[3/4] bag 재생 (실시간 속도, 약 64초)..."
ros2 bag play ~/person_bag_0904 > "$LOG/bag.log" 2>&1
sleep 4

echo "[4/4] 정리..."
kill -INT $DP 2>/dev/null
sleep 3
kill -9 $DP 2>/dev/null
kill -9 $PN 2>/dev/null
sleep 1

N=$(ls /tmp/vidframes 2>/dev/null | wc -l)
echo "덤프된 프레임: $N"
tar -C /tmp -czf /tmp/vidframes.tgz vidframes
ls -lh /tmp/vidframes.tgz
