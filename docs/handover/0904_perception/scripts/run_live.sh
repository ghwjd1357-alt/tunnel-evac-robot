#!/bin/bash
# 실시간 확인용: 카메라 + pn_FINAL + MJPEG 스트리머
#   기동: bash /tmp/run_live.sh
#   종료: bash /tmp/run_live.sh stop
# 원본 파일 무수정 — /tmp 안에서만 동작. p.sh 와 동시에 켜지 말 것(카메라 충돌).
set +u
LOG=/tmp/livelog
mkdir -p "$LOG"

stop_all() {
    for f in cam.pid pn.pid mjpeg.pid; do
        [ -f "$LOG/$f" ] || continue
        P=$(cat "$LOG/$f")
        kill -9 "$P" 2>/dev/null && echo "  killed $f ($P)"
        rm -f "$LOG/$f"
    done
    # 카메라 컨테이너는 launch 가 자식으로 띄우므로 따로 정리
    C=$(ps -eo pid,args | grep "component_container.*camera" | grep -v grep | awk '{print $1}')
    for p in $C; do kill -9 "$p" 2>/dev/null; done
    sleep 2
    echo "=== 종료 완료 (시리얼/USB 반납 2초 포함) ==="
}

if [ "$1" = "stop" ]; then
    stop_all
    exit 0
fi

stop_all >/dev/null 2>&1

source /opt/ros/humble/setup.bash
source ~/percep_ws/install/setup.bash
source ~/yolo_env/bin/activate
export LD_LIBRARY_PATH=/usr/local/cuda/targets/aarch64-linux/lib:/home/hanhan/cudss_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

echo "[1/3] 카메라 기동..."
nohup ros2 launch orbbec_camera gemini2.launch.py enable_point_cloud:=false \
    color_width:=640 color_height:=480 color_fps:=30 > "$LOG/cam.log" 2>&1 &
echo $! > "$LOG/cam.pid"
for i in $(seq 1 40); do
    ros2 topic list 2>/dev/null | grep -q "/camera/color/image_raw" && { echo "  카메라 준비됨 (${i}초)"; break; }
    sleep 1
done

echo "[2/3] perception (pn_FINAL) 기동... 모델 로딩 약 40초"
nohup python3 /tmp/perftest/pn_FINAL.py --ros-args \
    -p rotate_180:=false -p debug_pose:=false \
    -p pose_model_path:=/home/hanhan/yolo11n-pose_meta.engine > "$LOG/pn.log" 2>&1 &
echo $! > "$LOG/pn.pid"
for i in $(seq 1 120); do
    ros2 topic list 2>/dev/null | grep -q "/camera/debug_image" && { echo "  인지 준비됨 (${i}초)"; break; }
    sleep 1
done

echo "[3/3] MJPEG 스트리머 기동..."
nohup python3 /tmp/mjpeg_server.py > "$LOG/mjpeg.log" 2>&1 &
echo $! > "$LOG/mjpeg.pid"
sleep 2

IP=$(hostname -I | awk '{print $1}')
echo ""
echo "==================================================="
echo "  브라우저에서 열기:  http://$IP:8081"
echo "==================================================="
echo "  종료:  bash /tmp/run_live.sh stop"
