# 한 프레임의 탐지 결과 전량 — /detections 가 실어 나르는 타입.
#
# 계약 정본 = docs/PROJECT_CONTEXT.md §4.1.
# QoS = RELIABLE / VOLATILE / KEEP_LAST 5 · 10Hz · 0.5초 stale.
#
# 🔴 부분 실패 금지 — 인지 모델이 2개(Fire-Smoke + pose)여도 한 프레임의 결과는 하나다.
#    한쪽만 성공한 프레임은 발행하지 않고 /diagnostics ERROR 로 남긴다.
#    빈 배열(= 정상 미탐지)과 미발행(= 실패)을 섞지 않는다.

# stamp = color 촬영시각 · frame_id = camera optical frame
std_msgs/Header header

# 탐지 0건이면 빈 배열이다. 이것은 "정상 미탐지"이지 실패가 아니다.
Detection3D[] detections
