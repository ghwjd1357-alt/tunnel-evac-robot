# 탐지 1건 — 역할 B(YOLO+depth) → 역할 A(Perception Adapter)
#
# 계약 정본 = docs/PROJECT_CONTEXT.md §4.1 (08-17 재합의 4왕복 확정).
# 🔴 필드 추가 = 타입 변경 = 양측 동시 리빌드다. 확장은 V2 별도 메시지로 한다.

# 닫힌 열거 5종 · 소문자 고정:
#   person_fallen · person_ok · person_unknown · fire · smoke
# 🔴 car·human 은 이 필드에 실려 오지 않는다 — 역할 B가 열거 검사 이전에 drop 하고
#    /diagnostics 카운터로만 남긴다. 검사에 태우면 터널의 정상적인 차·사람이
#    매번 위반 경보가 되고, 사람이 경보를 꺼서 검사 자체가 없는 것과 같아진다.
string class_name

# 0.0 ~ 1.0
float32 confidence

# 이미지 평면 bounding box (color 프레임 픽셀 좌표)
# 자세 판정(넘어짐)의 기준축은 🔴 이미지 y축이다 — 중력 보정은 V1 계약에서 제외.
# 근거·재개방 조건 = docs/PITFALLS.md §13.
sensor_msgs/RegionOfInterest bbox

# camera optical frame 기준 3D 위치 [m].
# 🔴 depth 결합까지가 역할 B 책임이다. 역할 A는 이 값을 검증해 map 으로 옮긴다.
geometry_msgs/Point position
