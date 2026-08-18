cmake_minimum_required(VERSION 3.8)
project(tunnel_interfaces)

# 🔴 이 패키지는 `/detections` V1 계약만 소유한다. 노드·라이브러리를 추가하지 않는다.
#    계약 정본 = docs/PROJECT_CONTEXT.md §4.1 · 원문 = ~/Desktop/YOLO_탐지연동_합의사항.md §15

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Detection3D 가 참조하는 표준 패키지 — DEPENDENCIES 에 빠지면 생성 코드가 링크되지 않는다.
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection3D.msg"
  "msg/Detection3DArray.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
