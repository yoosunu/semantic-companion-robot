cmake_minimum_required(VERSION 3.8)
project(scr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# .msg 파일 → C++ 헤더 + Python 클래스 자동 생성 (빌드할 때)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Person.msg"
  "msg/People.msg"
  DEPENDENCIES std_msgs
)

ament_package()
