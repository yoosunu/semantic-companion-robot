cmake_minimum_required(VERSION 3.8)
project(scr_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(simple_mover src/simple_mover.cpp)
target_compile_features(simple_mover PUBLIC cxx_std_17)
ament_target_dependencies(simple_mover rclcpp geometry_msgs nav_msgs std_msgs angles tf2 tf2_geometry_msgs)

install(TARGETS simple_mover
  DESTINATION lib/${PROJECT_NAME})

ament_package()
