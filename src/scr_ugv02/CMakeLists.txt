cmake_minimum_required(VERSION 3.8)
project(scr_ugv02)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ugv02_bridge src/ugv02_bridge.cpp)
target_include_directories(ugv02_bridge PRIVATE include)
target_compile_features(ugv02_bridge PUBLIC cxx_std_17)
ament_target_dependencies(ugv02_bridge rclcpp geometry_msgs)

install(TARGETS ugv02_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
