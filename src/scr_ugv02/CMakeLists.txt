cmake_minimum_required(VERSION 3.8)
project(scr_ugv02)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlohmann_json 3 REQUIRED)   # JSON parsing (apt: nlohmann-json3-dev)

add_executable(ugv02_bridge src/ugv02_bridge.cpp)
target_include_directories(ugv02_bridge PRIVATE include)
target_compile_features(ugv02_bridge PUBLIC cxx_std_17)
ament_target_dependencies(ugv02_bridge rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros)
target_link_libraries(ugv02_bridge nlohmann_json::nlohmann_json)

install(TARGETS ugv02_bridge
  DESTINATION lib/${PROJECT_NAME})

ament_package()
