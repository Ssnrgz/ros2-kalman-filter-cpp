cmake_minimum_required(VERSION 3.8)
project(sensor_fusion_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(fusion_node src/fusion_node.cpp)
target_link_libraries(fusion_node Eigen3::Eigen)
ament_target_dependencies(fusion_node rclcpp sensor_msgs nav_msgs geometry_msgs)

install(TARGETS
  fusion_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
