cmake_minimum_required(VERSION 3.8)
project(encoder_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(encoder_node src/encoder_node.cpp)
target_compile_features(encoder_node PUBLIC cxx_std_17)
ament_target_dependencies(encoder_node rclcpp std_msgs)

install(TARGETS encoder_node DESTINATION lib/${PROJECT_NAME})

ament_package()
