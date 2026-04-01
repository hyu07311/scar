cmake_minimum_required(VERSION 3.5)
project(scar_state_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scar_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(scar_state_manager
  src/scar_state_manager.cpp)

ament_target_dependencies(scar_state_manager
  rclcpp
  scar_interfaces
  geometry_msgs
  std_msgs)

install(TARGETS scar_state_manager
  DESTINATION lib/${PROJECT_NAME})

ament_package()
