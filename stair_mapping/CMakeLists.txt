cmake_minimum_required(VERSION 3.5)
project(stair_mapping)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_executable(stair_mapping_node src/stair_mapping_node.cpp)
target_include_directories(stair_mapping_node PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(stair_mapping_node ${PCL_LIBRARIES})
ament_target_dependencies(stair_mapping_node rclcpp sensor_msgs pcl_conversions)

add_executable(map_cleaner_node src/map_cleaner_node.cpp)
target_include_directories(map_cleaner_node PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(map_cleaner_node ${PCL_LIBRARIES})
ament_target_dependencies(map_cleaner_node rclcpp sensor_msgs pcl_conversions)

install(TARGETS 
  stair_mapping_node 
  map_cleaner_node 
  DESTINATION lib/${PROJECT_NAME}
)

# 파라미터 변경을 위한 설치 규칙
install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

ament_package()
