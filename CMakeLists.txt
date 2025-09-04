cmake_minimum_required(VERSION 3.8)
project(pointcloud_compressor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(draco REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${draco_INCLUDE_DIRS}
)

add_executable(pointcloud_compressor_node 
  src/pointcloud_compressor_node.cpp
  src/pointcloud_codec.cpp
)
ament_target_dependencies(pointcloud_compressor_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(pointcloud_compressor_node
  draco::draco
)

add_executable(pointcloud_decompressor_node
  src/pointcloud_decompressor_node.cpp
  src/pointcloud_codec.cpp
)
ament_target_dependencies(pointcloud_decompressor_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(pointcloud_decompressor_node
  draco::draco
)

install(TARGETS
  pointcloud_compressor_node
  pointcloud_decompressor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
