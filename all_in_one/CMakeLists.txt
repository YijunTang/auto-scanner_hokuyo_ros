cmake_minimum_required(VERSION 2.8.3)
project(all_in_one)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  message_generation
)

add_message_files(
  FILES
  FlagedScan.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs pcl_conversions pcl_ros message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(preprocess_node src/preprocess.cpp)
add_executable(process_node src/process.cpp)
add_executable(extract_plane_node src/extract_plane.cpp)

target_link_libraries(preprocess_node
  ${catkin_LIBRARIES}
)
target_link_libraries(process_node
  ${catkin_LIBRARIES}
)
target_link_libraries(extract_plane_node
  ${catkin_LIBRARIES}
)
