cmake_minimum_required(VERSION 3.0.2)
project(pose_estimation_6d)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
catkin_python_setup()

# Service files
add_service_files(
  FILES
  UpdateKalmanFilterAndPredict.srv
  InitializeState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp rospy std_msgs geometry_msgs
  DEPENDS Eigen3
)

# Python files
catkin_install_python(PROGRAMS scripts/kalman_filter.py scripts/ros_kalman_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# C++ files
include_directories(
  include
  include/pose_estimation_6d/core_pose_estimation
  include/pose_estimation_6d/ros_pose_estimation
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(pose_estimation_6d ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(
  pose_estimation_6d
  src/core_pose_estimation/alignment.cpp
  src/core_pose_estimation/csv_parser.cpp
  src/core_pose_estimation/descriptor.cpp
  src/core_pose_estimation/pose_estimation.cpp
  src/core_pose_estimation/segmentation.cpp
  src/core_pose_estimation/utils.cpp

  src/ros_pose_estimation/ros_pose_estimation.cpp
  src/ros_pose_estimation/main.cpp
)

target_link_libraries(pose_estimation_6d ${catkin_LIBRARIES} ${PCL_LIBRARIES} pcl_visualization)
add_dependencies(pose_estimation_6d pose_estimation_6d_generate_messages_cpp)
