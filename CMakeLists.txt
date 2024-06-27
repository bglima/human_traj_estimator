cmake_minimum_required(VERSION 3.0.2)
project(human_traj_estimator)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rosdyn_core
  geometry_msgs
  tf
  tf2
  tf2_ros
  rotations_helper
  tf_conversions
  eigen_conversions
  franka_msgs
#   pbo_service
  )

catkin_package(
  INCLUDE_DIRS 
  include
  CATKIN_DEPENDS
  rosdyn_core
)
include_directories(
include
${catkin_INCLUDE_DIRS}
)


add_library(
human_traj_estimation src/human_traj_estimation.cpp
)

add_executable(human_traj_estimation_node src/human_traj_estimation_node.cpp)
# target_link_libraries( human_traj_estimation_node human_traj_estimation ${catkin_LIBRARIES} fuzzylite)
target_link_libraries( human_traj_estimation_node human_traj_estimation ${catkin_LIBRARIES} )
add_dependencies(human_traj_estimation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




