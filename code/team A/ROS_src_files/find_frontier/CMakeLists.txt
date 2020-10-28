cmake_minimum_required(VERSION 2.8.3)
project(find_frontier)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  nav_msgs
  std_msgs
  move_base_msgs
  actionlib
  geometry_msgs
)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(find_frontier_node src/find_frontier.cpp)

target_link_libraries(find_frontier_node
  ${catkin_LIBRARIES}
)
