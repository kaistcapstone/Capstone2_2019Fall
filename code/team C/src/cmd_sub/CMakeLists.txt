cmake_minimum_required(VERSION 2.8.3)
project(cmd_sub)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  core_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  #include
)


catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp
#  sensor_msgs
#  std_msgs
#  core_msgs
#  DEPENDS system_lib
)

add_executable(cmd_sub_node src/cmd_sub.cpp)
add_dependencies(cmd_sub_node core_msgs_generate_messages_cpp)
target_link_libraries(cmd_sub_node
   ${catkin_LIBRARIES}
)

add_executable(ball_count_node src/ball_count.cpp)
add_dependencies(ball_count_node core_msgs_generate_messages_cpp)

target_link_libraries(ball_count_node
   ${catkin_LIBRARIES}
)
