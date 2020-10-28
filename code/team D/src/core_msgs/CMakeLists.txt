cmake_minimum_required(VERSION 2.8.3)
project(core_msgs)
find_package(catkin REQUIRED
sensor_msgs
std_msgs	message_generation	geometry_msgs	
)

add_message_files( DIRECTORY msg
  FILES
  ball_position.msg
ball_bottom.msg

multiarray.msg
markermsg.msg
mark.msg
goal_sw.msg

)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs sensor_msgs)
catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs
std_msgs	geometry_msgs	)
