cmake_minimum_required(VERSION 2.8.3)
project(maurice_control)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  sensor_msgs
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(diffdrive_cmdvel src/diffdrive_cmdvel.cpp)
target_link_libraries(diffdrive_cmdvel ${catkin_LIBRARIES})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
