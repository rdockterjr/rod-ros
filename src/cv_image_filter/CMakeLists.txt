cmake_minimum_required(VERSION 2.8.3)
project(cv_image_filter)

### ROD to change opencv versions to compiled
#set(CMAKE_PREFIX_PATH /usr/local)
#SET("OpenCV_DIR" "/usr/local")

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
	image_transport
	tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensors_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cv_image_filter
#  CATKIN_DEPENDS cv_bridge opencv2 roscpp sensors_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

# Depth Filter
add_executable(depthimage_filter src/depthimage_filter.cpp)
target_link_libraries(
  depthimage_filter ${catkin_LIBRARIES}
)

# masking
add_executable(depth_threshold src/depth_threshold.cpp)
target_link_libraries(
  depth_threshold ${catkin_LIBRARIES}
)

# Color IMage
add_executable(simple_capture src/simple_capture.cpp)
target_link_libraries(
  simple_capture ${catkin_LIBRARIES}
)






