#!/bin/bash
#Packages in one command

DISTRO=melodic

#force reinstall if neccesary
sudo apt-get install ros-$DISTRO-rviz-imu-plugin

sudo apt-get install ros-$DISTRO-roscpp ros-$DISTRO-xmlrpcpp

sudo apt-get install  ros-$DISTRO-joy ros-$DISTRO-serial ros-$DISTRO-rosserial-python ros-$DISTRO-imu-filter-madgwick ros-$DISTRO-phidgets-imu ros-$DISTRO-phidgets-drivers ros-$DISTRO-phidgets-api

sudo apt-get install ros-$DISTRO-usb-cam ros-$DISTRO-cv-bridge ros-$DISTRO-costmap-2d ros-$DISTRO-image-proc ros-$DISTRO-image-view

sudo apt-get install ros-$DISTRO-ros-control ros-$DISTRO-joint-state-controller ros-$DISTRO-effort-controllers ros-$DISTRO-position-controllers
sudo apt-get install ros-$DISTRO-velocity-controllers ros-$DISTRO-ros-controllers ros-$DISTRO-gazebo-ros ros-$DISTRO-gazebo-ros-control sudo apt-get install ros-$DISTRO-hector-gazebo-plugins

sudo apt-get install ros-$DISTRO-robot-localization

sudo apt-get install libusb-1.0-0-dev #delete build and devel after installing this, then redo catkin_make

#phidgets imu setup
roscd phidgets_api
sudo chmod +x setup-udev.sh
sudo ./setup-udev.sh
