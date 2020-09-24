# rod-ros
rods catkin workspace

##Install
mkdir ~/catkin_ws
cd ~/catkin_ws
git clone https://github.com/rdockterjr/rod-ros.git .

#requires arduino/ros lib
sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial
cd src/robo_mower/arduino/libraries
rosrun rosserial_arduino make_libraries.py .
