#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

//nodehandler
ros::NodeHandle *n;

//callback function for subscriber
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	std::string who = msg->data;
  ROS_INFO("I heard: [%s]", who.c_str());
}

int main(int argc, char **argv)
{
	//setup to node
  ros::init(argc, argv, "listener");

  //nodehandle
  n = new ros::NodeHandle();

  //setup subscriber
  ros::Subscriber sub = n->subscribe("chatter", 1000, chatterCallback);

	//lets all background process go
  ros::spin();

  return 0;
}
