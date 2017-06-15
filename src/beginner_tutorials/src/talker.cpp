#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

//nodehandle
ros::NodeHandle *n;

int main(int argc, char **argv)
{
	//intialize
  ros::init(argc, argv, "talker");

	//nodehandle
	n = new ros::NodeHandle();

	//publisher setup, type is string, will buffer 1000 msgs in queue before overwriting
  ros::Publisher chatter_pub = n->advertise<std_msgs::String>("chatter", 1000);
	
	//how much to delay in sleep
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
		//variable
    std_msgs::String msg;
	
		//build a string
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

		//out to screen
    ROS_INFO("%s", msg.data.c_str());

		//publish it
    chatter_pub.publish(msg);
	
		//give other ros stuff a chance to think (ie subscribers)
    ros::spinOnce();

		//chill till the next episode
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
