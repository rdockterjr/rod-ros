#include "ros/ros.h"
#include "ros/console.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <iostream>


//Robot constraints
double g_WheelBase       = 0.49; //meters
double g_WheelRadius     = 0.125; //meters
std::string g_CmdVelTopic = "/cmd_vel";
std::string g_LeftWheelTopic = "/maurice/left_wheel_velocity_controller/command";
std::string g_RightWheelTopic = "/maurice/right_wheel_velocity_controller/command";

//timing stuff
ros::Time cmd_time, last_cmd_time;

//broadcaster stuff
ros::NodeHandle *nh;
ros::Publisher pub_command_left, pub_command_right;

unsigned long msg_count=0;

//for sending commands to gazebo
void send_command(float v_left,float v_right)
{
	//v_left and v_right should be angular velocities (rad/s)

  //struct for commands
  std_msgs::Float64 command_val_left, command_val_right;
  command_val_left.data = v_left;
	command_val_right.data = v_right;

	//publish to gazebo
  pub_command_left.publish(command_val_left);
	pub_command_right.publish(command_val_right);
}


//subscribe to ros standard cmd_vel
void sub_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg){

	//time stamps and dt
	cmd_time = ros::Time::now();
	ros::Duration durat = cmd_time - last_cmd_time;
	double dt = durat.toSec();
	msg_count++;

	//get out the propel and velocity commands
	double propel = msg->linear.x;
	double steer = msg->angular.z;

	//now convert these to wheel velocities
	//math time: need to convert propel and spin into wheel velocities
  //positive angular velocity meas robot spins counter clockwise, right wheel should be faster
  double linear_vel_left = propel - ( (steer * g_WheelBase) / 2.0 ); //m/s
  double linear_vel_right = propel + ( (steer * g_WheelBase) / 2.0 ); //m/s

	//now compute what our angular velocities should be to accomplish a given linear velocity from wheel diameter
	double angular_vel_left = linear_vel_left / (g_WheelRadius); //rad/s
  double angular_vel_right= linear_vel_right / (g_WheelRadius); //rad/s

	//publish out to arduino
	send_command(angular_vel_left, angular_vel_right);

	//update last time
	last_cmd_time = cmd_time;
}

int main(int argc, char **argv)
{
	//intialize ROS
	ros::init(argc, argv, "diffdrive_cmdvel");

  nh = new ros::NodeHandle("~");

	//get parameters
	nh->getParam("WHEEL_BASE", g_WheelBase);
	nh->getParam("WHEEL_RADIUS",g_WheelRadius);
	nh->getParam("CMD_VEL_TOPIC",g_CmdVelTopic);
	nh->getParam("LEFT_WHEEL_TOPIC",g_LeftWheelTopic);
	nh->getParam("RIGHT_WHEEL_TOPIC",g_RightWheelTopic);

	//Setup all the nodes
	ros::Subscriber sub_command = nh->subscribe(g_CmdVelTopic, 30, sub_cmd_vel);
  pub_command_left = nh->advertise<std_msgs::Float64>(g_LeftWheelTopic, 1);
	pub_command_right = nh->advertise<std_msgs::Float64>(g_RightWheelTopic, 1);

	//timing stuff
	cmd_time = ros::Time::now();
	last_cmd_time = ros::Time::now();

	//initialize commands
	send_command(0.0, 0.0);

	//let ros do its thing
  ros::spin();


	return 0;
}
