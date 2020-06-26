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

//timing stuff
ros::Time cmd_time, last_cmd_time;
float cmdvel_timeout = 10.0;

//broadcaster stuff
ros::NodeHandle *nh;
ros::Publisher pub_command_left, pub_command_right;

//limits
double min_lin_vel = -1.0; //meters/second
double max_lin_vel = 1.0; //meters/second
double min_rot_vel = -0.3; //-0.1 radians / sec
double max_rot_vel = 0.3; //0.1 radians / sec
double angular_vel_left = 0.0;
double angular_vel_right = 0.0;

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

  //limits
  if(propel > max_lin_vel){
    propel = max_lin_vel;
  }
  if(propel < min_lin_vel){
    propel = min_lin_vel;
  }
  if(steer > max_rot_vel){
    steer = max_rot_vel;
  }
  if(steer < min_rot_vel){
    steer = min_rot_vel;
  }

	//now convert these to wheel velocities
	//math time: need to convert propel and spin into wheel velocities
  //positive angular velocity meas robot spins counter clockwise, right wheel should be faster
  double linear_vel_left = propel - ( (steer * g_WheelBase) / 2.0 ); //m/s
  double linear_vel_right = propel + ( (steer * g_WheelBase) / 2.0 ); //m/s

	//now compute what our angular velocities should be to accomplish a given linear velocity from wheel diameter
	angular_vel_left = linear_vel_left / (g_WheelRadius); //rad/s
  angular_vel_right= linear_vel_right / (g_WheelRadius); //rad/s

	//update last time
	last_cmd_time = cmd_time;
}

//for cmd vel output
void timercallback(const ros::TimerEvent&)
{
	ros::Duration durat = ros::Time::now() - last_cmd_time;
  double time_since = durat.toSec();
	if(time_since > cmdvel_timeout){
		send_command(0.0, 0.0);
	}
	else{
		//velocity commands
    send_command(angular_vel_left, angular_vel_right);
	}
}


int main(int argc, char **argv)
{
	//intialize ROS
	ros::init(argc, argv, "diffdrive_cmdvel");

  nh = new ros::NodeHandle("~");

  float output_frequency = 30.0;

  std::string cmd_vel_topic = "/cmd_vel";
  std::string left_wheel_topic = "/maurice/left_wheel_velocity_controller/command";
  std::string right_wheel_topic = "/maurice/right_wheel_velocity_controller/command";


	//get parameters
	nh->getParam("/wheel_base", g_WheelBase);
	nh->getParam("/wheel_radius",g_WheelRadius);
  nh->getParam("/max_rot_vel", max_rot_vel);
	nh->getParam("/min_rot_vel",min_rot_vel);
  nh->getParam("/max_lin_vel", max_lin_vel);
	nh->getParam("/min_lin_vel",min_lin_vel);

	nh->getParam("cmd_vel_topic",cmd_vel_topic);
	nh->getParam("left_wheel_topic",left_wheel_topic);
	nh->getParam("right_wheel_topic",right_wheel_topic);
  nh->getParam("output_frequency",output_frequency);
  nh->getParam("/cmdvel_timeout", cmdvel_timeout); //seconds

	//Setup all the nodes
	ros::Subscriber sub_command = nh->subscribe(cmd_vel_topic, 30, sub_cmd_vel);
  pub_command_left = nh->advertise<std_msgs::Float64>(left_wheel_topic, 1);
	pub_command_right = nh->advertise<std_msgs::Float64>(right_wheel_topic, 1);

  ros::Timer timeroutput = nh->createTimer(ros::Duration(1.0 / output_frequency), timercallback);

	//timing stuff
	cmd_time = ros::Time::now();
	last_cmd_time = ros::Time::now();

	//initialize commands
	send_command(0.0, 0.0);

	//let ros do its thing
  ros::spin();


	return 0;
}
