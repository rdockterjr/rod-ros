/// ROS
#include <ros/ros.h>

/// Messages
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

//c++ junk
#include <string>
#include <iostream>


//publisher objects
ros::NodeHandle *nh;
ros::Publisher cmd_pub;
ros::Publisher blade_pub;

//cuz radians
#define PI 3.14159265358979323846
#define EPSILON 0.001

//topics
std::string joyname = "/joy";

//scaling
double propel_scale = 3.0;
double steer_scale = 1.0;

//most recent cmd
double propel_cmd = 0.0;
double steer_cmd = 0.0;
ros::Time last_input;
double timeoutcmd = 25.0; //zero after 3 seconds

//for debounce buttons
ros::Time blade_start, blade_end;
double debounce_time = 0.5;
bool bladeflipflop = false;

/// Callback when we get a new joint state
void joycallback(const sensor_msgs::Joy::ConstPtr& joy_msg){

  //get ros time and time change
  last_input = joy_msg->header.stamp;

  //get the inputs
  double steer_raw = joy_msg->axes[2];
  double propel_raw = joy_msg->axes[3];
  int blade_btn = joy_msg->buttons[7]; //start blades

  //scale the outputs and store as globals
  propel_cmd = propel_raw * propel_scale;
  steer_cmd = steer_raw * steer_scale;

  //if we should enable blades
  if(blade_btn == 1){
    blade_end = ros::Time::now();
    double press_duration = (blade_end - blade_start).toSec();
    if( press_duration < debounce_time){
      std::cout << "Ignoring Debounce " <<  press_duration << "\n";
    }
    else{
      //either start or stop the recording
      blade_start = ros::Time::now();
      bladeflipflop = !bladeflipflop;
      std_msgs::Int16 blademsg;
      blademsg.data = bladeflipflop;
      blade_pub.publish(blademsg);
    }
  }
}


//for cmd vel output at constant rate
void timercallback(const ros::TimerEvent&)
{
    ros::Duration timesince = ros::Time::now() - last_input;
    if(timesince.toSec() > timeoutcmd && timesince.toSec() < 1000.0){
        propel_cmd = 0.0;
        steer_cmd = 0.0;
    }
    //publish cmd vel
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = propel_cmd;
    cmd_vel_msg.angular.z = steer_cmd;
    cmd_pub.publish(cmd_vel_msg);
}


int main (int argc, char** argv)
{
  /// Initialize ROS
  ros::init (argc, argv, "joyop_cmdvel");
  nh = new ros::NodeHandle("~");

  //local scale vals
  nh->getParam("propel_scale",propel_scale);
  nh->getParam("steer_scale",steer_scale);
	

  //let people know we are publishing out odom and a tf
  cmd_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  //publish out the start/stop cutting blades
  blade_pub = nh->advertise<std_msgs::Int16>("/blade_enable",10);

  //debounce
  blade_start = ros::Time::now();

  //subscribe to all joint states
  nh->getParam("joy_topic",joyname);
  ros::Subscriber sub_joy = nh->subscribe(joyname,1,joycallback);

  //for issuing cmd vels
	double output_frequency = 15;
  nh->getParam("output_frequency",output_frequency);
  ros::Timer timeroutput = nh->createTimer(ros::Duration(1.0 / output_frequency), timercallback);


  // Spin
  ros::spin ();
}




