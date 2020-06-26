#include "ros/ros.h"
#include "ros/console.h"

#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>

#include <iostream>

///////////////ENCODER STUFF ////////////////////////////////
#define ARRSIZE 3
#define LEFT  0
#define RIGHT 1
#define DTIME 2
#define BLADE 2 //out

double angular_vel_left, angular_vel_right;

//timing stuff
ros::Time current_time, last_time, cmd_time, last_cmd_time;
double output_frequency = 30.0;
double cmdvel_timeout = 4.0;

//broadcaster stuff
ros::NodeHandle *nh;
ros::Publisher joint_pub;
ros::Publisher pub_velocity_command;

int msg_count=0;
int blade_enable = 0;


//for sending command array down to the arduino
void send_command(float v_left,float v_right)
{
    //v_left and v_right should be angular velocities (rad/s)

    //struct for commands
    std_msgs::Float32MultiArray command_velocity_val;
    command_velocity_val.data.resize(ARRSIZE);

    //motor commands to send
    command_velocity_val.data[LEFT]  = v_left;
    command_velocity_val.data[RIGHT] = v_right;
    command_velocity_val.data[BLADE] = blade_enable;

    //publish to arduino
    pub_velocity_command.publish(command_velocity_val);
}


void sub_blades(const std_msgs::Int16::ConstPtr& msg){
	blade_enable = msg->data; //0-100
}

//subscribe to ros float command
void sub_cmd_left(const std_msgs::Float64::ConstPtr& msg){
    //time stamps and dt
    last_cmd_time = ros::Time::now();

    //now compute what our angular velocities should be to accomplish a given linear velocity from wheel diameter
    angular_vel_left = msg->data;
}

//subscribe to ros float command
void sub_cmd_right(const std_msgs::Float64::ConstPtr& msg){
    //time stamps and dt
    last_cmd_time = ros::Time::now();

    //now compute what our angular velocities should be to accomplish a given linear velocity from wheel diameter
    angular_vel_right = msg->data;
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


//callbacks for getting velocity data
void sub_feedback(const std_msgs::Float32MultiArray& msg)
{
    //time stamp
    current_time = ros::Time::now();

		//timestep
		double dt_s = msg.data[DTIME]/1000.0; //comes in millisends

    //allocate joint states
    sensor_msgs::JointState joint_output;
    joint_output.name.resize(2);
    joint_output.position.resize(2);
    joint_output.velocity.resize(2);
    joint_output.effort.resize(2);

    //fill it in
    joint_output.name[0] = "left_wheel_joint";
    joint_output.name[1] = "right_wheel_joint";
    joint_output.velocity[0] = msg.data[LEFT];
    joint_output.velocity[1] = msg.data[RIGHT];

    joint_pub.publish(joint_output);

    //timer update
    last_time = current_time;
    msg_count++;

}

int main(int argc, char **argv)
{
    //intialize ROS
    ros::init(argc, argv, "maurice_serial_comm");

    nh = new ros::NodeHandle("~");

    std::string joint_topic = "/maurice/joint_states";
    std::string left_wheel_topic = "/maurice/left_wheel_velocity_controller/command";
    std::string right_wheel_topic = "/maurice/right_wheel_velocity_controller/command";


    //get parameters
    nh->getParam("joint_topic",joint_topic);
    nh->getParam("left_wheel_topic",left_wheel_topic);
  	nh->getParam("right_wheel_topic",right_wheel_topic);
		nh->getParam("/cmdvel_timeout", cmdvel_timeout); //seconds
		nh->getParam("output_frequency",output_frequency);

    //Setup all the nodes
    ros::Subscriber sub_command_left = nh->subscribe(left_wheel_topic, 1, sub_cmd_left);
    ros::Subscriber sub_command_right = nh->subscribe(right_wheel_topic, 1, sub_cmd_right);
    ros::Subscriber sub_velocity_actual = nh->subscribe("/velocity_feedback", 10, sub_feedback);
		ros::Subscriber sub_blade_enable = nh->subscribe("/blade_enable", 10, sub_blades);

		//pub to arduino
    pub_velocity_command = nh->advertise<std_msgs::Float32MultiArray>("/velocity_command", 1);

    ros::Timer timeroutput = nh->createTimer(ros::Duration(1.0 / output_frequency), timercallback);

    //odometry
    joint_pub = nh->advertise<sensor_msgs::JointState>(joint_topic, 50);


    //timing stuff
    last_cmd_time = ros::Time::now();
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    msg_count = 0;

    //initialize commands
    send_command(0.0, 0.0);

    //let ros do its thing
    ros::spin();


    return 0;
}
