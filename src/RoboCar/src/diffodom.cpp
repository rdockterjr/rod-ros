#include "ros/ros.h"
#include "ros/console.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "iostream"
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

#include <iostream>

///////////////ENCODER STUFF ////////////////////////////////
#define ARRSIZE 3
#define LEFT  0
#define RIGHT 1
#define DTIME 2


//Robot constraints 
double WHEEL_BASE       = 0.1; //meters
double WHEEL_RADIUS     = 0.01; //meters

//timing stuff
ros::Time current_time, last_time;

//broadcaster stuff
ros::Publisher odom_pub;
tf::TransformBroadcaster *odom_broadcaster;
ros::Publisher pub_velocity_command;

int msg_count=0;
int avg_msgs=100;

struct robot_motion_struct {
	float vx;
	float vy;
	float vth;
	float delta_x;
	float delta_y;
	float delta_th;
	float x;
	float y;
	float th;
} ;

//struct for position
robot_motion_struct diff_motions;

//initialize the struct
void initialize_motion(robot_motion_struct &motion)
{
	motion.vx = 0.0;
	motion.vy = 0.0;
	motion.vth= 0.0;
	motion.delta_x = 0.0;
	motion.delta_y = 0.0;
	motion.delta_th= 0.0;
	motion.x= 0.0;
	motion.y= 0.0;
	motion.th= 0.0;
}


//differential drive equaitons
void compute_translations(float u_r, float u_l, double dt, robot_motion_struct &motion)
{

	std::cout << "ul " << u_l << " ur " << u_r << "\n";

	/// Multiply by radius to go from radians/s to m/s
	u_l*=WHEEL_RADIUS;
	u_r*=WHEEL_RADIUS;


	/// Calculate linear and angular velocity in robot frame
	motion.vx = 0.5*(u_l + u_r);
	motion.vy = 0;
	motion.vth= (u_r - u_l)/WHEEL_BASE;

	/// Propagate the robot using basic odom
	motion.x += 0.5*(u_l + u_r)*cos(motion.th) * dt;
	motion.y += 0.5*(u_l + u_r)*sin(motion.th) * dt;
	motion.th += (u_r - u_l)/WHEEL_BASE * dt;
	
}



void send_command(float v_left,float v_right, float time)
{

    //struct for commands
    std_msgs::Float32MultiArray command_velocity_val;
    command_velocity_val.data.resize(ARRSIZE);

    //motor commands to send
    command_velocity_val.data[LEFT]  = v_left;
    command_velocity_val.data[RIGHT] = v_right;
		command_velocity_val.data[DTIME] = time;

    pub_velocity_command.publish(command_velocity_val);
}

//callbacks for getting velocity data
void sub_actual(const std_msgs::Float32MultiArray& msg)
{
		//time stamp
		current_time = ros::Time::now();

		//compute the odoms
		compute_translations(msg.data[RIGHT], msg.data[LEFT], msg.data[DTIME], diff_motions);

		//Now setup all the odom nodes
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(diff_motions.th);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = diff_motions.x;
		odom_trans.transform.translation.y = diff_motions.y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster->sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";

		//set the position
		odom.pose.pose.position.x = diff_motions.x;
		odom.pose.pose.position.y = diff_motions.y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.twist.twist.linear.x = diff_motions.vx;
		odom.twist.twist.linear.y = diff_motions.vy;
		odom.twist.twist.angular.z = diff_motions.vth;

		//publish the odom message
		odom_pub.publish(odom);

		//timer update
    last_time = current_time;
    msg_count++;

	
		send_command(0.7, 0.7, 0.0);
}

int main(int argc, char **argv)
{
	//intialize ROS
	ros::init(argc, argv, "diffodom");

	//Setup all the nodes
	ros::NodeHandle n;
	ros::Subscriber sub_velocity_actual = n.subscribe("velocity_actual", 1000, sub_actual);
	pub_velocity_command = n.advertise<std_msgs::Float32MultiArray>("velocity_command", 1);

	//odometry
	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
	odom_broadcaster = new tf::TransformBroadcaster();	

	//setup the struct to contain ronny info
	initialize_motion(diff_motions);

	//timing stuff
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	send_command(0.7, 0.7, 0.0);


  ros::spin();


	return 0;
}
