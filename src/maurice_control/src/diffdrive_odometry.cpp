#include "ros/ros.h"
#include "ros/console.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/JointState.h>

#include <iostream>
#include <string>
#include <algorithm>

//Robot constraints
double wheel_base       = 0.4875; //meters
double wheel_radius     = 0.12812; //meters (80.5 cm circumf)

//timing stuff
ros::Time current_time, last_time;

//Ros Topics
std::string joint_name_left = "left_wheel_joint";
std::string joint_name_right = "right_wheel_joint";

//broadcaster stuff
ros::NodeHandle *nh;
ros::Publisher odom_pub;
tf::TransformBroadcaster *odom_broadcaster;

int msg_count = 0;
bool odom_ekf = true;

struct motions {
	float v_x;
	float v_th;
	float delta_x;
	float delta_y;
	float delta_th;
	float x;
	float y;
	float th;
} ;

//struct for position
motions robot_motion;

//initialize the struct
void initialize_motion(motions &motion)
{
    motion.v_x = 0.0;
	motion.v_th = 0.0;
    motion.delta_x = 0.0;
    motion.delta_y = 0.0;
    motion.delta_th= 0.0;
    motion.x= 0.0;
    motion.y= 0.0;
    motion.th= 0.0;
}


//differential drive equaitons
void compute_translations(float w_r, float w_l, double dt, motions &motion)
{
    // Multiply by radius to go from radians/s to m/s
    w_r = w_r*wheel_radius;
    w_l = w_l*wheel_radius;

	//TAKEN FROM SIEGWART BOOK

	//compute instantaneous velocities (meters and radians)
    motion.v_x = (w_r + w_l) / 2.0; //delta_s
    motion.v_th = (w_r - w_l) / wheel_base; //radians

    //compute changes in x and y (for position)
	motion.delta_th = ((motion.v_th*dt) / 2.0);
	motion.th = motion.th + motion.delta_th;
    motion.delta_x = motion.v_x*cos(motion.th); //meters
    motion.delta_y = motion.v_x*sin(motion.th); //meters

    // Propagate the robot using basic odom
    motion.x = motion.x + motion.delta_x*dt;
    motion.y =  motion.y + motion.delta_y*dt;
    motion.th = motion.th + motion.delta_th;
}



//callbacks for getting velocity data
void sub_feedback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //time stamp
    current_time = msg->header.stamp;
	ros::Duration durat = current_time - last_time;
	double dt_s = durat.toSec();

	//extract joint states
	std::vector<std::string> joint_names = msg->name;
	float vel_left = msg->velocity[std::find(joint_names.begin(), joint_names.end(), joint_name_left) - joint_names.begin()];
	float vel_right = msg->velocity[std::find(joint_names.begin(), joint_names.end(), joint_name_right) - joint_names.begin()];

    //compute the odoms
    compute_translations(vel_right, vel_left, dt_s, robot_motion);

    //Now setup all the odom nodes
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_motion.th);


    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //set the position using current odom estimate
    odom.pose.pose.position.x = robot_motion.x;
    odom.pose.pose.position.y = robot_motion.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set up simple covariances for pose and twist
    //We need this for robot_pose_ekf
    //only diagonals (x,y,z) (roll,pitch,yaw)
    odom.pose.covariance[0]  = 0.5;
    odom.pose.covariance[7]  = 0.5;
    odom.pose.covariance[14] = 0.01; //we are really sure we are at Z=0
    odom.pose.covariance[21] = 0.1; //no knowledge of roll
    odom.pose.covariance[28] = 0.1; //no knowledge of pitch
    odom.pose.covariance[35] = 0.1;

    odom.twist.covariance[0]  = 0.05; //yes instant x velocity
    odom.twist.covariance[7]  = 0.05; // no instant Y velocity (we're sure)
    odom.twist.covariance[14] = 0.01; // no instant Z velocity (we're sure)
    odom.twist.covariance[21] = 0.01; //no knowledge of roll velocity
    odom.twist.covariance[28] = 0.01; //no knowledge of pitch velocity
    odom.twist.covariance[35] = 0.05; //we maybe know about our yaw velocity

    //set the velocity (m/s or rad/s) this is all in the child frame, no instantaneous Y velocity
    odom.twist.twist.linear.x = robot_motion.vs; //delta_x;
    odom.twist.twist.linear.y = 0.0; //delta_y
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.z = robot_motion.delta_th;

    //publish the /odom message
    odom_pub.publish(odom);

	if(!odom_ekf){
		//dont publish tf for odom when using robot_pose_ekf or robot_localization packages
		//http://answers.ros.org/question/10511/frames-and-tf-with-robot_pose_ekf/
		//http://wiki.ros.org/robot_pose_ekf/Tutorials/AddingGpsSensor

		//publish out the transform from odom to the car
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint"; //"base_footprint"; // "base_link";
		odom_trans.transform.translation.x = robot_motion.x;
		odom_trans.transform.translation.y = robot_motion.y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		//send the /odom to /base_footprint transform
		odom_broadcaster->sendTransform(odom_trans);
	}

    msg_count++;
	last_time = current_time;
}


int main(int argc, char **argv)
{
    //intialize ROS
    ros::init(argc, argv, "diffdrive_odometry");

    nh = new ros::NodeHandle("~");

	std::string joint_topic = "/maurice/joint_states";

    //get parameters
    nh->getParam("/wheel_base", wheel_base); //meters
    nh->getParam("/wheel_radius",wheel_radius); //meters
	nh->getParam("/odom_ekf", odom_ekf); // bool
	nh->getParam("joint_topic", joint_topic); // string
	nh->getParam("joint_name_left", joint_name_left); // string
	nh->getParam("joint_name_right", joint_name_right); // string

	//setup the struct to contain ronny info
	initialize_motion(robot_motion);
	last_time = ros::Time::now();

    //Setup all the nodes
	ros::Subscriber sub_velocity_actual = nh->subscribe(joint_topic, 10, sub_feedback);

    //odometry
    odom_pub = nh->advertise<nav_msgs::Odometry>("/odometry/wheel", 50);

	if(!odom_ekf){
		odom_broadcaster = new tf::TransformBroadcaster();
		//next we'll publish odom to the car
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint"; //"base_footprint"; // "base_link";

		odom_trans.transform.translation.x = 0.0;
		odom_trans.transform.translation.y = 0.0;
		odom_trans.transform.translation.z = 0.0;
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
		odom_trans.transform.rotation = odom_quat;

		odom_broadcaster->sendTransform(odom_trans);
	}


    //let ros do its thing
    ros::spin();


    return 0;
}
