#include "ros/ros.h"
#include "ros/console.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
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
#define BLADE 2 //out

//Robot constraints 
double WHEEL_BASE       = 0.4875; //meters
double WHEEL_RADIUS     = 0.12812; //meters (80.5 cm circumf)

//set up some limits for the sake of our tractor
double min_lin_vel = -1.0; //meters/second
double max_lin_vel = 1.0; //meters/second
double min_rot_vel = -0.3; //-0.1 radians / sec
double max_rot_vel = 0.3; //0.1 radians / sec
double angular_vel_left, angular_vel_right;

//timing stuff
ros::Time current_time, last_time, cmd_time, last_cmd_time;
double output_frequency = 30.0;
double cmdvel_timeout = 4.0;

//broadcaster stuff
ros::NodeHandle *nh;
ros::Publisher odom_pub;
tf::TransformBroadcaster *odom_broadcaster;
ros::Publisher pub_velocity_command;

int msg_count=0;
int blade_enable = 0;
bool odom_ekf = true;

struct motions {
	float vs;
	float delta_x;
	float delta_y;
	float delta_th;
	float x;
	float y;
	float th;
} ;

//struct for position
motions diff_motions;

//initialize the struct
void initialize_motion(motions &motion)
{
    motion.vs = 0.0;
    motion.delta_x = 0.0;
    motion.delta_y = 0.0;
    motion.delta_th= 0.0;
    motion.x= 0.0;
    motion.y= 0.0;
    motion.th= 0.0;
}


//differential drive equaitons
void compute_translations(float u_r, float u_l, double dt, motions &motion)
{

    // Multiply by radius to go from radians/s to m/s
    u_r = u_r*WHEEL_RADIUS;
    u_l = u_l*WHEEL_RADIUS;

		//TAKEN FROM SIEGWART BOOK, PAGE 271
    //compute distance traveled by each wheel (meters)
    double delta_s_right = u_r * dt;
    double delta_s_left = u_l * dt;

		//compute instantaneous velocities (meters and radians)
    motion.vs = (delta_s_right + delta_s_left) / 2.0; //delta_s
    motion.delta_th = (delta_s_right - delta_s_left) / WHEEL_BASE; //radians

    //compute changes in x and y (for position)
    motion.delta_x = motion.vs*cos(motion.th + (motion.delta_th / 2.0) ); //meters
    motion.delta_y = motion.vs*sin(motion.th + (motion.delta_th / 2.0) ); //meters

    // Propagate the robot using basic odom
    motion.x = motion.x + motion.delta_x;
    motion.y =  motion.y + motion.delta_y;
    motion.th = motion.th + motion.delta_th;
}


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

//subscribe to ros standard cmd_vel
void sub_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg){

    //time stamps and dt
    cmd_time = ros::Time::now();

    //get the linear angular components
    geometry_msgs::Vector3 lin = msg->linear;
    geometry_msgs::Vector3 ang = msg->angular;

    //get propel and spin commands from the cmd vel
    double propel_cmd = lin.x; // in meters/second
    double spin_cmd = ang.z; //in radians/s

		//check our bounds on propel and spin
    if(propel_cmd > max_lin_vel){
        propel_cmd = max_lin_vel;
    }
    if(propel_cmd < min_lin_vel){
        propel_cmd = min_lin_vel;
    }
    if(spin_cmd > max_rot_vel){
        spin_cmd = max_rot_vel;
    }
    if(spin_cmd < min_rot_vel){
        spin_cmd = min_rot_vel;
    }

    //math time: need to convert propel and spin into wheel velocities
    //positive angular velocity meas robot spins counter clockwise, right wheel should be faster
    double linear_vel_left = propel_cmd - ( (spin_cmd * WHEEL_BASE) / 2.0 ); //m/s
    double linear_vel_right = propel_cmd + ( (spin_cmd * WHEEL_BASE) / 2.0 ); //m/s

    //now compute what our angular velocities should be to accomplish a given linear velocity from wheel diameter
    angular_vel_left = linear_vel_left / WHEEL_RADIUS; //rad/s
    angular_vel_right= linear_vel_right / WHEEL_RADIUS; //rad/s

   
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


//callbacks for getting velocity data
void sub_feedback(const std_msgs::Float32MultiArray& msg)
{
    //time stamp
    current_time = ros::Time::now();

		//timestep
		double dt_s = msg.data[DTIME]/1000.0; //comes in millisends

    //compute the odoms
    compute_translations(msg.data[RIGHT], msg.data[LEFT], dt_s, diff_motions);

    //Now setup all the odom nodes
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(diff_motions.th);

  
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //set the position using current odom estimate
    odom.pose.pose.position.x = diff_motions.x;
    odom.pose.pose.position.y = diff_motions.y;
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
    odom.twist.twist.linear.x = diff_motions.vs/dt_s; //delta_x;
    odom.twist.twist.linear.y = 0.0; //delta_y
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.z = diff_motions.delta_th/dt_s;

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
        odom_trans.transform.translation.x = diff_motions.x;
        odom_trans.transform.translation.y = diff_motions.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        //send the /odom to /base_footprint transform
        odom_broadcaster->sendTransform(odom_trans);
		}

    //timer update
    last_time = current_time;
    msg_count++;

}

int main(int argc, char **argv)
{
    //intialize ROS
    ros::init(argc, argv, "wheel_odometry");

    nh = new ros::NodeHandle("~");

    //get parameters
    nh->getParam("/wheel_base", WHEEL_BASE); //meters
    nh->getParam("/wheel_radius",WHEEL_RADIUS); //meters
    nh->getParam("/min_rot_vel", min_rot_vel); //rad/s
    nh->getParam("/max_rot_vel", max_rot_vel);//rad/s
    nh->getParam("/min_lin_vel", min_lin_vel); //m/s
    nh->getParam("/max_lin_vel", max_lin_vel); //m/s
		nh->getParam("/odom_ekf", odom_ekf); // bool
		nh->getParam("/cmdvel_timeout", cmdvel_timeout); //seconds
		nh->getParam("output_frequency",output_frequency);

    //Setup all the nodes
    ros::Subscriber sub_command = nh->subscribe("/cmd_vel", 1, sub_cmd_vel);
    ros::Subscriber sub_velocity_actual = nh->subscribe("/velocity_feedback", 10, sub_feedback);
		ros::Subscriber sub_blade_enable = nh->subscribe("/blade_enable", 10, sub_blades);

		//pub
    pub_velocity_command = nh->advertise<std_msgs::Float32MultiArray>("/velocity_command", 1);

    ros::Timer timeroutput = nh->createTimer(ros::Duration(1.0 / output_frequency), timercallback);

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

    //setup the struct to contain ronny info
    initialize_motion(diff_motions);

    //timing stuff
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    cmd_time = ros::Time::now();
    last_cmd_time = ros::Time::now();

    //initialize commands
    send_command(0.0, 0.0);

    //let ros do its thing
    ros::spin();


    return 0;
}
