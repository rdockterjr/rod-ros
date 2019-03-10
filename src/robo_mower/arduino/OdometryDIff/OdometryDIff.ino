
#include <ArduinoHardware.h>
#include <Cytron.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>

//variables for ros arrays
#define ARRSIZE 3
#define LEFT 0
#define RIGHT 1
#define TIME 2

#define LOOP_TME 100
#define COUNTS_PER_REV 1024
#define GEAR_RATIO 17.75 //rough calibration


#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>


#ifndef M_PI
  #define M_PI 3.14159265358
#endif


//Pin Assignments
int pwmpin_left = 35;
int dirpin_left = 34;
int pwmpin_right = 38;
int dirpin_right = 39;
int enca_left = 11;
int encb_left = 12;
int enca_right = 24;
int encb_right = 25;

//variables for control
unsigned long last_cmd_time;
unsigned long loop_count;

//velocity and timing variables
long left_diff, right_diff, last_left, last_right, new_left, new_right;
float velocity_actual_left, velocity_actual_right;
float velocity_cmd_left, velocity_cmd_right;
float counts_2_rads;
unsigned long last_time, new_time, time_diff;

//errors for control loop
float motor_command_left, motor_command_right;
float error_prev_left, error_prev_right;
unsigned long last_pid_time, new_pid_time;
float dt_s;

//Specify the ratios and initial tuning parameters, different for each wheel
float Kp_R = 0.0; //error diff
float Ki_R = 10.0; //current error
float Kd_R = 0.0;
float Kp_L = 0.0; //error diff
float Ki_L = 10.0; //current error
float Kd_L = 0.0;



//encoder setup
Encoder LeftEnc(enca_left, encb_left);
Encoder RightEnc(enca_right, encb_right);

//cytron class
Cytron left_motor(pwmpin_left, dirpin_left, HIGH);
Cytron right_motor(pwmpin_right, dirpin_right, LOW);



/////////////////////Wheel Velocity Helper Funcitons //////


 //compute velocity for each wheel
void compute_velocities(){
  //encoder diff
  new_left = LeftEnc.read();
  new_right = RightEnc.read();
  left_diff = new_left - last_left;
  right_diff = new_right - last_right;
  //time diff
  new_time = millis();
  time_diff = new_time - last_time;
  //compute veloicities (counts)
  velocity_actual_left = -(float(left_diff)/float(time_diff))*counts_2_rads;
  velocity_actual_right = (float(right_diff)/float(time_diff))*counts_2_rads;
  //update
  last_left = new_left;
  last_right = new_right;
  last_time = new_time;
}


//PI control to set motor PWM to correct value to keep desired velocity
void compute_motor_commands()
{
  //get dt
  new_pid_time = millis();
  dt_s = float(new_pid_time - last_pid_time)/1000.0;
  last_pid_time = new_pid_time;
  
  //get error from command velocity to actual
  float error_now_left = velocity_cmd_left - velocity_actual_left ;
  float error_now_right = velocity_cmd_right - velocity_actual_right ;

  //velocity PID
  float changeL = error_now_left*Ki_L*dt_s + Kp_L*(error_now_left-error_prev_left);
  float changeR = error_now_right*Ki_R*dt_s + Kp_R*(error_now_right-error_prev_right);

  //Motor commandings in PWM units -255 to +255
  motor_command_left+=changeL;
  motor_command_right+=changeR;
  
  //Move the new value into the past
  error_prev_left=error_now_left;
  error_prev_right=error_now_right;
}



/////////////////////ROS STUFF/////////////////////////////

//Gets called whenever motor node from ros publishes something
void cmd_callback(const std_msgs::Float32MultiArray& msg){
  velocity_cmd_left = msg.data[LEFT]; 
  velocity_cmd_right = msg.data[RIGHT];
  
  //update the last time
  last_cmd_time=millis();
}


std_msgs::Float32MultiArray Velocity_Out;
//ros node stuff
ros::NodeHandle nh;
ros::Publisher pub_feedback("/velocity_feedback", &Velocity_Out);
ros::Subscriber<std_msgs::Float32MultiArray> sub_cmd("/velocity_command", &cmd_callback);

//////////////////////////////Setup//////////////////////////////////////////
void setup()
{
  
  //get starting encoder
  //initialize timing
  last_time = millis();
  last_pid_time = millis();
  last_left = 0;
  last_right = 0;
  counts_2_rads = (2.0*M_PI*1000.0)/(float(COUNTS_PER_REV)*GEAR_RATIO);

  //initialize variables
  velocity_actual_left = 0.0;
  velocity_actual_right = 0.0;
  velocity_cmd_left = 0.0;
  velocity_cmd_right = 0.0;
  motor_command_left = 0.0;
  motor_command_right = 0.0;
  loop_count = 0;

  // set up motor control
  left_motor.Init();
  right_motor.Init();
  

  //ROS array size
  Velocity_Out.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 1);
  Velocity_Out.layout.dim[0].label = "velocity_feedback";
  Velocity_Out.layout.dim[0].size = ARRSIZE;
  Velocity_Out.layout.dim[0].stride = 1*ARRSIZE;
  Velocity_Out.layout.data_offset = 0;
  Velocity_Out.data_length = ARRSIZE;
  Velocity_Out.data = (float *)malloc(sizeof(float)*ARRSIZE);
  

  //Initialize ROS stuff
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_feedback);
  nh.subscribe(sub_cmd);

 
  last_cmd_time=millis();
}


/////////////////////////////////////////Loop//////////////////////////////
void loop()
{

  //update velocities
  compute_velocities();
  
  /// Compute Outputs
  compute_motor_commands();

  // Send Motor commands
  left_motor.Control(motor_command_left);
  right_motor.Control(motor_command_right);

  //Send velocity to ROS Node (rad/s)
  Velocity_Out.data[RIGHT] = velocity_actual_right;
  Velocity_Out.data[LEFT ] = velocity_actual_left;
  Velocity_Out.data[TIME] = loop_count;
  pub_feedback.publish( &Velocity_Out );


  //delay
  delay(LOOP_TME);
  nh.spinOnce();
  

  //check if timeout should trigger
  unsigned long timeout = millis() - last_cmd_time;
  if(timeout>20000ul){
    /// Tell the motors to shut their faces
    motor_command_left = 0.0;
    motor_command_right = 0.0;
  }

  loop_count++;
}
