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

//variables for directions
#define MOTORFORWARD 1
#define MOTORBACKWARD -1
#define MOTORSTOP 0
#define PWM_MAX 254
#define CHANGE_LIMIT 50

#include <Encoder.h>

#ifndef M_PI
  #define M_PI 3.14159265358
#endif

//PINS FOR L298N
// LEFT MOTOR A
int enA = 6;
int in1 = 7;
int in2 = 8;
// RIGHT MOTOR B
int enB = 11;
int in3 = 9;
int in4 = 10;



//variables for control
unsigned long last_received_time;
unsigned long last_sent_time;
const int spin_time_ms = 10;
float dirFlipL = 1.0; //because left wheel is backwards
float dirFlipR = 1.0; // right wheel goes correct direction

//velocity variables
float velocity_actual_left = 0.0;
float velocity_actual_right = 0.0;
long oldLeft  = 0;
long oldRight = 0;
long newLeft, newRight, diffLeft, diffRight;
float dt = 1.0; // 1hz
unsigned long last_timeL, last_timeR;
float MotorCommandL,MotorCommandR;
float Error_Prev_Left, Error_Prev_Right;
float Error_Now_Left, Error_Now_Right;
float Setpoint_ActualLeft,Setpoint_ActualRight;

//Specify the ratios and initial tuning parameters
float Kp = 1.0;
float Ki = 0.00005;
float Kd = 0.0;
int EncPerRev = 1120; //1920; //32 cpr x 35:1 gear ratio
float EncPerRad;
float RadPerEncUs;

//Encoders 
Encoder EncMotorL(2,4);
Encoder EncMotorR(3,5);


/////////////////////Wheel Velocity Helper Funcitons //////


//set target velocities
void set_target(float target_left,float target_right)
{
  Setpoint_ActualLeft = dirFlipL*target_left; 
  Setpoint_ActualRight = dirFlipR*target_right;
}


//set direction in L298 chip
void setDirection(int directionL, int directionR)
{
  //Left Motor
  if(directionL == MOTORFORWARD){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(directionL == MOTORBACKWARD){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if(directionL == MOTORSTOP){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  
  //Right Motor
  if(directionR == MOTORFORWARD){
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if(directionR == MOTORBACKWARD){
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if(directionR == MOTORSTOP){
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  
}

//change the enable pins to change direction
void setMotorPWM(int pwmL, int pwmR)
{
  int dirL = MOTORFORWARD;
  int dirR = MOTORFORWARD;
  int magL = abs(pwmL);
  int magR = abs(pwmR);

  //deal with directions
  if(pwmL < 0){
    dirL = MOTORBACKWARD;
  }
  else if(pwmL == 0){
    dirL = MOTORSTOP;
  }
  if(pwmR < 0){
    dirR = MOTORBACKWARD;
  }
  else if(pwmR == 0){
    dirR = MOTORSTOP;
  }
  setDirection(dirL, dirR);

  //Left Motor
  analogWrite(enA, magL);
  
  //Right Motor
  analogWrite(enB, magR);
}


//PI control to set motor PWM to correct value to keep desired velocity
void VelocityController()
{
  //Get new values of the wheel encoders
  //get current encoder count and delta time
  newLeft = EncMotorL.read();
  dt=micros()-last_timeL;
  last_timeL = micros();
  //get angular velocity
  diffLeft = newLeft - oldLeft;
  velocity_actual_left = (float(diffLeft) / dt) * RadPerEncUs;

  //get current encoder count and delta time
  newRight = EncMotorR.read();
  dt=micros()-last_timeR;
  last_timeR = micros();
  //get angular velocity
  diffRight = newRight - oldRight;
  velocity_actual_right= (float(diffRight) / dt) * RadPerEncUs;

  //get error from command velocity to actual
  Error_Now_Left = Setpoint_ActualLeft - velocity_actual_left ;
  Error_Now_Right = Setpoint_ActualRight - velocity_actual_right ;

  //velocity PID
  float changeL = Error_Now_Left*Ki*dt + Kp*(Error_Now_Left-Error_Prev_Left);
  float changeR = Error_Now_Right*Ki*dt + Kp*(Error_Now_Right-Error_Prev_Right);
  if(changeL>CHANGE_LIMIT){changeL=CHANGE_LIMIT;}
  if(changeL<-CHANGE_LIMIT){changeL=-CHANGE_LIMIT;}
  if(changeR>CHANGE_LIMIT){changeR=CHANGE_LIMIT;}
  if(changeR<-CHANGE_LIMIT){changeR=-CHANGE_LIMIT;}

  //Motor commandings in PWM units -255 to +255
  MotorCommandL+=changeL;
  MotorCommandR+=changeR;
  
  //Avoid integral windup, if you are failing to meet target 
  //even with max power.
  if (MotorCommandL > PWM_MAX) {MotorCommandL= PWM_MAX;}
  if (MotorCommandL <-PWM_MAX) {MotorCommandL=-PWM_MAX;}
  if (MotorCommandR > PWM_MAX) {MotorCommandR= PWM_MAX;}
  if (MotorCommandR <-PWM_MAX) {MotorCommandR=-PWM_MAX;}
  
  //Move the new value into the past
  oldLeft=newLeft;
  oldRight=newRight;
  Error_Prev_Left=Error_Now_Left;
  Error_Prev_Right=Error_Now_Right;

  //Send the command to the motor!
  setMotorPWM(int(MotorCommandL), int(MotorCommandR));
}



/////////////////////ROS STUFF/////////////////////////////

//Gets called whenever motor node from ros publishes something
void cmd_message(const std_msgs::Float32MultiArray& msg){
  
  //set the wheel velocities
  set_target(msg.data[LEFT],msg.data[RIGHT]);

  //update the last time
  last_received_time=millis();
}

std_msgs::Float32MultiArray Velocity_Out;

//ros node stuff
ros::NodeHandle nh;
ros::Publisher pub_actual("velocity_actual", &Velocity_Out);

ros::Subscriber<std_msgs::Float32MultiArray> sub_cmd("velocity_command", &cmd_message);


//////////////////////////////Setup//////////////////////////////////////////
void setup()
{
  // Some maths
  EncPerRad=(float(EncPerRev)/(2.0*3.14159));
  RadPerEncUs=(1e6/EncPerRad);
  
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  //get starting encoder
  oldLeft = EncMotorL.read();
  oldRight = EncMotorR.read();
  last_timeL = micros() - 1;
  last_timeR = micros() - 1;

  //initialize
  velocity_actual_left = 0.0;
  velocity_actual_right = 0.0;
  Setpoint_ActualLeft = 0.0;
  Setpoint_ActualRight = 0.0;
  MotorCommandL = 0.0;
  MotorCommandR = 0.0;
  
  //speed init
  setMotorPWM(0, 0);

  //ROS array size
  Velocity_Out.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 1);
  Velocity_Out.layout.dim[0].label = "Velocity_Out";
  Velocity_Out.layout.dim[0].size = ARRSIZE;
  Velocity_Out.layout.dim[0].stride = 1*ARRSIZE;
  Velocity_Out.layout.data_offset = 0;
  Velocity_Out.data_length = ARRSIZE;
  Velocity_Out.data = (float *)malloc(sizeof(float)*ARRSIZE);
  
  //Initialize ROS stuff
  nh.initNode();
  nh.advertise(pub_actual);
  nh.subscribe(sub_cmd);
  
  last_received_time=millis();
  last_sent_time=micros();
}


/////////////////////////////////////////Loop//////////////////////////////
void loop()
{

  /// Find a dt
  unsigned long current_time = micros();
  unsigned long dt = current_time - last_sent_time;
  last_sent_time = current_time;
  float tempfdt = float(dt/1000000.f);

  /// Run the controller
  VelocityController();

  //Send to ROS Node
  Velocity_Out.data[RIGHT] = velocity_actual_right;
  Velocity_Out.data[LEFT ] = velocity_actual_left;
  Velocity_Out.data[TIME] = tempfdt;
  
  pub_actual.publish( &Velocity_Out );

  delay(spin_time_ms);
  
  //check if timeout should trigger
  unsigned long timeout = millis() - last_received_time;
  if(timeout>20000ul){
    /// Tell the motors to shut their faces
    set_target(0.0,0.0);
  }

  nh.spinOnce();
}
