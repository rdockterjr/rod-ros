#include <Cytron.h>

#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>

#define LOOP_TME 100
#define COUNTS_PER_REV 1024
#define GEAR_RATIO 17.75
#define FLOOR_VELOCITY 0.01
#ifndef M_PI
  #define M_PI 3.14159265358
#endif

//Pin Assignments
int pwmpin_left = 35;
int dirpin_left = 34;
int pwmpin_right = 38;
int dirpin_right = 39;
int enca_left = 24;
int encb_left = 25;
int enca_right = 11;
int encb_right = 12;

//cytron class
Cytron left_motor(pwmpin_left, dirpin_left, HIGH);
Cytron right_motor(pwmpin_right, dirpin_right, HIGH);


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
float Ki_R = 500.0; //current error
float Kp_L = 0.0; //error diff
float Ki_L = 500.0; //current error

//encoder setup
Encoder LeftEnc(enca_left, encb_left);
Encoder RightEnc(enca_right, encb_right);

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
  
  //Motor commandings in PWM units -1023 to 1023
  motor_command_left = motor_command_left + changeL;
  motor_command_right = motor_command_right + changeR;

  if(motor_command_left > MAX_PWM_10_BIT){
    motor_command_left = MAX_PWM_10_BIT;
  }
  if(motor_command_left < -MAX_PWM_10_BIT){
    motor_command_left = -MAX_PWM_10_BIT;
  }
  if(motor_command_right > MAX_PWM_10_BIT){
    motor_command_right = MAX_PWM_10_BIT;
  }
  if(motor_command_right < -MAX_PWM_10_BIT){
    motor_command_right = -MAX_PWM_10_BIT;
  }

  if(velocity_cmd_left < FLOOR_VELOCITY && velocity_cmd_left > -FLOOR_VELOCITY && velocity_actual_left < FLOOR_VELOCITY && velocity_actual_left > -FLOOR_VELOCITY){
    motor_command_left = 0.0;
  }
  if(velocity_cmd_right < FLOOR_VELOCITY && velocity_cmd_right > -FLOOR_VELOCITY && velocity_actual_right < FLOOR_VELOCITY && velocity_actual_right > -FLOOR_VELOCITY){
    motor_command_right = 0.0;
  }
  
  //Move the new value into the past
  error_prev_left=error_now_left;
  error_prev_right=error_now_right;
}


void setup() {
  // set up pin modes
  left_motor.Init();
  right_motor.Init();

  //initialize variables
  velocity_actual_left = 0.0;
  velocity_actual_right = 0.0;
  velocity_cmd_left = 0.0;
  velocity_cmd_right = 0.0;
  motor_command_left = 0;
  motor_command_right = 0;

  //initialize encoders
  last_time = millis();
  last_pid_time = millis();
  last_left = 0;
  last_right = 0;
  counts_2_rads = (2.0*M_PI*1000.0)/(float(COUNTS_PER_REV)*GEAR_RATIO);

  Serial.begin(9600);
}


void loop() {
   //update velocities
  compute_velocities();

  // Compute Outputs
  compute_motor_commands();

  // Send Motor commands
  int out_left = motor_command_left;
  int out_right = motor_command_right;
  left_motor.Control(out_left);
  right_motor.Control(out_right);
  
  if (Serial.available() > 0) {
    // read the val
    int percent = Serial.parseInt();
    while(Serial.available() >0){
      Serial.read(); //ignore
    }

    velocity_cmd_left = float(percent)/100.0; 
    velocity_cmd_right = float(percent)/100.0;
  }

      // say what you got:
  Serial.print(velocity_actual_left);
  Serial.print(",");
  Serial.println(velocity_actual_right);

  
  delay(100);

}
