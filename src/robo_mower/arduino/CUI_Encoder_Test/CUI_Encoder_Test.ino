


// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability

#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>

#define LOOP_TME 100
#define COUNTS_PER_REV 1024

Encoder LeftEnc(11, 12);
Encoder RightEnc(24, 25);

//velocity
long left_diff, right_diff, last_left, last_right, new_left, new_right;
float velocity_left, velocity_right;
float counts_2_rads;
unsigned long last_time, new_time, time_diff;

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");

  //initialize
  last_time = millis();
  last_left = 0;
  last_right = 0;
  counts_2_rads = (2.0*M_PI*1000.0)/(float(COUNTS_PER_REV));
}

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
  //compute veloicities
  velocity_left = (float(left_diff)/float(time_diff))*counts_2_rads;
  velocity_right = (float(right_diff)/float(time_diff))*counts_2_rads;
  //update
  last_left = new_left;
  last_right = new_right;
  last_time = new_time;
}


void loop() {
  compute_velocities();
  Serial.print(velocity_left);
  Serial.print(",");
  Serial.println(velocity_right);
  
  delay(LOOP_TME);
}
