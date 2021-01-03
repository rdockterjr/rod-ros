
#include <Wire.h>
#include <LIDARLite.h>
#include <Servo.h>


// Globals
LIDARLite lidarLite; //black=gnd, red=5v, blue=sda (A4), green=scl (A5)
Servo servoscan;  // create servo object to control a servo
#define servopin 3 //d3

int angle = 0;    // variable to store the servo position
int center_angle = 98; //offset (calibrated)
int angle_fov = 120; //FOV

int step_angle = 2; // min>angle>max repeat
bool forward = true;

int min_angle = center_angle - (angle_fov/2);
int max_angle = center_angle + (angle_fov/2);

void setup()
{
  Serial.begin(38400); // Todo: make serial to teensy

  //setup servo
  servoscan.attach(servopin);

  //setup lidar
  lidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  lidarLite.configure(0); // Change this number to try out alternate configurations

  //initialize
  angle = center_angle;
  servoscan.write(angle); 
  delay(500);
}

void loop()
{
  int dist;

  // Increment scan angle
  servoscan.write(angle); 
  delay(50); //wait to arrive before scanning

  // At the beginning of scan use bias correction
  if (forward && angle >= max_angle) {
    dist = lidarLite.distance();      // With bias correction
  } else if (!forward && angle <= min_angle) {
    dist = lidarLite.distance();      // With bias correction
  } else {
    dist = lidarLite.distance(false); // Without bias correction
  }

  // Display distance
  int true_angle = angle - center_angle;
  
  Serial.print(dist);
  Serial.print(",");
  Serial.println(true_angle);

  //angle update
  if(forward && angle >= max_angle){
    step_angle = -2;
    forward = false;
  }
  if(!forward && angle <= min_angle){
    step_angle = 2;
    forward = true;
  }
  angle += step_angle;
}
