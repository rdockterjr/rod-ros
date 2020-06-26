
#include <Wire.h>
#include <LIDARLite.h>
#include <Servo.h>


// Globals
LIDARLite lidarLite;
Servo servoscan;  // create servo object to control a servo
#define servopin 2 //d2
int angle = 0;    // variable to store the servo position
int step_angle = 1; // 0->180->0 repeat
int angle_offset = 0; // hard coded, uncalibrated

void setup()
{
  Serial.begin(9600); // Todo: make serial to teensy

  //setup servo
  servoscan.attach(servopin);

  //setup lidar
  lidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  lidarLite.configure(0); // Change this number to try out alternate configurations

  //initialize
  angle = 90;
  servoscan.write(angle); 
  delay(15);
}

void loop()
{
  int dist;

  // Increment scan angle
  servoscan.write(angle); 
  delay(15); //wait to arrive before scanning

  // At the beginning of scan use bias correction
  if ( angle == 0 || angle == 180) {
    dist = lidarLite.distance();      // With bias correction
  } else {
    dist = lidarLite.distance(false); // Without bias correction
  }

  // Display distance
  int true_angle = angle + angle_offset;
  Serial.print(true_angle);
  Serial.print(",");
  Serial.println(dist);

  //angle update
  if(angle == 180){
    step_angle = -1;
  }
  if(angle == 0){
    step_angle = 1;
  }
  angle += step_angle;
}
