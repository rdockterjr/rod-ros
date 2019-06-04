/*
  ESC.h - Library for controlling Brushless Motors with ESC
  Created by Rod Dockter, December 13, 2018.
*/

#include "ESC.h"


//pwmpin (white)
ESC::ESC(int pwmpin)
{
  _pwmpin = pwmpin;
}

//setup pins
void ESC::Init()
{
	//setup freqeuncy and resolution
	ESC_Servo.attach(_pwmpin) ;  //Set pwmpin as servo
	ESC_Servo.write(90) ; //init the esc by writing half? (180/2=90)

}

//percent 0-100
void ESC::Control(int percent)
{
	//bound it
	if(percent > MAX_ESC){
		percent = MAX_ESC;
	}
	if(percent < MIN_ESC){
		percent = MIN_ESC;
	}
	
	//deadband from 0-90 in servo, map 0-100 into 90-180
	int servo_val = 90;
	if(percent > 10){
		servo_val = 80 + percent; //90-180 range
	}
	//set pwm
  ESC_Servo.write(servo_val) ;     //setting pwm
	
	//store
	_lastpwm = servo_val;
}
