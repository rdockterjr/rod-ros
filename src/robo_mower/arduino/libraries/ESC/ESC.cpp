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
	ESC_Servo.write(0) ;

	//analogWriteFrequency(_pwmpin, 58593);//optimal frequency for 10 bit
	//analogWriteResolution(10); //0-1023 range
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
	
	//deadband from 0-180 in servo, map 0-100 into 100-180
	int servo_val = 0;
	if(percent > 0){
		servo_val = 100 + int(0.8f*float(percent)); //80-180 range
	}
	//set pwm
  ESC_Servo.write(servo_val) ;     //setting pwm
	
	//store
	_lastpwm = servo_val;
}
