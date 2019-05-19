/*
  ESC.h - Library for controlling brushless motors with ESC
  Created by Rod Dockter, December 13, 2018.
*/
//https://www.pjrc.com/teensy/td_pulse.html
//FTM0	5, 6, 9, 10, 20, 21, 22, 23	 Default: 488.28 Hz
//FTM1	3, 4
//FTM2	29, 30
//FTM3	2, 7, 8, 14, 35, 36, 37, 38

#ifndef ESC_h
#define ESC_h

#include "Arduino.h"
#include "Servo.h" //used for esc control

#define MAX_ESC 100 
#define MIN_ESC 0

class ESC
{
  public:
    ESC(int pwmpin);
    void Init();
    void Control(int pwm_10bit);
  private:
    int _pwmpin;
		int _lastpwm;
		Servo ESC_Servo;
};

#endif
