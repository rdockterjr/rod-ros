/*
  ESC.h - Library for controlling brushless motors with ESC
  Created by Rod Dockter, December 13, 2018.
*/


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
		int _lastpwm;
  private:
    int _pwmpin;
		Servo ESC_Servo;
};

#endif
