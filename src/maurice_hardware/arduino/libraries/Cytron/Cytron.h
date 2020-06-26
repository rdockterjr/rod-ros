/*
  Cytron.h - Library for controlling motors with Cytron MD30C
  Created by Rod Dockter, December 13, 2018.
*/
//https://www.pjrc.com/teensy/td_pulse.html
//FTM0	5, 6, 9, 10, 20, 21, 22, 23	 Default: 488.28 Hz
//FTM1	3, 4
//FTM2	29, 30
//FTM3	2, 7, 8, 14, 35, 36, 37, 38

#ifndef Cytron_h
#define Cytron_h

#include "Arduino.h"

#define MAX_PWM_10_BIT 1023

class Cytron
{
  public:
    Cytron(int pwmpin, int dirpin, int forwarddir);
		void SetFrequency(int hz);
    void Init();
    void Control(int pwm_10bit);
  private:
    int _pwmpin;
		int _dirpin;
		int _lastpwm;
		int _lastdir;
		int _forwarddir;
};

#endif