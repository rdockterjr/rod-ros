/*
  Cytron.h - Library for controlling motors with Cytron MD30C
  Created by Rod Dockter, December 13, 2018.
*/
#ifndef Cytron_h
#define Cytron_h

#include "Arduino.h"

class Cytron
{
  public:
    Cytron(int pwmpin, int dirpin, int forwarddir);
    void Init();
    void Control(int percent);
  private:
    int _pwmpin;
		int _dirpin;
		int _lastpwm;
		int _lastdir;
		int _forwarddir;
};

#endif