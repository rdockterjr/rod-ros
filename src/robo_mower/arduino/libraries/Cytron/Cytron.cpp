/*
  Cytron.h - Library for controlling motors with Cytron MD30C
  Created by Rod Dockter, December 13, 2018.
*/

#include "Arduino.h"
#include "Cytron.h"

Cytron::Cytron(int pwmpin, int dirpin, int forwarddir)
{
  _pwmpin = pwmpin;
	_dirpin = dirpin;
	_forwarddir = forwarddir;
}

//setup pins
void Cytron::Init()
{
  pinMode(_dirpin, OUTPUT);
  pinMode(_pwmpin, OUTPUT);
  digitalWrite(_dirpin, _forwarddir);
  analogWrite(_pwmpin,0);
}

//percent 0-1000
void Cytron::Control(int percent)
{
	//bound it
	if(percent > 1000){
		percent = 1000;
	}
	if(percent < - 1000){
		percent = -1000;
	}
	//set direction
  if(percent > 0){
    digitalWrite(_dirpin, _forwarddir);
		_lastdir = _forwarddir;
  }
  else{
    digitalWrite(_dirpin, !_forwarddir);
		_lastdir = !_forwarddir;
  }
	//compute pwm value and write
  int pwmval = (float(abs(percent))/1000.0)*255.0;
  analogWrite(_pwmpin,pwmval);
	
	//store
	_lastpwm = pwmval;
}