/*
  Cytron.h - Library for controlling motors with Cytron MD30C
  Created by Rod Dockter, December 13, 2018.
*/

#include "Arduino.h"
#include "Cytron.h"

//pwmpin (red)
//dirpin (yellow)
//forwarddir (LOW/HIGH): switches forward direction
Cytron::Cytron(int pwmpin, int dirpin, int forwarddir)
{
  _pwmpin = pwmpin;
	_dirpin = dirpin;
	_forwarddir = forwarddir;
}

//setup frequency
void Cytron::SetFrequency(int hz){
	analogWriteFrequency(_pwmpin, hz); //find optimal frequency for 10 bit resolution (board dependent)
}

//setup pins
void Cytron::Init()
{
	//setup freqeuncy and resolution
	SetFrequency(58593); //optimal frequency for 10 bit
	analogWriteResolution(10); //0-1023 range
	//setup pin modes and initialize
  pinMode(_dirpin, OUTPUT);
  pinMode(_pwmpin, OUTPUT);
  digitalWrite(_dirpin, _forwarddir);
  analogWrite(_pwmpin,0);
}

//percent -1023 < x < 1023
void Cytron::Control(int pwm_10bit)
{
	//bound it
	if(pwm_10bit > MAX_PWM_10_BIT){
		pwm_10bit = MAX_PWM_10_BIT;
	}
	if(pwm_10bit < -MAX_PWM_10_BIT){
		pwm_10bit = -MAX_PWM_10_BIT;
	}
	//set direction
  if(pwm_10bit > 0){
    digitalWrite(_dirpin, _forwarddir);
		_lastdir = _forwarddir;
  }
  else{
    digitalWrite(_dirpin, !_forwarddir);
		_lastdir = !_forwarddir;
  }
	//write pwm value
  analogWrite(_pwmpin,abs(pwm_10bit));
	
	//store
	_lastpwm = pwm_10bit;
}