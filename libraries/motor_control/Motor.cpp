// Motor.cpp
// by Andrew Kramer and modified by Eroaldo Gomes

// Provides low-level control of a the speed and direction of a single DC motor
// by means of a motor driver such as the IBT-2

#include "Arduino.h"
#include "Motor.h"

// accepts three ints as parameters: 
//    the pin numbers for the direction control pins
//    and the pin number of the pwm pin
Motor::Motor(int en_l, int en_r, int pwm_l, int pwm_r, bool reverse)
{
	_en_l = en_l;
	_en_r = en_r;
	_pwm_l = pwm_l;
	_pwm_r = pwm_r;
	_set_fwd = false;
	_set_back = false;
	_reverse = reverse;
	pinMode(_en_l, OUTPUT);
	pinMode(_en_r, OUTPUT);
	pinMode(_pwm_l, OUTPUT);
	pinMode(_pwm_r, OUTPUT);
}

// sets the motor's direction to forward
void Motor::setFwd()
{
	_set_fwd = true;
	_set_back = false;
	analogWrite(_pwm_l, 0);
	analogWrite(_pwm_r, 0);
	digitalWrite(_en_l, HIGH);
	digitalWrite(_en_r, HIGH);
}

// sets the motor's direction to backward
void Motor::setBack()
{
	_set_fwd = false;
	_set_back = true;
	analogWrite(_pwm_l, 0);
	analogWrite(_pwm_r, 0);
	digitalWrite(_en_l, HIGH);
	digitalWrite(_en_r, HIGH);
}

// sets the motor to freewheel
void Motor::setFree()
{
	_set_fwd = false;
	_set_back = false;
	analogWrite(_pwm_l, 0);
	analogWrite(_pwm_r, 0);
	digitalWrite(_en_l, LOW);
	digitalWrite(_en_r, LOW);
}

// sets the motor to brake
void Motor::setStop()
{
	_set_fwd = true;
	_set_back = true;
	analogWrite(_pwm_l, 0);
	analogWrite(_pwm_r, 0);
	digitalWrite(_en_l, HIGH);
	digitalWrite(_en_r, HIGH);
}

// accepts an int, the PWM level, as a parameter
// sets the PWM output to the motor to the given int
// level must be between 0 and 255 inclusive
// behavior is undefined if level is outside this range
void Motor::setPWM(int level)
{
	if(_set_fwd == true && _set_back == false){
		if(_reverse){
			analogWrite(_pwm_l, level);
			analogWrite(_pwm_r, 0);
		}else{
			analogWrite(_pwm_l, 0);
			analogWrite(_pwm_r, level);
		}
	}else if(_set_fwd == false && _set_back == true){
		if(_reverse){
			analogWrite(_pwm_l, 0);
			analogWrite(_pwm_r, level);
		}else{
			analogWrite(_pwm_l, level);
			analogWrite(_pwm_r, 0);
		}
	} else {
		analogWrite(_pwm_l, 0);
		analogWrite(_pwm_r, 0);
	}
}
