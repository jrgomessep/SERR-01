// Motor
// by Andrew Kramer and modified by Eroaldo Gomes

// controls direction and pwm input to a DC motor

#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor
{
public:
	Motor(int en_l, int en_r, int pwm_l, int pwm_r, bool reverse);
	void setFwd();
	void setBack();
	void setFree();
	void setStop();
	void setPWM(int level);
private:
	int _en_l, _en_r, _pwm_l, _pwm_r;
	bool _set_fwd, _set_back, _reverse;
};

#endif
