// Encoder
// by Andrew Kramer

// initializes an object to read input from a shaft encoder and
// track the number of encoder ticks using interrupts
// only works with Arduino Uno, Duemilanove, and others that
// have interrupt 0 defined as digital pin 2 and interrupt 1
// defined as digital pin 3

#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"

class Encoder
{
public:
	Encoder(int encoderA, int encoderB, 
			long deltaT, int ticksPerRev);
	float getSpeed(); // returns speed in degrees per second
	float getDistance(); // returns distance rotated in degrees
	float getDistanceRT();
	int getCount();
	void updateCount();
	void resetDistance();
private:
	int _encoderA, _encoderB; // encoder pins
	double _degPerTick; // degrees of output shaft rotation per encoder tick
	volatile long _count, _oldCount, _newCount, _oldCountRT, _newCountRT;
	long _deltaT; // in microseconds
	int _lastSpeed;
	long _totalCount, _totalCountRT;
};

#endif
