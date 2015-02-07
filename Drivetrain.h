/*
 * The Drivetrain component class handles driving related functionality.
 */

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <pthread.h>

//Robot
#include "ComponentBase.h"			//For ComponentBase class

//WPILib
#include "WPILib.h"
#include "ADXRS453Z.h"


const float JOYSTICK_DEADZONE = 0.10;
const float MAX_GAIN_PER_MESSAGE = 0.1;

class Drivetrain : public ComponentBase
{
public:
	Drivetrain();
	~Drivetrain();
	static void *StartTask(void *pThis)
	{
		((Drivetrain *)pThis)->DoWork();
		return(NULL);
	}
private:
	CANTalon* leftMotor;
	CANTalon* rightMotor;
	ADXRS453Z *gyro;
	Encoder *encoder;
	float left, right; //previous motor values
	float initialAngle, errorAngle;
	float coveredDist;

	void OnStateChange();
	void Run();
	void DriveStraight(float);
	float LimitMotor(float,float);
};

#endif			//DRIVETRAIN_H
