/** \file
 * Definitions of class to control the drive train.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the pallet jack wheels.
 */

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <pthread.h>

//Robot
#include "WPILib.h"

#include "ComponentBase.h"			//For ComponentBase class
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
	BuiltInAccelerometer accelerometer;
	Timer *pAutoTimer; //watches autonomous time and disables it if needed.
	bool bIsAuto = false;
	//AutoMode autoMode = AUTO_DRIVETRAIN_STOP;
	//stores motor values during autonomous
	float left = 0;
	float right = 0;

	const float fAutoLength = 15.0; //number of seconds for autonomous
	//how strong direction recovery is, lower = stronger
	const float recoverStrength = 15;
	//how far from goal the robot can be before stopping
	const float distError = 1;//inches
	const float angleError = 3;//degrees
	const float turnSpeedLimit = .3;
	const float fEncoderRatio = 0.023009;
	//diameter*pi/encoder_resolution : 1.875 * 3.14 / 256

	void OnStateChange();
	void Run();
	void Put();//for SmartDashboard
	void ArcadeDrive(float, float);
	void MeasuredMove(float,float);
	void Turn(float);
	void StraightDrive(float);
};

#endif			//DRIVETRAIN_H
