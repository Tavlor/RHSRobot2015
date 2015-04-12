/** \file
 * Definitions of class to control the drive train.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the pallet jack wheels.
 */

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <pthread.h>
//#include <unistd.h>

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

	bool GetToteSensor();
	bool GetGyroAngle();
private:

	CANTalon* leftMotor;
	CANTalon* rightMotor;
	ADXRS453Z *gyro;
	Encoder *encoder;
	BuiltInAccelerometer accelerometer;
	DigitalInput *toteSensor;
	Timer *pAutoTimer; //watches autonomous time and disables it if needed.
	//stores motor values during autonomous
	float left = 0;
	float right = 0;

	bool bFrontLoadTote = false;
	bool bBackLoadTote = false;
	bool bKeepAligned = false;
	const float fFrontLoadSpeed = .250;
	const float fBackLoadSpeed = -.250;
	const float fToteSeekSpeed = -.50;

	///how strong direction recovery is in straight drive, higher = stronger
	const float recoverStrength = .04;
	const float fMaxRecoverSpeed = .3;
	const float fMaxRecoverAngle = 30.0; 		//used to keep straight drive recovery from becoming to violent
	///how far from goal the robot can be before stopping
	const float distError = 1.0;				//inches
	const float angleError = 2.0;				//degrees
	const float turnAngleSpeedMultiplyer = .03;

	//angle * mult = speed to be reduced by limit
	const float turnSpeedLimit = .50;
	const float fEncoderRatio = 0.023009;

	//diameter*pi/encoder_resolution : 1.875 * 3.14 / 256

	void OnStateChange();
	void Run();
	void Put();//for SmartDashboard
	void ArcadeDrive(float, float);
	void MeasuredMove(float,float);
	void Turn(float,float);
	void KeepAligned();
	void SeekTote(float,float);
	void StraightDrive(float, float);
	void StraightDriveLoop(float);
};

#endif			//DRIVETRAIN_H
