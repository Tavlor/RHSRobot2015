/**  Definitions of class to control the drive train.
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
	float left, right; //motor values
	float targetAngle, errorAngle;
	float coveredDist;
	float speed = .2;

	const float recoverStrength = 15;

	void OnStateChange();
	void Run();
	void Put();//for SmartDashboard
	void ArcadeDrive(float, float);
	void DriveStraight(float);
	void Turn(float);//for auto, use with DriveStraight
	void AddDistance(float);
};

#endif			//DRIVETRAIN_H
