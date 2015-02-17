/** \file
 * Definitions of class to control conveyor on the pallet jack.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the pallet jack's conveyor.
 */

#ifndef CONVEYOR_H
#define CONVEYOR_H

#include <pthread.h>

//Robot
#include "WPILib.h"

#include "ComponentBase.h"			//For ComponentBase class

class Conveyor: public ComponentBase
{
public:
	Conveyor();
	virtual ~Conveyor();
	static void *StartTask(void *pThis)
	{
		((Conveyor*)pThis)->DoWork();
		return(NULL);
	}

private:
	CANTalon *conveyorMotor;
	CANTalon *intakeLeftMotor;
	CANTalon *intakeRightMotor;

	float fConveyorSpeed = .5;
	float fIntakeSpeed = .75;	//typical vertical intake speed
	float fAdjustSpeed = .15;	//intake speed used when shifting cans

	void OnStateChange();
	void Run();
};

#endif			//CONVEYOR_H
