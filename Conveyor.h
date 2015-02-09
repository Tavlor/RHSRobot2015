/**  Definitions of class to control conveyor on the pallet jack.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the pallet jack's conveyor.
 */

#ifndef CONVEYOR_H
#define CONVEYOR_H

#include <pthread.h>

//Robot
#include "ComponentBase.h"			//For ComponentBase class

//WPILib
#include "WPILib.h"

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

	void OnStateChange();
	void Run();
};

#endif			//CONVEYOR_H
