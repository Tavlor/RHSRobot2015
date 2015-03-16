/** \file
 * Definitions of class to control conveyor on the pallet jack.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the pallet jack's conveyor.
 */

#ifndef CONVEYOR_H
#define CONVEYOR_H
///Controls the conveyor on the pallet jack.

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
	float fConveyorSpeed = .5;

	void OnStateChange();
	void Run();
};

#endif			//CONVEYOR_H
