/**  Definitions of class to control can lifter on the pallet jack.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the pallet jack's can lifter.
 * NOTE: Worked into the clicker class, files kept as backup
 */

#ifndef CANLIFTER_H
#define CANLIFTER_H

#include <pthread.h>

//Robot
#include "WPILib.h"

#include "ComponentBase.h"			//For ComponentBase class

class CanLifter : public ComponentBase
{
public:
	CanLifter();
	virtual ~CanLifter();
	static void *StartTask(void *pThis)
	{
		((CanLifter *)pThis)->DoWork();
		return(NULL);
	}

private:
	CANTalon *lifterMotor;

	//All Cube sensors are connected to the Talons, and are thus not
	// represented in the code.
	void OnStateChange();
	void Run();
};

#endif			//CANLIFTER_H
