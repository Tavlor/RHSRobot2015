/**  Definitions of class to control tote lifter on the pallet jack.
 *
 * This class is derived from the standard Component base class and includes
 * definitions for the devices used to control the pallet jack's tote lifter.
 */


#ifndef JACKCLICKER_H
#define JACKCLICKER_H

#include <pthread.h>

//Robot
#include "ComponentBase.h"			//For ComponentBase class

//WPILib
#include "WPILib.h"

class JackClicker : public ComponentBase
{
public:
	JackClicker();
	virtual ~JackClicker();
	static void *StartTask(void *pThis)
	{
		((JackClicker *)pThis)->DoWork();
		return(NULL);
	}

private:
	CANTalon *clickerMotor;

	//All Cube sensors are connected to the Talons, and are thus not
	// represented in the code.
	void OnStateChange();
	void Run();
};

#endif			//JACKCLICKER_H
