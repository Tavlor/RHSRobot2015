/**  Definitions of class to control tote lifter on the cube.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the cube's tote lifter.
 */

#ifndef CLICKER_H
#define CLICKER_H



#include <pthread.h>

//Robot
#include "ComponentBase.h"			//For ComponentBase class

//WPILib
#include "WPILib.h"

class Clicker : public ComponentBase
{
public:
	Clicker();
	virtual ~Clicker();
	static void *StartTask(void *pThis)
	{
		((Clicker *)pThis)->DoWork();
		return(NULL);
	}

private:

	enum ClickerState{
			STATE_CUBECLICKER_RAISE,
			STATE_CUBECLICKER_LOWER,
			STATE_CUBECLICKER_TOP,
			STATE_CUBECLICKER_BOTTOM
	};

	CANTalon *clickerMotor;
	CANTalon *intakeMotor;
	bool bEnableAutoCycle;
	bool bAutoCubeIntake;

	//bool bEnableAutoCycle;
	bool hitTop;
	bool hitBottom;

	ClickerState lastState = STATE_CUBECLICKER_TOP;
	int iNumOfTotes = 1;
	int iLastChecked;

	//All Cube sensors are connected to the Talons, and are thus not
	// represented in the code.
	void OnStateChange();
	void Run();
	void Top();
	void Bottom();
	void Lower();
	void Raise();
};

#endif			//CLICKER_H
