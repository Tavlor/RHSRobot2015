/**  Definitions of class to control tote lifter on the cube.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the cube's tote lifter.
 */

#ifndef CLICKER_H
#define CLICKER_H



#include <pthread.h>

//Robot
#include "WPILib.h"

#include "ComponentBase.h"			//For ComponentBase class

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
			STATE_CLICKER_RAISE,
			STATE_CLICKER_LOWER,
			STATE_CLICKER_TOP,
			STATE_CLICKER_BOTTOM,
			STATE_CLICKER_BOTTOMHOLD
	};

	enum LifterState {
		STATE_LIFTER_BOTTOM,
		STATE_LIFTER_RAISE,
		STATE_LIFTER_TOP,
		STATE_LIFTER_LOWER
	};

	CANTalon *clickerMotor;
	CANTalon *intakeMotor;
	CANTalon *lifterMotor;
	bool bEnableAutoCycle;
	bool bAutoCubeIntake;

	//bool bEnableAutoCycle;
	bool hitTop;
	bool hitBottom;

	ClickerState clickerLastState = STATE_CLICKER_TOP;
	LifterState lifterLastState = STATE_LIFTER_BOTTOM;
	int iNumOfTotes = 0;
	int iLastChecked;

	//All Cube sensors are connected to the Talons, and are thus not
	// represented in the code.
	void OnStateChange();
	void Run();
	void Top();
	void Bottom();
	void Lower();
	void Raise();
	void Reset();
};

#endif			//CLICKER_H
