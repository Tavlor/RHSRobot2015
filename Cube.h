/** \file
 * Definitions of class to control tote lifter on the cube.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the cube's tote lifter.
 */

#ifndef CUBE_H
#define CUBE_H



#include <pthread.h>

//Robot
#include "WPILib.h"

#include "ComponentBase.h"			//For ComponentBase class

class Cube : public ComponentBase
{
public:
	Cube();
	virtual ~Cube();
	static void *StartTask(void *pThis)
	{
		((Cube *)pThis)->DoWork();
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
	Timer *pSafetyTimer;

	bool bEnableAutoCycle;
	bool hitTop;
	bool hitBottom;

	float fClickerPaused;
	float fLifterPaused;

	/* CANLIFTER
	 * 	Raise: -  (using CIM, not bag)
	 *	Lower: +
	 * CLICKER
	 *	Raise:+	  (using CIM, not bag)
	 *	Lower:-
	 * INTAKE     (using bag)
	 *	Run: -
	 */
	const float fLifterRaise = -1.0;
	const float fLifterLower = 1.0;
	const float fLifterStop = 0.0;
	const float fClickerRaise = 1.0;
	const float fClickerLower = -1.0;
	const float fClickerHover = 0.55;
	const float fClickerStop = 0.0;
	const float fIntakeRun = -0.5;
	const float fIntakeStop = 0.0;

	ClickerState clickerLastState = STATE_CLICKER_TOP;
	LifterState lifterLastState = STATE_LIFTER_TOP;
	int iNumOfTotes = 0;
	int iLastChecked;

	//All Cube sensors are connected to the Talons, and are thus not
	// represented in the code.
	void OnStateChange();
	void Run();
};

#endif			//CUBE_H
