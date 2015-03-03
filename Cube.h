/** \file
 * Definitions of class to control tote lifter on the cube.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the cube's tote lifter.
 */

#ifndef CUBE_H
#define CUBE_H


/**
\dot
digraph Cube {
	label="Clicker State Machine";
	START -> CLICKER_RAISE [label = "COMMAND_CUBEAUTOCYCLE_START"];
	CLICKER_RAISE -> CLICKER_TOP [label = "Top Sensor TRUE"];
	CLICKER_TOP -> CLICKER_TOP [label = "Totes = 5, Lifter WAITTILLRAISE"];
	CLICKER_TOP -> CLICKER_LOWER [label = "IR Sensor TRUE"];
	CLICKER_LOWER -> CLICKER_BOTTOM [label = "Bottom Sensor TRUE"];
	CLICKER_BOTTOM -> CLICKER_BOTTOMHOLD [label = "Totes = 5"];
	CLICKER_BOTTOM -> CLICKER_BOTTOMHOLD [label = "Totes = 6"];
	CLICKER_BOTTOM -> CLICKER_RAISE [label = "Totes < 5"];
	CLICKER_BOTTOMHOLD -> CLICKER_DELAYAFTERCYLE [label = "IR Sensor FALSE, Totes = 6"];
	CLICKER_BOTTOMHOLD -> CLICKER_RAISE [label = "Lifter UP, Totes = 5"];
	CLICKER_DELAYAFTERCYLE -> CLICKER_RAISE [label = "Delay Expired", Lifter LOWER];
}
\enddot
*/

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
			STATE_CLICKER_BOTTOMHOLD,
			STATE_CLICKER_DELAYAFTERCYLE
	};

	enum LifterState {
		STATE_LIFTER_BOTTOM,
		STATE_LIFTER_WAITTILLRAISE,
		STATE_LIFTER_RAISE,
		STATE_LIFTER_TOP,
		STATE_LIFTER_LOWER
	};

	CANTalon *clickerMotor;
	CANTalon *intakeMotor;
	CANTalon *lifterMotor;
	Timer *pSafetyTimer;
	Timer *pAutoTimer;
	Timer *pRemoteUpdateTimer;
	Timer *pInterCycleTimer;

	bool bEnableAutoCycle;
	bool hitTop;
	bool hitBottom;
	bool bOkToRaiseCan;

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
	const float fClickerTopHold = .75;
	const float fClickerHover = 0.55;
	const float fClickerStop = 0.0;
	const float fIntakeRun = -0.5;
	const float fIntakeStop = 0.0;

	ClickerState clickerLastState = STATE_CLICKER_TOP;
	LifterState lifterLastState = STATE_LIFTER_TOP;
	int iNumOfTotes = 0;
	int iLastChecked;

	bool irBlocked;
	bool clickerHallEffectBottom;
	bool clickerHallEffectTop;
	bool lifterHallEffectBottom;
	bool lifterHallEffectTop;

	//All Cube sensors are connected to the Talons, and are thus not
	// represented in the code.
	void OnStateChange();
	void Run();
};

#endif			//CUBE_H
