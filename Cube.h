/** \file
 * Definitions of class to control tote lifter on the cube.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the cube's tote lifter.
 */

#ifndef CUBE_H
#define CUBE_H


/**
	Runs a state machine to operate automatic tote stacking.
\dot
digraph Cube {
	label="Clicker State Machine";
	START -> CLICKER_RAISE [label = "COMMAND_CUBEAUTOCYCLE_START"];
	CLICKER_RAISE -> CLICKER_TOP [label = "Top Sensor TRUE"];
	CLICKER_TOP -> CLICKER_GATEDELAY [label = "IR Sensor TRUE"];
	CLICKER_TOP -> CLICKER_GATEDELAY [label = "Driver Ready\nto Remove"];
	CLICKER_GATEDELAY -> CLICKER_LOWER [label = "Delay Expired"];
	CLICKER_LOWER -> CLICKER_BOTTOM [label = "Bottom Sensor TRUE"];
	CLICKER_BOTTOM -> CLICKER_BOTTOMHOLD [label = "Driver Ready\nto Remove"];
	CLICKER_BOTTOM -> CLICKER_RAISE [label = "Driver Not Ready"];
	CLICKER_BOTTOMHOLD -> CLICKER_DELAYAFTERCYLE [label = "IR Sensor FALSE"];
	CLICKER_DELAYAFTERCYLE -> CLICKER_RAISE [label = "Delay Expired"];
}
\enddot
*/

#include "WPILib.h"
#include "ComponentBase.h"
#include "RobotParams.h"

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
			STATE_CLICKER_GATEDELAY,
			STATE_CLICKER_FIRSTTOTEDELAY,
			STATE_CLICKER_TOP,
			STATE_CLICKER_BOTTOM,
			STATE_CLICKER_BOTTOMHOLD,
			STATE_CLICKER_DELAYAFTERCYLE
	};

	CANTalon *clickerMotor;
	CANTalon *intakeMotor;
	Timer *pSafetyTimer;
	//Timer *pAutoTimer;IN COMPONENT BASE
	//Timer *pRemoteUpdateTimer;
	Timer *pInterCycleTimer;
	Timer *pGateTimer;

	bool bEnableAutoCycle;
	bool bPrepareToRemove;
	float fClickerPaused;
	bool bFirstTote;

	/* CLICKER
	 *	Raise:+	  (using CIM, not bag)
	 *	Lower:-
	 * INTAKE     (using bag)
	 *	Run: -
	 */
	const float fClickerRaise = 1.0;
	const float fClickerLower = -1.0;
	const float fClickerTopHold = 1.0;
	const float fClickerStop = 0.0;
	const float fIntakeRun = -0.5;
	const float fIntakeStop = 0.0;
	const float fDelayAfterCycle = 1.5;

	ClickerState clickerLastState = STATE_CLICKER_TOP;

	bool irBlocked;
	bool topBlocked;
	bool clickerHallEffectBottom;
	bool clickerHallEffectTop;

	void OnStateChange();
	void Run();
};

#endif			//CUBE_H
