/** \file
 * Definitions of class to control tote lifter on the cube.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the cube's tote lifter.
 */

#ifndef CANLIFTER_H
#define CANLIFTER_H
///Controls the can lift claw on the pallet jack.

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
	CANTalon *clawMotor;
	Timer *pSafetyTimer;
	Timer *pUpdateTimer;
	Timer *pClawTimer;


	/* CANLIFTER  (using bag, not CIM)
	 * 	Raise: +
	 *	Lower: -
	 *
	 *	CLAW  (using bag, not CIM)
	 *	Open: -
	 *	Lower: +
	 */
	const float fLifterRaise = 1.0;		//not needed
	const float fLifterLower = -1.0;	//not needed
	const float fLifterHover = .15;
	const float fLifterStop = 0.0;

	const float fClawOpen = -.25;
	const float fClawClose = .75;
	const float fClawStop = 0.0;
	const float fClawMotorCurrentMax = 30.0;

	bool lifterHallEffectBottom;
	bool lifterHallEffectTop;
	bool bHover;						//should the lift keep itself at a constant height?

	///claw is open or closed
	//bool bClawOpen;
	float clawStopCurrent;				//used to tell when the claw mechanism is pressing against the bar

	void OnStateChange();
	void Run();
};

#endif			//CANLIFTER_H
