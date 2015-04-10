/** \file
 * Definitions of class to control can lifter on the pallet jack.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the pallet jack's can lifter.
 */

#ifndef CANLIFTER_H
#define CANLIFTER_H

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

	bool GetHallEffectTop() { return lifterMotor->IsFwdLimitSwitchClosed();	}
	bool GetHallEffectMiddle() { return midHallEffect->Get(); }
	bool GetHallEffectBottom() { return lifterMotor->IsRevLimitSwitchClosed(); }
	bool GetGoingUp() { return(bGoingUp);}
	bool GetGoingDown() { return(bGoingDown);}

private:

	CANTalon *lifterMotor;
	DigitalInput *midHallEffect;
	Counter *midDetect;
	Timer *pSafetyTimer;


	/* CANLIFTER  (using bag, not CIM)
	 * 	Raise: +
	 *	Lower: -
	 */
	const float fLifterRaise = 1.0;
	const float fLifterLower = -1.0;

	const float fLifterHoverNoTotes = .15;
	const float fLifterHoverOneTotes = .20;
	const float fLifterHoverTwoTotes = .25;
	const float fLifterHoverThreeTotes = .30;

	const float fLifterLiftNoTotes = 0.50;
	const float fLifterLiftOneTotes = 1.0;
	const float fLifterLiftTwoTotes = 1.0;
	const float fLifterLiftThreeTotes = 0.35;

	const float fLifterLowerNoTotes = -0.50;
	const float fLifterLowerOneTotes = -0.50;
	const float fLifterLowerTwoTotes = -0.50;
	const float fLifterLowerThreeTotes = -0.50;

	const float fLifterStop = 0.0;
	const float fLifterMotorCurrentMax = 30;

	bool lifterHallEffectBottom;
	bool lifterHallEffectTop;
	bool bHover;
	bool bMiddleHover;
	bool bGoingUp;
	bool bGoingDown;
	int iToteLoad;


	void OnStateChange();
	void Run();
	bool CheckLifterCurrentOK();
};

#endif			//CANLIFTER_H
