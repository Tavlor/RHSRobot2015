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

	bool GetHallEffectMiddle() { return lifterMotor->IsFwdLimitSwitchClosed();	}
	bool GetGoingUp() { return(bGoingUp);}
	bool GetGoingDown() { return(bGoingDown);}

private:

	CANTalon *lifterMotor;
	DigitalInput *hoverHallEffect;
	Counter *hoverHallDetect;
	/*DigitalInput *upperHall;
	DigitalInput *lowerHall;
	Counter *upperDetect;
	Counter *lowerDetect;*/
	Timer *pSafetyTimer;
	//Timer *pAutoTimer;IN COMPONENT BASE


	/* CANLIFTER  (using bag, not CIM)
	 * 	Raise: -
	 *	Lower: +
	 */
	const float fLifterUpMult = -1.0;
	const float fLifterDownMult = 1.0;

	const float fLifterRaise = -1.0;
	const float fLifterStartRaise = -0.6;
	const float fLifterRaiseLoMid = -0.75;
	const float fLifterLower = 1.0;
	const float fLifterHover = -.25;
	const float fLifterStop = .0;

	const float fLifterHoverNoTotes = -.2;
	const float fLifterHoverOneTotes = -.20;
	const float fLifterHoverTwoTotes = -.25;
	const float fLifterHoverThreeTotes = -.30;

	const float fLifterLiftNoTotes = -0.50;
	const float fLifterLiftOneTotes = -1.0;
	const float fLifterLiftTwoTotes = -1.0;
	const float fLifterLiftThreeTotes = -0.35;//don't ask.

	const float fLifterLowerNoTotes = 0.50;
	const float fLifterLowerOneTotes = 0.50;
	const float fLifterLowerTwoTotes = 0.50;
	const float fLifterLowerThreeTotes = 0.50;

	const float fLifterMotorCurrentMax = 30;
	const float fLifterMotorCurrentMaxOneCan = 20;

	//bool lifterHallEffectTopPast;
	//bool lifterHallEffectBottom;
	//bool bHover;//hovers by hitting the higher hall effect
	bool bHoverEnabled = true;	//able to hover, should start true
	bool bHoverWhileStopped = false; //should hover when stopped
	//bool bHovering;		//actually hovering
	//bool bLowerHover;
	bool bGoingUp = false;
	bool bGoingDown = false;
	int iToteLoad = 0;


	void OnStateChange();
	void Run();
	bool LifterCurrentLimitDrive(float);
};

#endif			//CANLIFTER_H
