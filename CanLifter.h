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

#define TOPHALLEFFECT		lifterMotor->IsRevLimitSwitchClosed()
#define BOTTOMHALLEFFECT	lifterMotor->IsFwdLimitSwitchClosed()

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
	//Counter *midDetect;
	Timer *pSafetyTimer;
	//Timer *pAutoTimer;IN COMPONENT BASE


	/* CANLIFTER  (using bag, not CIM)
	 * 	Raise: -
	 *	Lower: +
	 */
	const float fLifterUpMult = -1.0;
	const float fLifterDownMult = 1.0;

	const float fLifterRaise = -1.0;
	const float fLifterLower = 1.0;
	const float fLifterHover = -.5;

	const float fLifterHoverNoTotes = -.2;
	const float fLifterHoverOneTotes = -.20;
	const float fLifterHoverTwoTotes = -.25;
	const float fLifterHoverThreeTotes = -.30;

	const float fLifterLiftNoTotes = -0.50;
	const float fLifterLiftOneTotes = -1.0;
	const float fLifterLiftTwoTotes = -1.0;
	const float fLifterLiftThreeTotes = -0.35;

	const float fLifterLowerNoTotes = 0.50;
	const float fLifterLowerOneTotes = 0.50;
	const float fLifterLowerTwoTotes = 0.50;
	const float fLifterLowerThreeTotes = 0.50;

	const float fLifterStop = -.2;///should hover with a can
	const float fLifterMotorCurrentMax = 30;
	const float fLifterMotorCurrentMaxOneCan = 20;

	//bool lifterHallEffectTopPast;
	//bool lifterHallEffectBottom;
	//bool bHover;//hovers by hitting the higher hall effect
	bool bHoverEnabled;	//able to hover
	bool bHovering;		//actually hovering
	bool bLowerHover;
	bool bGoingUp;
	bool bGoingDown;
	int iToteLoad;


	void OnStateChange();
	void Run();
	bool LifterCurrentLimitDrive(float);
};

#endif			//CANLIFTER_H
