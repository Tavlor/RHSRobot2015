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
	bool GetHallEffectTop();
	bool GetHallEffectMiddle();
	bool GetHallEffectBottom();


private:

	CANTalon *lifterMotor;
	DigitalInput *midHallEffect;
	Timer *pSafetyTimer;
	//Timer *pUpdateTimer;


	/* CANLIFTER  (using bag, not CIM)
	 * 	Raise: +
	 *	Lower: -
	 */
	const float fLifterRaise = 1.0;		//not needed
	const float fLifterLower = -1.0;	//not needed
	const float fLifterHover = .15;
	const float fLifterStop = 0.0;

	bool lifterHallEffectBottom;
	bool lifterHallEffectTop;
	bool bHover;					//should the lift keep itself at a constant height?

	void OnStateChange();
	void Run();
};

#endif			//CANLIFTER_H
