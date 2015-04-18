/** \file
 * Definitions of class to control claw on the pallet jack.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the pallet jack's claw.
 */
/**
 * A class to control the movements of the claw. It implements a current limit to keep the motor from burning out.
 */
#ifndef CLAW_H
#define CLAW_H

//Robot
#include "WPILib.h"


#include "ComponentBase.h"			//For ComponentBase class

class Claw : public ComponentBase
{
public:
	Claw();
	virtual ~Claw();
	static void *StartTask(void *pThis)
	{
		((Claw *)pThis)->DoWork();
		return(NULL);
	}

private:

	CANTalon *clawMotor;
	Timer *pSafetyTimer;
	Timer *pClawTimer;


	/*
	 *	CLAW  (using bag, not CIM)
	 *	Open: +
	 *	Close: -
	 */
	const float fClawOpen = .50;
	const float fClawClose = -.40;
	const float fClawStop = 0.0;
	const float fClawActionTimeLimit = 0.5;
	const float fClawMotorCurrentMax = 8.0;

	///claw is open or closed
	//bool bClawOpen;

	void OnStateChange();
	void Run();
};

#endif			//CLAW_H
