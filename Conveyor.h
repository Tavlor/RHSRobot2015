/** \file
 * Definitions of class to control conveyor on the pallet jack.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the devices used to control the pallet jack's conveyor.
 */

#ifndef CONVEYOR_H
#define CONVEYOR_H
///Controls the conveyor on the pallet jack.

//Robot
#include "WPILib.h"

#include "ComponentBase.h"			//For ComponentBase class
#include "RobotMessage.h"

class Conveyor: public ComponentBase
{
public:
	Conveyor();
	virtual ~Conveyor();
	static void *StartTask(void *pThis)
	{
		((Conveyor*)pThis)->DoWork();
		return(NULL);
	}

	bool RevLimitSwitchClosed();
	bool FwdLimitSwitchClosed();

private:
	/*MOTOR VALUES with CAN
	 * + backwards (towards back)
	 * - forwards (towards front)
	 *
	 * revLimit - at the front
	 * fwdLimit - at the back
	 */

	CANTalon *conveyorMotor;
	Timer *pAutoTimer;
	const float fConveyorSpeed = 0.75;//1.0;
	const float fConveyorSpeedBack = 0.5;
	const float fConveyorSpeedFwd = -0.5;
	//dial these down as you go - but start fast.
	const float fLoadSpeed = .75;
	const float fShiftSpeed = 0.3;//good to keep low.
	const float fPushSpeed = 0.1;
	const float fDepositSpeed = 1.0;
	bool bBackStopEnable;
	MessageCommand responseCommand;

	void OnStateChange();
	void Run();
};

#endif			//CONVEYOR_H
