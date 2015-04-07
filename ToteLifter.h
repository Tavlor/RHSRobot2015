/*
 * ToteLifter.h
 *
 *  Created on: Mar 24, 2015
 *      Author: Jacob
 */

#ifndef TOTELIFTER_H
#define TOTELIFTER_H

//Robot
#include "WPILib.h"
#include "ComponentBase.h"			//For ComponentBase class

class ToteLifter : public ComponentBase
{
public:
	ToteLifter();
	virtual ~ToteLifter();
	static void *StartTask(void *pThis)
	{
		((ToteLifter *)pThis)->DoWork();
		return(NULL);
	}

private:

	CANTalon *toteMotor;
	Timer *pExtendTimer;
	Timer *pRetractTimer;
	bool bExtending;
	bool bRetracting;

	const float fToteExtend = 1.00;
	const float fToteRetract = -1.00;
	const float fToteStop = 0.0;
	const float fExtendTime = 1.00;
	const float fRetractTime = 1.00;

	void OnStateChange();
	void Run();
};

#endif /* TOTELIFTER_H */
