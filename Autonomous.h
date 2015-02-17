/** \file
 * Class for our autonomous behaviours
 */

#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include <iostream>
#include <fstream>
#include <string>

#include "WPILib.h"
#include "RobotMessage.h"
#include "RobotParams.h"
#include "AutonomousBase.h"

//from 2014
const float MAX_VELOCITY_PARAM = 1.0;
const float MAX_DISTANCE_PARAM = 100.0;
#define WAIT_FOR_AUTOREPLYMSG  (sysClkRateGet() * 10)

class Autonomous: public AutonomousBase {
public:
	Autonomous();
	virtual ~Autonomous();
	static void *StartTask(void *pThis) {
		((Autonomous *) pThis)->DoWork();
		return (NULL);
	}

private:
	void Evaluate(std::string rStatement);

	//from 2014
	double fCurrAccX;
	double fCurrAccY;
	double fVelocityX;
	double fVelocityY;
	double fLastAccX;
	double fLastAccY;
	double fDistanceX;
	double fDistanceY;
	float fGyroDrift;
	bool bCalibrated;
	int iMotionTaskID;
	int iPiTaskID;

	bool Move(char *);
	bool Stop(char *);
	bool MeasuredMove(char *);
	bool TimedMove(char *);
	bool Turn(char *);

	bool CommandResponse(const char *szQueueName);
	bool CommandNoResponse(const char *szQueueName);

};

#endif // AUTONOMOUS_H

