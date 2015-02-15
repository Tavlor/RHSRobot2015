/***
 This file contains our autonomous algorithms.  It should detect if we are in
 autonomous mode or not, select an algorithm based upon switch settings at
 the driver station and implement the behaviours till autonomous mode ends.
 ***/

#include "Autonomous.h"
#include "WPILib.h"
#include <unistd.h>

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"
#include "AutoParser.h"

extern "C" {
}

Autonomous::Autonomous() :
		AutonomousBase() {
	//previously empty, this may belong in AutonomousBase()
	pTask = new Task(AUTONOMOUS_TASKNAME, (FUNCPTR) &Autonomous::StartTask,
			AUTONOMOUS_PRIORITY, AUTONOMOUS_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int) this);
}

Autonomous::~Autonomous() {
	delete (pTask);
}

bool Autonomous::CommandResponse(const char *szQueueName) {
	int iPipeXmt;
	int iPipeRcv;
	bool bReturn = true;

	iPipeXmt = open(szQueueName, O_WRONLY);
	wpi_assert(iPipeXmt > 0);

	Message.replyQ = AUTOPARSER_QUEUE;
	write(iPipeXmt, (char*) &Message, sizeof(RobotMessage));
	close(iPipeXmt);

	// wait for a response
	iPipeRcv = open(AUTOPARSER_QUEUE, O_WRONLY);
	wpi_assert(iPipeRcv > 0);

	if(read(iPipeRcv, (char*) &Message, sizeof(RobotMessage)) <= 0) {
		bReturn = false;
	}
	close(iPipeRcv);

	if(Message.command != COMMAND_SYSTEM_OK) {
		bReturn = false;
	}
	return bReturn;
}

bool Autonomous::CommandNoResponse(const char *szQueueName) {
	int iPipeXmt;

	iPipeXmt = open(szQueueName, O_WRONLY);
	wpi_assert(iPipeXmt > 0);

	write(iPipeXmt, (char*) &Message, sizeof(RobotMessage));
	close(iPipeXmt);
	return (true);
}

