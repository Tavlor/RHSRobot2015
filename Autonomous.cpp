/***
This file contains our autonomous algorithms.  It should detect if we are in 
autonomous mode or not, select an algorithm based upon switch settings at
the driver station and implement the behaviours till autonomous mode ends.
 ***/

#include "Autonomous.h"
#include "WPILib.h"

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"
#include "AutoParser.h"


extern "C" {
}

Autonomous::Autonomous() : AutonomousBase()
{
	//previously empty, this may belong in AutonomousBase()
	pTask = new Task(AUTONOMOUS_TASKNAME, (FUNCPTR) &Autonomous::StartTask,
			AUTONOMOUS_PRIORITY, AUTONOMOUS_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int)this);
}

Autonomous::~Autonomous()
{
	delete(pTask);
}



