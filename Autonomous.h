
#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include <iostream>
#include <fstream>
#include <string>

#include "WPILib.h"
#include "RobotMessage.h"
#include "RobotParams.h"
#include "AutonomousBase.h"

class Autonomous : public AutonomousBase
{
	public:
		Autonomous();
		virtual ~Autonomous();
		static void *StartTask(void *pThis)
		{
			((Autonomous *)pThis)->DoWork();
			return(NULL);
		}

	private:
		void Evaluate(std::string rStatement);
};

#endif // AUTONOMOUS_H

