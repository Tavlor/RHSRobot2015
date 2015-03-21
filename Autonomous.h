/** \file
 * The AutonomousBase component class handles basic autonomous functionality.
 */

#ifndef AUTONOMOUS_BASE_H
#define AUTONOMOUS_BASE_H

//Robot
#include <string>

#include "WPILib.h"

#include "ComponentBase.h" //For the ComponentBase class
#include "RobotParams.h" //For various robot parameters

const int AUTONOMOUS_SCRIPT_LINES = 64;
const int AUTONOMOUS_CHECKLIST_LINES = 64;
const char* const AUTONOMOUS_SCRIPT_FILEPATH = "/home/lvuser/RhsScript.txt";

//from 2014
const float MAX_VELOCITY_PARAM = 1.0;
const float MAX_DISTANCE_PARAM = 100.0;

class Autonomous : public ComponentBase
{
public:
	Autonomous();
	~Autonomous();
	void DoScript();

	static void *StartTask(void *pThis)
	{
		((Autonomous *)pThis)->DoWork();
		return(NULL);
	}

	static void *StartScript(void *pThis)
	{
		((Autonomous *)pThis)->DoScript();
		return(NULL);
	}

protected:
	bool Evaluate(std::string statement);	//Evaluates an autonomous script statement
	RobotMessage Message;
	bool bInAutoMode;
	bool bPauseAutoMode;

private:
	std::string script[AUTONOMOUS_SCRIPT_LINES];	//Autonomous script
	int lineNumber;	//Current line number
	Task *pScript;
	bool bReceivedCommandResponse;
	MessageCommand ReceivedCommand;

	bool Begin(char *);
	bool End(char *);
	bool Move(char *);
	bool Stop(char *);
	bool MeasuredMove(char *);
	bool TimedMove(char *);
	bool Turn(char *);
	bool SeekTote(char *);
	bool Straight(char *);
	bool CubeAuto(char *);

	bool CommandResponse(const char *szQueueName);
	bool CommandNoResponse(const char *szQueueName);

	void Init();
	void OnStateChange();
	void Run();
	bool LoadScriptFile();
};

#endif //AUTONOMOUS_BASE_H
