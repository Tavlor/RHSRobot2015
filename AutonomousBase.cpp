/** \file
 * The AutonomousBase component class handles basic autonomous functionallity.
 */

//Local
#include "AutonomousBase.h"

#include <iostream>
#include <fstream>
#include <string>

#include "ComponentBase.h"
#include "RobotParams.h"

using namespace std;

AutonomousBase::AutonomousBase()
: ComponentBase(AUTONOMOUS_TASKNAME, AUTONOMOUS_QUEUE, AUTONOMOUS_PRIORITY)
{
	lineNumber = 0;
	bInAutoMode = false;
	iExecTaskID = -1;
	LoadScriptFile();
}

AutonomousBase::~AutonomousBase()	//Destructor
{
}

void AutonomousBase::Init()	//Initializes the autonomous component
{
	LoadScriptFile();
}
 
void AutonomousBase::OnStateChange()	//Handles state changes
{
	if(localMessage.command == COMMAND_ROBOT_STATE_AUTONOMOUS)
	{
		pTask = new Task(AUTOEXEC_TASKNAME, (FUNCPTR) &AutonomousBase::StartTask,
				AUTOEXEC_PRIORITY, AUTOEXEC_STACKSIZE);
		wpi_assert(pTask);
		pTask->Start((int)this);
	}	
	else if((localMessage.command == COMMAND_ROBOT_STATE_TELEOPERATED) ||
			(localMessage.command == COMMAND_ROBOT_STATE_DISABLED))
	{
		delete(pTask);
	}
}

void AutonomousBase::Run()	//Autonomous logic
{
	switch(localMessage.command)
	{
		case COMMAND_AUTONOMOUS_RUN:
			if(lineNumber < AUTONOMOUS_SCRIPT_LINES)
			{
				//printf("%i: \n",lineNumber);
				if(script[lineNumber].empty() == false)
				{
					Evaluate(script[lineNumber]);
					++lineNumber;
				}
			}
			break;
		case COMMAND_CHECKLIST_RUN:
			if(lineNumber < AUTONOMOUS_CHECKLIST_LINES)
			{
				Evaluate(script[lineNumber]);
				++lineNumber;
			}
			break;

		default:
			break;
	}
}

void AutonomousBase::LoadScriptFile()
{
	printf("Auto Script Filepath: [%s]\n",AUTONOMOUS_SCRIPT_FILEPATH);
	ifstream scriptStream;
	scriptStream.open(AUTONOMOUS_SCRIPT_FILEPATH);
	
	if(scriptStream.is_open())//not working
	{
		if(bInAutoMode == false)
		{
			for(int i = 0; i < AUTONOMOUS_SCRIPT_LINES; ++i)
			{
				if(!scriptStream.eof())
				{
					getline(scriptStream, script[i]);
					cout << script[i] << endl;		
				}
				else
				{
					script[i].clear();
				}
			}
			printf("Autonomous script loaded\n");
		}
		else
		{
			printf("Attempt to read script in auto mode\n");
		}

		scriptStream.close();
	}	
	else
	{
		printf("No auto file found\n");
	}
}

void AutonomousBase::DoWork()
{
	lineNumber = 0;
	
	while(lineNumber < AUTONOMOUS_SCRIPT_LINES)
	{
		if(script[lineNumber].empty() == false)
		{
			Evaluate(script[lineNumber]);
			++lineNumber;
		}
	}
}
