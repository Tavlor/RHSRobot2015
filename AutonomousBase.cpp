/** \file
 * The AutonomousBase component class handles basic autonomous functionallity.
 */

//Local
#include "Autonomous.h"

#include <iostream>
#include <fstream>
#include <string>

#include "ComponentBase.h"
#include "RobotParams.h"

using namespace std;

Autonomous::Autonomous()
: ComponentBase(AUTONOMOUS_TASKNAME, AUTONOMOUS_QUEUE, AUTONOMOUS_PRIORITY)
{
	lineNumber = 0;
	bInAutoMode = false;

	pTask = new Task(AUTONOMOUS_TASKNAME, (FUNCPTR) &Autonomous::StartTask,
		AUTONOMOUS_PRIORITY, AUTONOMOUS_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int)this);

	pScript = new Task(AUTOEXEC_TASKNAME, (FUNCPTR) &Autonomous::StartScript,
			AUTOEXEC_PRIORITY, AUTOEXEC_STACKSIZE);
	wpi_assert(pScript);
	pScript->Start((int)this);
}

Autonomous::~Autonomous()	//Destructor
{
	delete(pTask);
	delete(pScript);
}

void Autonomous::Init()	//Initializes the autonomous component
{
}
 
void Autonomous::OnStateChange()	//Handles state changes
{
	// to handle unexpected state changes before the auto script finishes (like in OKC last year)
	// we will leave the script running - hope this works!

	if(localMessage.command == COMMAND_ROBOT_STATE_AUTONOMOUS)
	{
		bPauseAutoMode = false;
		bInAutoMode = true;
	}
	else if(localMessage.command == COMMAND_ROBOT_STATE_TELEOPERATED)
	{
		bPauseAutoMode = true;
	}
	else if(localMessage.command == COMMAND_ROBOT_STATE_TELEOPERATED)
	{
		bPauseAutoMode = true;
	}
}

void Autonomous::Run()
{
	switch(localMessage.command)
	{
		case COMMAND_AUTONOMOUS_RUN:
			break;

		case COMMAND_CHECKLIST_RUN:
			break;

		default:
			break;
	}
}

bool Autonomous::LoadScriptFile()
{
	bool bReturn = true;
	//printf("Auto Script Filepath: [%s]\n", AUTONOMOUS_SCRIPT_FILEPATH);
	ifstream scriptStream;
	scriptStream.open(AUTONOMOUS_SCRIPT_FILEPATH);
	
	if(scriptStream.is_open())//not working
	{
		for(int i = 0; i < AUTONOMOUS_SCRIPT_LINES; ++i)
		{
			if(!scriptStream.eof())
			{
				getline(scriptStream, script[i]);
				//cout << script[i] << endl;
			}
			else
			{
				script[i].clear();
			}
		}

		//printf("Autonomous script loaded\n");
		scriptStream.close();
	}	
	else
	{
		//printf("No auto file found\n");
		bReturn = false;
	}

	return(bReturn);
}

void Autonomous::DoScript()
{
	printf("DoScript\n");
	
	while(true)
	{
		lineNumber = 0;
		SmartDashboard::PutNumber("Script Line Number", lineNumber);

		if(LoadScriptFile() == false)
		{
			// wait a little and try again, really only useful if when practicing

			SmartDashboard::PutBoolean("Script File Loaded", false);
			Wait(1.0);
		}
		else
		{
			SmartDashboard::PutBoolean("Script File Loaded", true);

			// if there is a script we will execute it some hell or high water!

			while(bInAutoMode)
			{
				SmartDashboard::PutNumber("Script Line Number", lineNumber);

				if (lineNumber < AUTONOMOUS_SCRIPT_LINES)
				{
					// can we have empty lines?  at the end I guess

					if(script[lineNumber].empty() == false)
					{
						// handle pausing in the Evaluate method

						SmartDashboard::PutString("Script Line", script[lineNumber].c_str());

						if(Evaluate(script[lineNumber]))
						{
							break;
						}
					}

					lineNumber++;
				}
				else
				{
					break;
				}
			}

			bInAutoMode = false;
			Wait(5.0);
		}
	}

	bInAutoMode = false;
}
