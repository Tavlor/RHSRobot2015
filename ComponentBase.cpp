/** \file
 * Component base class implementation.
 *
 * In the RhsRobot Framework, each physical subsystem has a corresponding component class.
 * These component classes should inherit the ComponentBase class for access to functions that
 * all components use.
 */

#include "ComponentBase.h"
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/types.h>

//Local

//Robot
class RhsRobot;
#include "RobotMessage.h"

ComponentBase::ComponentBase(const char* componentName, const char *queueName, int priority)
{	
	iLoop = 0;
	iPipeRcv = -1;
	iPipeXmt = -1;
	pTask = NULL;

	pRemoteUpdateTimer = new Timer();
	pRemoteUpdateTimer->Start();

	pAutoTimer = new Timer();
	pAutoTimer->Start();

	pDebugTimer = new Timer();
	pDebugTimer->Start();

	mkfifo(queueName, 0666);
	queueLocal = queueName;
	//printf("COMPONENT: %s\n",componentName); //Added by Talyor for debugging
}

void ComponentBase::SendMessage(RobotMessage* robotMessage)
{
	RobotMessage message = *robotMessage;

	if(iPipeXmt < 0)
	{
		iPipeXmt = open(queueLocal.c_str(), O_WRONLY);
		//fcntl(iPipeXmt, F_SETFL, fcntl(iPipeXmt, F_GETFL) | O_NONBLOCK);
		assert(iPipeXmt > 0);
	}

	write(iPipeXmt, (char*)&message, sizeof(RobotMessage));
}

void ComponentBase::ReceiveMessage()			//Receives a message and copies it into localMessage
{
	fd_set selectSet;
	struct timeval timeout;

	if(iPipeRcv < 0)
	{
		//printf("ComponentBase opening pipe\n");
		iPipeRcv = open(queueLocal.c_str(), O_RDONLY);
		assert(iPipeRcv > 0);
	}

	FD_ZERO(&selectSet);
	FD_SET(iPipeRcv, &selectSet);

	timeout.tv_sec = 0;
	timeout.tv_usec = 40000;

	if(select(iPipeRcv + 1, &selectSet, NULL, NULL, &timeout) == 0)
	{
		localMessage.command = COMMAND_SYSTEM_MSGTIMEOUT;
	}
	else
	{
		read(iPipeRcv, (char*)&localMessage, sizeof(RobotMessage));
	}
}

void ComponentBase::ClearMessages(void)
{
	RobotMessage eatMessage;
	
	// eat all the messages in the queue
	
	fcntl(iPipeRcv, F_SETFL, O_NONBLOCK);

	while(read(iPipeRcv, (char*)&eatMessage, sizeof(RobotMessage)) > 0)
	{
		// intentionally empty
	}

	fcntl(iPipeRcv, F_SETFL, 0);

	// make sure the localMessage is innocuous
	
	localMessage.command = COMMAND_SYSTEM_MSGTIMEOUT;
}

void ComponentBase::DoWork()
{
	while(true)
	{
		ReceiveMessage();		//Receives a message and copies it into localMessage

		if(localMessage.command == COMMAND_ROBOT_STATE_DISABLED ||			//Tests for state change messages
				localMessage.command == COMMAND_ROBOT_STATE_AUTONOMOUS ||
				localMessage.command == COMMAND_ROBOT_STATE_TELEOPERATED ||
				localMessage.command == COMMAND_ROBOT_STATE_TEST ||
				localMessage.command == COMMAND_ROBOT_STATE_UNKNOWN)
		{
			OnStateChange();			//Handles state changes
		}

		Run();			//Component logic
		//if(ISAUTO) { AutoBehavior(); } //TODO: add this after world's for easier auto coding
		//AutoBehavior is where the actual auto stuff is called - it should be periodic rather than stop up the thread
		//It should be structured as a state machine; Run will change the state.
		//if(pRemoteUpdateTimer->Get() > fUpdateDelay) { pRemoteUpdateTimer->Reset(); SmartDashboardUpdate(); } //TODO: add this after world's
		lastCommand = localMessage.command;
		iLoop++;
	}
}
void ComponentBase::SendCommandResponse(MessageCommand command)
{
	RobotMessage replyMessage;
		replyMessage.command = command;
		//Send a message back to auto to tell it that code is done.
		int iPipeXmt = open(localMessage.replyQ, O_WRONLY);
		assert(iPipeXmt > 0);

		write(iPipeXmt, (char*) &replyMessage, sizeof(RobotMessage));
		close(iPipeXmt);
}
