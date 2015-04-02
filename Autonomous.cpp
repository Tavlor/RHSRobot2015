/** \file
 * Class for our autonomous behaviours
 *
 *  This file contains our autonomous algorithms.  It should detect if we are in
 *  autonomous mode or not, select an algorithm based upon switch settings at
 *  the driver station and implement the behaviours till autonomous mode ends.
 */

#include "Autonomous.h"
#include "WPILib.h"
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"
#include "AutoParser.h"

using namespace std;

extern "C" {
}

bool Autonomous::CommandResponse(const char *szQueueName) {
	int iPipeXmt;
	bool bReturn = true;

	iPipeXmt = open(szQueueName, O_WRONLY);
	wpi_assert(iPipeXmt > 0);

	Message.replyQ = AUTONOMOUS_QUEUE;
	write(iPipeXmt, (char*) &Message, sizeof(RobotMessage));
	close(iPipeXmt);

	bReceivedCommandResponse = false;

	while (!bReceivedCommandResponse)
	{
		//purposefully empty
	}
	printf("Response received\n");

	if (ReceivedCommand == COMMAND_AUTONOMOUS_RESPONSE_OK)
	{
		SmartDashboard::PutString("Auto Status","auto ok");
		bReturn = true;
	}
	else if (ReceivedCommand == COMMAND_AUTONOMOUS_RESPONSE_ERROR)
	{
		SmartDashboard::PutString("Auto Status","auto died, tell programmers");
		//TODO: if auto quits abnormally, signal the drivers via SD or something
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

bool Autonomous::Begin(char *pCurrLinePos)
{
	//tell all the components who may need to know that auto is beginning
	Message.command = COMMAND_AUTONOMOUS_RUN;
	return (CommandNoResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::End(char *pCurrLinePos)
{
	//tell all the components who may need to know that auto is beginning
	Message.command = COMMAND_AUTONOMOUS_COMPLETE;
	CommandNoResponse(DRIVETRAIN_QUEUE);
	return (true);
}


bool Autonomous::Move(char *pCurrLinePos) {
	char *pToken;
	float fLeft;
	float fRight;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fLeft = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fRight = atof(pToken);

	if ((fabs(fLeft) > MAX_VELOCITY_PARAM)
			|| (fabs(fRight) > MAX_VELOCITY_PARAM))
	{
		return (false);
	}
	Message.command = COMMAND_DRIVETRAIN_AUTO_MOVE;
	Message.params.tankDrive.left = fLeft;
	Message.params.tankDrive.right = fRight;

	return (CommandNoResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::Stop(char *pCurrLinePos) {
	//tell those who need to know that the autonomous behavior is over - reset variables
	Message.command = COMMAND_DRIVETRAIN_STOP;
	return (CommandNoResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::MeasuredMove(char *pCurrLinePos) {

	char *pToken;
	float fDistance;
	float fSpeed;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fSpeed = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fDistance = atof(pToken);

	// send the message to the drive train

	Message.command = COMMAND_DRIVETRAIN_DRIVE_STRAIGHT;
	Message.params.autonomous.driveSpeed = fSpeed;
	Message.params.autonomous.driveDistance = fDistance;

	return (CommandResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::TimedMove(char *pCurrLinePos) {
	/*
	 char *pToken;
	 float fSpeed;
	 float fTime;

	 // parse remainder of line to get length to move

	 pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	 fSpeed = atof(pToken);

	 pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	 fTime = atof(pToken);

	 // send the message to the drive train

	 Message.command = COMMAND_DRIVETRAIN_DRIVE_TIMEDTANK;
	 Message.params.timedDrive.speed = fSpeed;
	 Message.params.timedDrive.time = fTime;

	 return(CommandResponse(DRIVETRAIN_QUEUE));
	 */
	return false;
}

bool Autonomous::Turn(char *pCurrLinePos) {
	char *pToken;
	float fAngle;
	float fTimeout;

	// parse remainder of line to get target angle and timeout
	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fAngle = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fTimeout = atof(pToken);

	// send the message to the drive train
	Message.command = COMMAND_DRIVETRAIN_TURN;
	Message.params.autonomous.turnAngle = fAngle;
	Message.params.autonomous.timeout = fTimeout;
	return (CommandResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::SeekTote(char *pCurrLinePos) {
	char *pToken;
	float fTimein;
	float fTimeout;

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fTimein = atof(pToken);
	// parse remainder of line to get timeout
	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fTimeout = atof(pToken);

	// send the message to the drive train
	Message.command = COMMAND_DRIVETRAIN_SEEK_TOTE;
	Message.params.autonomous.timeout = fTimeout;
	Message.params.autonomous.timein = fTimein;
	return (CommandResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::Straight(char *pCurrLinePos) {
	char *pToken;
	float fSpeed;
	float fTime;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fSpeed = atof(pToken);
	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fTime = atof(pToken);

	// send the message to the drive train
	Message.command = COMMAND_DRIVETRAIN_DRIVE_STRAIGHT;
	Message.params.autonomous.driveSpeed = fSpeed;
	Message.params.autonomous.driveTime = fTime;
	return (CommandResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::CubeAuto(char *pCurrLinePos){
	Message.command = COMMAND_CUBEAUTOCYCLE_START;
	return(CommandNoResponse(CUBE_QUEUE));
}

