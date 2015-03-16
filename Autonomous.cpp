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

	if (read(iPipeRcv, (char*) &Message, sizeof(RobotMessage)) <= 0)
	{
		bReturn = false;
	}
	close(iPipeRcv);

	if (Message.command != COMMAND_SYSTEM_OK)
	{
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
	Message.command = COMMAND_DRIVETRAIN_DRIVE_TANK;
	Message.params.tankDrive.left = fLeft;
	Message.params.tankDrive.right = fRight;

	return (CommandNoResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::Stop(char *pCurrLinePos) {
	Message.command = COMMAND_DRIVETRAIN_DRIVE_TANK;
	Message.params.tankDrive.left = 0.0;
	Message.params.tankDrive.right = 0.0;

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
	float fSpeed;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fSpeed = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fAngle = atof(pToken);

	// send the message to the drive train

	Message.command = COMMAND_DRIVETRAIN_TURN;
	Message.params.autonomous.turnSpeed = fSpeed;
	Message.params.autonomous.turnAngle = fAngle;

	return (CommandResponse(DRIVETRAIN_QUEUE));
}

bool Autonomous::CubeAuto(char *pCurrLinePos){
	Message.command = COMMAND_CUBEAUTOCYCLE_START;
	return(CommandNoResponse(CUBE_QUEUE));
}

