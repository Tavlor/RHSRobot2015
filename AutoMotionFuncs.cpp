/*
 * Copyright (c) 2014 David Shafer. All rights reserved.
 * For Rockwall Robotics, FRC Team 1296: Full Metal Jackets
 * Licensed under a Creative Commons Attribution-ShareAlike 3.0 Unported License.
 */
#include <WPILib.h>
#include <string.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <WPILib.h>

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

//Local
#include "Autonomous.h"
#include "AutoParser.h"

using namespace std;

bool Autonomous::Move(char *pCurrLinePos) {
	char *pToken;
	float fLeft;
	float fRight;

	// parse remainder of line to get length to move

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fLeft = atof(pToken);

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
	fRight = atof(pToken);

	if((fabs(fLeft) > MAX_VELOCITY_PARAM)
			|| (fabs(fRight) > MAX_VELOCITY_PARAM)) {
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

	 return(CommandResponse(DRIVETRAIN_QUEUE));
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

bool Autonomous::Turn(char *pCurrLinePos)
{
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

	 return(CommandResponse(DRIVETRAIN_QUEUE));
}
