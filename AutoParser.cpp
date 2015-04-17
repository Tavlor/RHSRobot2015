/** \file
 *  Autonomous script parser
 */

#include "AutoParser.h"
#include <string.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <string>

#include <WPILib.h>

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"
#include "Autonomous.h"

using namespace std;

const char *szTokens[] = {
		"MODE",
		"BEGIN",
		"END",
		"DELAY",			//!<(seconds)
		"MOVE",				//!<(left speed) (right speed)
		"MMOVE",			//!<(speed) (distance:inches) (timeout)
		"TURN",				//!<(degrees) (timeout)
		"STRAIGHT",			//!<(speed) (duration)
		"CLAWOPEN",			//!<
		"CLAWCLOSE",		//!<
		//LIFTER
		"CLAWTOTOP",		//!<
		"CLAWTOBOTTOM",		//!<
		//"CANUP",			//!<
		"RAISECANTOLOMID",	//!<
		"LOWERCANTOHIMID",	//!<
		"STACKUP",			//!<
		"STACKDOWN",		//!<
		"STARTSTACKUP",		//!<
		//CONVEYOR/DRIVETRAIN
		"FRONTLOADTOTE",	//!<(timeout)
		"BACKLOADTOTE",		//!<(timeout)
		"FRONTSEEKTOTE",	//!<(drive speed) (timeout)
		"BACKSEEKTOTE",		//!<(drive speed) (timeout)
		//DRIVETRAIN
		"STARTDRIVEFWD",	//!<(drive speed)
		"STARTDRIVEBCK",	//!<(drive speed)
		"STOPDRIVE",
		//CONVEYOR
		"WAITFRONTBEAM",
		"WAITBACKBEAM",
		"DEPOSITTOTESBACK",	//!<
		"TOTESHIFTFWD",		//!<
		"TOTESHIFTBCK",		//!<
		"TOTEPUSHBCK",		//!<
		"CANARMOPEN",		//!<
		"CANARMCLOSE",		//!<
		//Old commands from past auto attemts
		"SEEKTOTE",			//!<
		"STARTTOTEUP",		//!<
		"TOTEEXTEND",		//!<
		"TOTERETRACT",		//!<
		"CUBEAUTO",			//!<
		"CLICKERUP",		//!<
		"CLICKERDOWN",		//!<
		"NOP" };
//TODO: add START and FINISH, which send messages to all components
// (Begin and End are doing this now, but they shouldn't)

bool Autonomous::Evaluate(std::string rStatement) {
	char *pToken;
	char *pCurrLinePos;
	int iCommand;
	float fParam1;
	int iParam1;
	bool bReturn = false; ///setting this to true WILL cause auto parsing to quit!
	string rStatus;

	if(rStatement.empty()) {
		printf("statement is empty");
		return (bReturn);
	}

	// process the autonomous motion

	pCurrLinePos = (char *) rStatement.c_str();

	if(*pCurrLinePos == sComment) {
		return (bReturn);
	}

	printf("%s\n", rStatement.c_str());

	// find first token

	pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

	// which command are we to execute??
	// this can be (easily) be made much faster... any student want to improve on this?

	for(iCommand = AUTO_TOKEN_MODE; iCommand < AUTO_TOKEN_LAST; iCommand++) {
		//printf("comparing %s to %s\n", pToken, szTokens[iCommand]);
		if(!strncmp(pToken, szTokens[iCommand], strlen(szTokens[iCommand]))) {
			break;
		}
	}

	if(iCommand == AUTO_TOKEN_LAST) {
		// no valid token found
		rStatus.append("no tokens - check script spelling");
		printf("%s\n", rStatus.c_str());
		return (bReturn);
	}

	// if we are paused wait here before executing a real command

	while(bPauseAutoMode)
	{
		Wait(0.02);
	}

	// execute the proper command

	switch (iCommand)
	{
	case AUTO_TOKEN_BEGIN:
		Begin(pCurrLinePos);
		rStatus.append("start");
		break;

	case AUTO_TOKEN_END:
		End(pCurrLinePos);
		rStatus.append("done");
		bReturn = true;
		break;

	case AUTO_TOKEN_DELAY:
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

		if (pToken == NULL)
		{
			rStatus.append("missing parameter");
		}
		else
		{
			fParam1 = atof(pToken);
			rStatus.append("wait");

			Delay(fParam1);
		}
		break;

	case AUTO_TOKEN_MOVE:
		if (!Move(pCurrLinePos))
		{
			rStatus.append("move error");
		}
		else
		{
			rStatus.append("move");
		}
		break;

	case AUTO_TOKEN_MMOVE:
		if (!MeasuredMove(pCurrLinePos))
		{
			rStatus.append("move error");
		}
		else
		{
			rStatus.append("move");
		}
		break;

	case AUTO_TOKEN_TURN:
		if (!Turn(pCurrLinePos))
		{
			rStatus.append("turn error");
		}
		else
		{
			rStatus.append("turn");
		}
		break;

	case AUTO_TOKEN_STRAIGHT:
		if (!Straight(pCurrLinePos))
		{
			rStatus.append("straight error");
		}
		else
		{
			rStatus.append("straight");
		}
		break;

	case AUTO_TOKEN_RAISE_TOTES:
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		iParam1 = atoi(pToken);

		Message.command = COMMAND_CANLIFTER_RAISE_TOTES;
		Message.params.canLifterParams.iNumTotes = iParam1;
		bReturn = !CommandResponse(CANLIFTER_QUEUE);
		//the conveyor should've pushed the totes. Stop it.
		Message.command = COMMAND_CONVEYOR_STOP;
		bReturn = !CommandNoResponse(CONVEYOR_QUEUE);
		break;

	case AUTO_TOKEN_LOWER_TOTES:
		/*pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		iParam1 = atoi(pToken);*/
		Message.command = COMMAND_CANLIFTER_LOWER_TOTES;
		//Message.params.canLifterParams.iNumTotes = iParam1;
		bReturn = !CommandResponse(CANLIFTER_QUEUE);
		break;

	case AUTO_TOKEN_START_RAISE_TOTES:
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		iParam1 = atoi(pToken);

		Message.command = COMMAND_CANLIFTER_START_RAISE_TOTES;
		Message.params.canLifterParams.iNumTotes = iParam1;
		bReturn = !CommandNoResponse(CANLIFTER_QUEUE);
		break;

	case AUTO_TOKEN_CLAW_TO_TOP:
		Message.command = COMMAND_CANLIFTER_CLAW_TO_TOP;
		bReturn = !CommandResponse(CANLIFTER_QUEUE);
		break;

	case AUTO_TOKEN_CLAW_TO_BOTTOM:
		Message.command = COMMAND_CANLIFTER_CLAW_TO_TOP;
		bReturn = !CommandResponse(CANLIFTER_QUEUE);
		break;

	case AUTO_TOKEN_RAISE_CAN_LOMID:
		Message.command = COMMAND_CANLIFTER_RAISE_LOMID;
		bReturn = !CommandNoResponse(CANLIFTER_QUEUE);
		break;

	case AUTO_TOKEN_LOWER_CAN_HIMID:
		Message.command = COMMAND_CANLIFTER_LOWER_HIMID;
		bReturn = !CommandNoResponse(CANLIFTER_QUEUE);
		break;

	case AUTO_TOKEN_CLAW_OPEN:
		Message.command = COMMAND_CLAW_OPEN;
		bReturn = !CommandNoResponse(CLAW_QUEUE);
		break;

	case AUTO_TOKEN_CLAW_CLOSE:
		Message.command = COMMAND_CLAW_CLOSE;
		bReturn = !CommandNoResponse(CLAW_QUEUE);
		break;

	case AUTO_TOKEN_FRONT_SEEK_TOTE:
		//drive & convey until the tote is seen by the front sensor

		//Get speed and timeout
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		Message.params.autonomous.driveSpeed = atof(pToken);
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		Message.params.autonomous.timeout = atof(pToken);

		//start the drive train
		Message.command = COMMAND_DRIVETRAIN_START_DRIVE_FWD;//simply drives forward
		CommandNoResponse(DRIVETRAIN_QUEUE);
		//when front sensor sees tote, stop drivetrain and conveyor
		Message.command = COMMAND_CONVEYOR_SEEK_TOTE_FRONT;
		bReturn = !CommandResponse(CONVEYOR_QUEUE);
		Message.command = COMMAND_DRIVETRAIN_STOP;//stop it!
		CommandNoResponse(DRIVETRAIN_QUEUE);
		break;

	case AUTO_TOKEN_FRONT_LOAD_TOTE:
		//draw tote into the robot from the front
		//assumes tote is in position: convey until back sensor

		//Get timeout
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		Message.params.autonomous.timeout = atof(pToken);

		Message.command = COMMAND_CONVEYOR_FRONTLOAD_TOTE;
		bReturn = !CommandResponse(CONVEYOR_QUEUE);
		break;

	case AUTO_TOKEN_BACK_SEEK_TOTE:
		//drive & convey until the tote is seen by the back sensor

		//Get speed and timeout
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		Message.params.autonomous.driveSpeed = atof(pToken);
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		Message.params.autonomous.timeout = atof(pToken);

		//start the drive train
		Message.command = COMMAND_DRIVETRAIN_START_DRIVE_BCK;	//simply drives backwards
		CommandNoResponse(DRIVETRAIN_QUEUE);
		//when back sensor sees tote, stop drivetrain and conveyor
		Message.command = COMMAND_CONVEYOR_SEEK_TOTE_BACK;
		bReturn = !CommandResponse(CONVEYOR_QUEUE);
		Message.command = COMMAND_DRIVETRAIN_STOP;		//stop it!
		CommandNoResponse(DRIVETRAIN_QUEUE);
		break;

	case AUTO_TOKEN_BACK_LOAD_TOTE:
		//draw tote into the robot from the back
		//assumes tote is in position: convey until front sensor

		//Get timeout
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		Message.params.autonomous.timeout = atof(pToken);

		Message.command = COMMAND_CONVEYOR_BACKLOAD_TOTE;
		bReturn = !CommandResponse(CONVEYOR_QUEUE);
		break;

	case AUTO_TOKEN_START_DRIVE_FWD:
		//Get speed and timeout
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		Message.params.autonomous.driveSpeed = atof(pToken);

		Message.command = COMMAND_DRIVETRAIN_START_DRIVE_FWD;//simply drives forward
		CommandNoResponse(DRIVETRAIN_QUEUE);
		break;

	case AUTO_TOKEN_START_DRIVE_BCK:
		//Get speed and timeout
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		Message.params.autonomous.driveSpeed = atof(pToken);

		Message.command = COMMAND_DRIVETRAIN_START_DRIVE_BCK;	//simply drives back
		CommandNoResponse(DRIVETRAIN_QUEUE);
		break;

	case AUTO_TOKEN_STOP_DRIVE:
		Message.command = COMMAND_DRIVETRAIN_STOP;
		CommandNoResponse(DRIVETRAIN_QUEUE);
		break;

	case AUTO_TOKEN_WAIT_FRONT_BEAM:
		Message.command = COMMAND_CONVEYOR_WAIT_FRONT_BEAM;
		bReturn = !CommandResponse(CONVEYOR_QUEUE);
		break;

	case AUTO_TOKEN_WAIT_BACK_BEAM:
			Message.command = COMMAND_CONVEYOR_WAIT_BACK_BEAM;
			bReturn = !CommandResponse(CONVEYOR_QUEUE);
			break;

		//TESTED
	case AUTO_TOKEN_DEPOSITTOTES_BCK:
		Message.command = COMMAND_CONVEYOR_DEPOSITTOTES_BCK;
		bReturn = !CommandResponse(CONVEYOR_QUEUE);
		break;

		//TESTED
	case AUTO_TOKEN_SHIFT_TOTES_FWD:
		Message.command = COMMAND_CONVEYOR_SHIFTTOTES_FWD;
		bReturn = !CommandNoResponse(CONVEYOR_QUEUE);
		break;

		//TESTED
	case AUTO_TOKEN_SHIFT_TOTES_BCK:
		Message.command = COMMAND_CONVEYOR_SHIFTTOTES_BCK;
		bReturn = !CommandNoResponse(CONVEYOR_QUEUE);
		break;

		//TESTED
	case AUTO_TOKEN_PUSH_TOTES_BCK:
		Message.command = COMMAND_CONVEYOR_PUSHTOTES_BCK;
		bReturn = !CommandNoResponse(CONVEYOR_QUEUE);
		break;

	case AUTO_TOKEN_CAN_ARM_OPEN:
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		Message.params.autonomous.driveSpeed = atof(pToken);
		/*pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		Message.params.autonomous.timeout = atof(pToken);*/

		//start the drive train
		Message.command = COMMAND_DRIVETRAIN_START_DRIVE_FWD;//simply drives forward - we're borrowing this (should change it)
		CommandNoResponse(DRIVETRAIN_QUEUE);
		Message.command = COMMAND_CANARM_OPEN;
		bReturn = !CommandResponse(CANARM_QUEUE);
		Message.command = COMMAND_DRIVETRAIN_STOP;//stop it!
		CommandNoResponse(DRIVETRAIN_QUEUE);
		//if we want to keep still
		/*Message.command = COMMAND_DRIVETRAIN_START_KEEPALIGN;
		bReturn = !CommandNoResponse(DRIVETRAIN_QUEUE);
		//Message.command = COMMAND_DRIVETRAIN_STOP_KEEPALIGN;
		bReturn = !CommandNoResponse(DRIVETRAIN_QUEUE);*/
		break;

	case AUTO_TOKEN_CAN_ARM_CLOSE:
		Message.command = COMMAND_CANARM_CLOSE;
		bReturn = !CommandNoResponse(CANARM_QUEUE);
		break;

//OLD FUNCTIONS
	case AUTO_TOKEN_SEEK_TOTE:
		if (!SeekTote(pCurrLinePos))
		{
			rStatus.append("seekTote error");
			bReturn = true;
		}
		else
		{
			rStatus.append("seekTote");
		}
		break;

	case AUTO_TOKEN_CUBE_AUTO:
		if (!CubeAuto(pCurrLinePos))
		{
			rStatus.append("cube auto error");
		}
		else
		{
			rStatus.append("cube auto");
		}
		break;

	case AUTO_TOKEN_CLICKER_UP:
		break;

	case AUTO_TOKEN_CLICKER_DOWN:
		break;

	default:
		rStatus.append("unknown token");
		break;
	}

	printf("%s\n", rStatus.c_str());
	return (bReturn);
}
