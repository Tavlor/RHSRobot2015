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
		"DELAY",
		"MOVE",
		"MMOVE",
		"TURN",
		"STRAIGHT",
		"STACKUP",
		"STACKDOWN",
		"CLAWOPEN",
		"CLAWCLOSE",
		"CANUP",
		"CANDOWN",
		"FRONTLOADTOTE",
		"BACKLOADTOTE",
		"DEPOSITTOTESBACK",
		"TOTESHIFTFWD",
		"TOTESHIFTBCK",
		"CANARMOPEN",
		"CANARMCLOSE",
		//Old commands from past auto attemts
		"SEEKTOTE",
		"STARTTOTEUP",
		"TOTEEXTEND",
		"TOTERETRACT",
		"CUBEAUTO",
		"CLICKERUP",
		"CLICKERDOWN",
		"NOP" };

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
		rStatus.append("no tokens");
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

			// break out if auto mode is over

			for (double fWait = 0.0; fWait < fParam1; fWait += 0.01)
			{
				// if we are paused break the delay into pieces

				while (bPauseAutoMode)
				{
					Wait(0.02);
				}

				Wait(0.01);
			}
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
		break;

	case AUTO_TOKEN_LOWER_TOTES:
		/*pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		iParam1 = atoi(pToken);*/

		Message.command = COMMAND_CANLIFTER_LOWER_TOTES;
		//Message.params.canLifterParams.iNumTotes = iParam1;
		bReturn = !CommandNoResponse(CANLIFTER_QUEUE);
		break;

	case AUTO_TOKEN_RAISE_CAN:
		//TODO AUTO_TOKEN_RAISE_CAN
		Message.command = COMMAND_CANLIFTER_RAISE_CAN;
		bReturn = !CommandResponse(CANLIFTER_QUEUE);
		break;

	case AUTO_TOKEN_LOWER_CAN:
		//TODO AUTO_TOKEN_LOWER_CAN
		Message.command = COMMAND_CANLIFTER_LOWER_CAN;
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

	//TESTED
	case AUTO_TOKEN_FRONT_LOAD_TOTE:
		//draw tote into the robot from the front
		//convey, drive robot fwd until front sensor, cont. convey until back sensor

		//start driving forwards
		Message.command = COMMAND_DRIVETRAIN_FRONTLOAD_TOTE;//simply drives forward
		CommandNoResponse(DRIVETRAIN_QUEUE);
		//when front sensor sees tote, stop drivetrain no matter what
		Message.command = COMMAND_CONVEYOR_WATCH_TOTE_FRONT;
		bReturn = !CommandResponse(CONVEYOR_QUEUE);
		Message.command = COMMAND_DRIVETRAIN_STOP;//simply drives forward
		CommandNoResponse(DRIVETRAIN_QUEUE);
		//if front sensor sees, continue until back sensor sees; otherwise, stop.
		if(!bReturn)
		{
			Message.command = COMMAND_CONVEYOR_FRONTLOAD_TOTE;
			bReturn = !CommandResponse(CONVEYOR_QUEUE);
		}
		else
		{
			Message.command = COMMAND_CONVEYOR_STOP;
			CommandNoResponse(CONVEYOR_QUEUE);
		}
		break;

	//TESTED
	case AUTO_TOKEN_BACK_LOAD_TOTE:
		//draw tote into the robot from the back
		//convey, drive robot back until front sensor, cont. convey until front sensor

		//start driving backwards
		Message.command = COMMAND_DRIVETRAIN_BACKLOAD_TOTE;	//simply drives forward
		CommandNoResponse(DRIVETRAIN_QUEUE);
		//when front sensor sees tote, stop drivetrain no matter what
		Message.command = COMMAND_CONVEYOR_WATCH_TOTE_BACK;
		bReturn = !CommandResponse(CONVEYOR_QUEUE);
		Message.command = COMMAND_DRIVETRAIN_STOP;	//simply drives backwards
		CommandNoResponse(DRIVETRAIN_QUEUE);
		//if back sensor sees, continue until front sensor sees; otherwise, stop.
		if (!bReturn)
		{
			Message.command = COMMAND_CONVEYOR_BACKLOAD_TOTE;
			bReturn = !CommandResponse(CONVEYOR_QUEUE);
		}
		else
		{
			Message.command = COMMAND_CONVEYOR_STOP;
			CommandNoResponse(CONVEYOR_QUEUE);
		}
		break;

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
	case AUTO_TOKEN_CAN_ARM_OPEN:
		Message.command = COMMAND_CANARM_OPEN;
		bReturn = !CommandNoResponse(CANARM_QUEUE);
			break;

	//TESTED
	case AUTO_TOKEN_CAN_ARM_CLOSE:
		Message.command = COMMAND_CANARM_CLOSE;
		bReturn = !CommandNoResponse(CANARM_QUEUE);
			break;

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

	case AUTO_TOKEN_START_RAISE_TOTE:
		Message.command = COMMAND_CANLIFTER_STARTRAISETOTES;
		bReturn = !CommandNoResponse(CANLIFTER_QUEUE);
		break;

	/*case AUTO_TOKEN_EXTEND_TOTE:
		Message.command = COMMAND_TOTELIFTER_EXTEND;
		bReturn = !CommandNoResponse(TOTELIFTER_QUEUE);
		break;

	case AUTO_TOKEN_RETRACT_TOTE:
		Message.command = COMMAND_TOTELIFTER_RETRACT;
		bReturn = !CommandNoResponse(TOTELIFTER_QUEUE);
		break;*/

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
