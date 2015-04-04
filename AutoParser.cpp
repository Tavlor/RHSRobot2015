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
		"SEEKTOTE",
		"STRAIGHT",
		"TOTEEXTEND",
		"TOTERETRACT",
		"STARTTOTEUP",
		"TOTEUP",
		"TOTEDOWN",
		"CLAWOPEN",
		"CLAWCLOSE",
		"CLICKERUP",
		"CLICKERDOWN",
		"CUBEAUTO",
		"NOP" };

bool Autonomous::Evaluate(std::string rStatement) {
	char *pToken;
	char *pCurrLinePos;
	int iCommand;
	float fParam1;
	int iParam1;
	bool bReturn = false;
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

	switch(iCommand) {
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

		if(pToken == NULL) {
			rStatus.append("missing parameter");
		}
		else {
			fParam1 = atof(pToken);
			rStatus.append("wait");

			// break out if auto mode is over

			for(double fWait = 0.0; fWait < fParam1; fWait += 0.01)
			{
				// if we are paused break the delay into pieces

				while(bPauseAutoMode)
				{
					Wait(0.02);
				}

				Wait(0.01);
			}
		}
		break;
	case AUTO_TOKEN_MOVE:
		if(!Move(pCurrLinePos)) {
			rStatus.append("move error");
		}
		else {
			rStatus.append("move");
		}
		break;
	case AUTO_TOKEN_MMOVE:
		if(!MeasuredMove(pCurrLinePos)) {
			rStatus.append("move error");
		}
		else {
			rStatus.append("move");
		}
		break;
	case AUTO_TOKEN_TURN:
		if(!Turn(pCurrLinePos)) {
			rStatus.append("turn error");
		}
		else {
			rStatus.append("turn");
		}
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
	case AUTO_TOKEN_STRAIGHT:
		if(!Straight(pCurrLinePos)) {
			rStatus.append("straight error");
		}
		else {
			rStatus.append("straight");
		}
		break;

	case AUTO_TOKEN_EXTEND_TOTE:
		Message.command = COMMAND_TOTELIFTER_EXTEND;
		bReturn = !CommandNoResponse(TOTELIFTER_QUEUE);
		break;

	case AUTO_TOKEN_RETRACT_TOTE:
		Message.command = COMMAND_TOTELIFTER_RETRACT;
		bReturn = !CommandNoResponse(TOTELIFTER_QUEUE);
		break;

	case AUTO_TOKEN_START_RAISE_TOTE:
		Message.command = COMMAND_CANLIFTER_STARTRAISETOTES;
		bReturn = !CommandNoResponse(CANLIFTER_QUEUE);
		break;

	case AUTO_TOKEN_RAISE_TOTE:
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		iParam1 = atoi(pToken);

		Message.command = COMMAND_CANLIFTER_RAISETOTES;
		Message.params.canLifterParams.iNumTotes = iParam1;
		bReturn = !CommandResponse(CANLIFTER_QUEUE);
		break;

	case AUTO_TOKEN_LOWER_TOTE:
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);
		iParam1 = atoi(pToken);

		Message.command = COMMAND_CANLIFTER_LOWERTOTES;
		Message.params.canLifterParams.iNumTotes = iParam1;
		bReturn = !CommandResponse(CANLIFTER_QUEUE);
		break;

	case AUTO_TOKEN_CLAW_OPEN:
		Message.command = COMMAND_CLAW_OPEN;
		bReturn = !CommandNoResponse(CLAW_QUEUE);
		break;

	case AUTO_TOKEN_CLAW_CLOSE:
		Message.command = COMMAND_CLAW_CLOSE;
		bReturn = !CommandNoResponse(CLAW_QUEUE);
		break;

	case AUTO_TOKEN_CLICKER_UP:
		break;

	case AUTO_TOKEN_CLICKER_DOWN:
		break;

	case AUTO_TOKEN_CUBE_AUTO:
		if(!CubeAuto(pCurrLinePos)) {
			rStatus.append("cube auto error");
		}
		else {
			rStatus.append("cube auto");
		}
		break;

	default:
		rStatus.append("unknown token");
		break;
	}

	printf("%s\n", rStatus.c_str());
	return (bReturn);
}
