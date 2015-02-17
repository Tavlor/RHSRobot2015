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
		"ADDTOTE"
		"NOP" };

void Autonomous::Evaluate(std::string rStatement) {
	char *pToken;
	char *pCurrLinePos;
	int iCommand;
	float fParam1;

	string rStatus;

	if(rStatement.empty()) {
		printf("statement is empty");
		return;
	}

	// process the autonomous motion

	pCurrLinePos = (char *) rStatement.c_str();
	printf("%s\n", rStatement.c_str());

	if(*pCurrLinePos == sComment) {
		rStatus.append("comment");
		printf("%s\n", rStatus.c_str());
		return;
	}

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
		return;
	}

	// execute the proper command

	switch(iCommand) {
	case AUTO_TOKEN_BEGIN:
		rStatus.append("start");
		break;

	case AUTO_TOKEN_END:
		rStatus.append("done");
		break;

	case AUTO_TOKEN_DELAY:
		pToken = strtok_r(pCurrLinePos, szDelimiters, &pCurrLinePos);

		if(pToken == NULL) {
			rStatus.append("missing parameter");
		}
		else {
			fParam1 = atof(pToken);
			rStatus.append("wait");
			Wait(fParam1);
			// break out if auto mode is over

			//for(fWait = 0.0; fWait < fParam1; fWait += 0.01)
			//{
			//	if(bInAutoMode)
			//	{
			//		Wait(0.01);
			//	}
			//	else
			//	{
			//		break;
			//	}
			//}
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
			rStatus.append("move error");
		}
		else {
			rStatus.append("move");
		}
		break;

	default:
		rStatus.append("unknown token");
		break;
		;
	}

	printf("%s\n", rStatus.c_str());
}
