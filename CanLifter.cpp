/** \file
 * Implementation of class to control can lifter.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the can lifter.
 *
 * Lifts the can onto the top of a stack of totes. It also has hall effect
 * sensors to stop motion at the top and bottom, as well as trigger other actions.
 * 	Raise: positive (with CIM)
 *	Lower: negative (with CIM)
 *
 */

//Please DO NOT modify the +/- motor value notes above with out FULL TESTING
#include "CanLifter.h"

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

CanLifter::CanLifter() :
		ComponentBase(CANLIFTER_TASKNAME, CANLIFTER_QUEUE, CANLIFTER_PRIORITY) {

	//bHover = false;
	bHovering = false;
	bHoverEnabled = false;
	bGoingUp = false;
	bGoingDown = false;
	iToteLoad = 0;

	lifterMotor = new CANTalon(CAN_PALLET_JACK_BIN_LIFT);
	wpi_assert(lifterMotor);
	lifterMotor->SetVoltageRampRate(120.0);
	lifterMotor->ConfigNeutralMode(
			CANSpeedController::NeutralMode::kNeutralMode_Brake);
	lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

	wpi_assert(lifterMotor->IsAlive());


	//midDetect = new Counter(midHallEffect);
	//midDetect->Reset();

	pSafetyTimer = new Timer();
	pSafetyTimer->Start();
	//pAutoTimer = new Timer();IN COMPONENT BASE
	//pAutoTimer->Start();

	pTask = new Task(CANLIFTER_TASKNAME, (FUNCPTR) &CanLifter::StartTask,
			CANLIFTER_PRIORITY, CANLIFTER_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int) this);
}

CanLifter::~CanLifter() {
	delete (pTask);
	delete lifterMotor;
	delete pSafetyTimer;
}
;

void CanLifter::OnStateChange() {
	switch (localMessage.command)
	{
		case COMMAND_ROBOT_STATE_AUTONOMOUS:
			lifterMotor->Set(fLifterStop);
			pSafetyTimer->Reset();
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
			break;
		case COMMAND_ROBOT_STATE_TEST:
			lifterMotor->Set(fLifterStop);
			pSafetyTimer->Reset();
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
			break;
		case COMMAND_ROBOT_STATE_TELEOPERATED:
			lifterMotor->Set(fLifterStop);
			pSafetyTimer->Reset();
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
			break;
		case COMMAND_ROBOT_STATE_DISABLED:
			lifterMotor->Set(fLifterStop);
			pSafetyTimer->Reset();
			break;
		case COMMAND_ROBOT_STATE_UNKNOWN:
			lifterMotor->Set(fLifterStop);
			pSafetyTimer->Reset();
			break;
		default:
			lifterMotor->Set(fLifterStop);
			pSafetyTimer->Reset();
			break;
	}
}

void CanLifter::Run() {
	switch (localMessage.command)
	{
		case COMMAND_CANLIFTER_RAISE:
			/*if(bHovering)
			{
				if(!bHoverEnabled)
				{
					lifterMotor->ConfigLimitMode(
							CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
					LifterCurrentLimitDrive(fLifterUpMult * localMessage.params.canLifterParams.lifterSpeed);
				}
			}
			else
			{
				lifterMotor->ConfigLimitMode(
						CANSpeedController::kLimitMode_SwitchInputsOnly);
				LifterCurrentLimitDrive(fLifterUpMult * localMessage.params.canLifterParams.lifterSpeed);
				bHoverEnabled = true;
			}*/
			bHoverEnabled = false;
			lifterMotor->ConfigLimitMode(
									CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
							LifterCurrentLimitDrive(fLifterUpMult * localMessage.params.canLifterParams.lifterSpeed);
			pSafetyTimer->Reset();
			break;

		case COMMAND_CANLIFTER_LOWER:
			LifterCurrentLimitDrive(fLifterDownMult * localMessage.params.canLifterParams.lifterSpeed);
			bHoverEnabled = false;
			pSafetyTimer->Reset();
			break;

		/*case COMMAND_CANLIFTER_HOVER:
			lifterMotor->Set(fLifterHover);
			bHover = true;
			pSafetyTimer->Reset();
			break;*/

		case COMMAND_CANLIFTER_RAISE_TOTES:
			//to load pos
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
			bHoverEnabled = true;
			bLowerHover = false;
			pSafetyTimer->Reset();
			iToteLoad = localMessage.params.canLifterParams.iNumTotes;

			while(ISAUTO && !TOPHALLEFFECT)
			{
				lifterMotor->Set(fLifterRaise);
				Wait(0.02);
			}

			//hovering, so don't stop it
			// MrB - we need to do other things while lifting

			SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
			break;

		case COMMAND_CANLIFTER_LOWER_TOTES:
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
			bHoverEnabled = false;
			bLowerHover = true;
			pSafetyTimer->Reset();

			while(ISAUTO && !BOTTOMHALLEFFECT)
			{
				lifterMotor->Set(fLifterLower);
				Wait(0.02);
			}

			lifterMotor->Set(fLifterStop);
			SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
			break;

		case COMMAND_CANLIFTER_START_RAISE_TOTES:
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
			bHoverEnabled = true;
			bLowerHover = false;
			pSafetyTimer->Reset();
			iToteLoad = localMessage.params.canLifterParams.iNumTotes;
			lifterMotor->Set(fLifterRaise);
			break;

		case COMMAND_CANLIFTER_CLAW_TO_TOP:
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
			bHoverEnabled = false;
			bLowerHover = false;

			while(ISAUTO)
			{
				if (lifterMotor->GetOutputCurrent() > fLifterMotorCurrentMaxOneCan)
				{
					lifterMotor->Set(fLifterHover);
					break;
				}

				SmartDashboard::PutNumber("Lift Current", lifterMotor->GetOutputCurrent());
				lifterMotor->Set(fLifterRaise);
				Wait(0.02);
			}

			SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
			pSafetyTimer->Reset();
			break;

		case COMMAND_CANLIFTER_CLAW_TO_BOTTOM:
			bHoverEnabled = false;
			bLowerHover = false;
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);

			while(ISAUTO && LifterCurrentLimitDrive(fLifterLower))
			{
				SmartDashboard::PutNumber("Lift Current", lifterMotor->GetOutputCurrent());
				Wait(0.02);
			}

			SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
			pSafetyTimer->Reset();
			break;

		/*case COMMAND_CANLIFTER_RAISE_CAN:
			//to hook pos
			bHover = false;
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
			pSafetyTimer->Reset();
			break;*/
		case COMMAND_CANLIFTER_RAISE_LOMID:
			bLowerHover = true;
			bHoverEnabled = false;
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

			while(ISAUTO && !BOTTOMHALLEFFECT)
			{
				lifterMotor->Set(fLifterRaise);
				Wait(0.02);
			}

			//go above the halleffect
			/*while(ISAUTO && BOTTOMHALLEFFECT)
			{
				lifterMotor->Set(fLifterRaise);
			}*/
			lifterMotor->Set(fLifterStop);
			pSafetyTimer->Reset();
			break;

		case COMMAND_CANLIFTER_LOWER_HIMID:
			bHoverEnabled = true;
			bLowerHover = false;
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

			while(ISAUTO && !TOPHALLEFFECT)
			{
				lifterMotor->Set(fLifterLower);
				Wait(0.02);
			}

			pSafetyTimer->Reset();
			break;

		case COMMAND_CANLIFTER_STOP:
			lifterMotor->Set(fLifterStop);
			pSafetyTimer->Reset();
			bHoverEnabled = false;
			bLowerHover = false;
			break;

		default:
			break;

	}	//end of command switch

	//if the connection times out, shut everything off
	if (pSafetyTimer->Get() > 30.0)
	{
		lifterMotor->Set(fLifterStop);
		pSafetyTimer->Reset();
	}

	//check that the lifter is not hurting itself
	if (lifterMotor->GetOutputCurrent() > fLifterMotorCurrentMax)
	{
		lifterMotor->Set(fLifterStop);
	}

	// if the middle sensor is tripped were we moving up or down or should we hover holding a tote?

	/*if(midDetect->Get())
	{
		midDetect->Reset();*/

	if(!bHoverEnabled)
	{
		bHovering = false;
	}
	if(bHoverEnabled && TOPHALLEFFECT)
	{
		bHovering = true;
	}

	if(bHovering)
	{
		if(ISAUTO)//if it's auto, we adjust for tote count
		{
			// hover here with enough juice to hold steady
			switch(iToteLoad)
			{
				case 1:
				lifterMotor->Set(fLifterHoverOneTotes);
				break;
				case 2:
				lifterMotor->Set(fLifterHoverTwoTotes);
				break;
				case 3:
				lifterMotor->Set(fLifterHoverThreeTotes);
				break;
				default:
				lifterMotor->Set(fLifterHover);
				break;
			}
		}
		else
		{
			lifterMotor->Set(fLifterHover);
		}
		//SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
	}

	else if(ISAUTO && bLowerHover)
			{
				if(BOTTOMHALLEFFECT)
				{
					lifterMotor->Set(fLifterHoverNoTotes);
				}
				else
				{
					lifterMotor->Set(fLifterStop);
				}
			}
	else
	{
		// not hovering, just normal can up and down motions

		if(lifterMotor->Get() < 0.0)
		{
			// was on the way up

			bGoingUp = true;
			bGoingDown = false;
		}
		else if(lifterMotor->Get() > 0.0)
		{
			// was on the way down

			bGoingUp = false;
			bGoingDown = true;
		}
	}

	// update the Smart Dashboard periodically to reduce traffic
	if (pRemoteUpdateTimer->Get() > 0.2)
	{
		pRemoteUpdateTimer->Reset();

		SmartDashboard::PutNumber("Lift Current", lifterMotor->GetOutputCurrent());
		SmartDashboard::PutBoolean("Lifter @ Top", TOPHALLEFFECT);
		SmartDashboard::PutBoolean("Lifter @ Bottom", BOTTOMHALLEFFECT);
		SmartDashboard::PutBoolean("Lifter Hover Enabled", bHoverEnabled);
		SmartDashboard::PutBoolean("Lifter Hovering", bHovering);
		SmartDashboard::PutBoolean("Lifter Raising", bGoingUp);
		SmartDashboard::PutBoolean("Lifter Lowering", bGoingDown);
	}
}

bool CanLifter::LifterCurrentLimitDrive(float speed) {
	if (lifterMotor->GetOutputCurrent() > fLifterMotorCurrentMax)
	{
		lifterMotor->Set(fLifterStop);
		return false;
	}
	lifterMotor->Set(speed);
	return true;
}
