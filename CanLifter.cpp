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

	bHover = false;
	bMiddleHover = false;
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

	midHallEffect = new DigitalInput(DIO_CANLIFTER_MID_HALL_EFFECT);
	lifterHallEffectBottom = lifterMotor->IsRevLimitSwitchClosed();
	lifterHallEffectTop = lifterMotor->IsFwdLimitSwitchClosed();

	midDetect = new Counter(midHallEffect);
	midDetect->Reset();

	pSafetyTimer = new Timer();
	pSafetyTimer->Start();

	pTask = new Task(CANLIFTER_TASKNAME, (FUNCPTR) &CanLifter::StartTask,
			CANLIFTER_PRIORITY, CANLIFTER_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int) this);
}

CanLifter::~CanLifter() {
	delete (pTask);
	delete lifterMotor;
	delete pSafetyTimer;
	delete midHallEffect;
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
		if (CheckLifterCurrentOK())
		{
			lifterMotor->Set(localMessage.params.canLifterParams.lifterSpeed);
		}
		bHover = false;
		bMiddleHover = false;
		pSafetyTimer->Reset();
		break;

	case COMMAND_CANLIFTER_LOWER:
		if (CheckLifterCurrentOK())
		{
			lifterMotor->Set(-localMessage.params.canLifterParams.lifterSpeed);
		}
		bHover = false;
		bMiddleHover = false;
		pSafetyTimer->Reset();
		break;

		case COMMAND_CANLIFTER_HOVER:
			lifterMotor->Set(fLifterHoverNoTotes);
			bHover = true;
			pSafetyTimer->Reset();
			break;

		case COMMAND_CANLIFTER_STARTRAISETOTES:
			if (CheckLifterCurrentOK())
			{
				lifterMotor->Set(fLifterRaise);
			}
			bMiddleHover = true;
			pSafetyTimer->Reset();
			break;


		case COMMAND_CANLIFTER_RAISE_TOTES:
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
			bMiddleHover = true;
			pSafetyTimer->Reset();
			iToteLoad = localMessage.params.canLifterParams.iNumTotes;
			lifterMotor->Set(fLifterRaise);
			/*if(iToteLoad == 1)
			{
				lifterMotor->Set(fLifterLiftOneTotes);
			}
			else if(iToteLoad == 2)
			{
				lifterMotor->Set(fLifterLiftTwoTotes);
			}
			else if(iToteLoad == 3)
			{
				lifterMotor->Set(fLifterLiftThreeTotes);
				SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
			}
			else
			{
				lifterMotor->Set(fLifterLiftNoTotes);
			}*/
			break;

		case COMMAND_CANLIFTER_LOWER_TOTES:
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
			bMiddleHover = false;
			pSafetyTimer->Reset();

			lifterMotor->Set(fLifterLower);
			/*if(iToteLoad == 1)
			{
				lifterMotor->Set(fLifterLowerOneTotes);
			}
			else if(iToteLoad == 2)
			{
				lifterMotor->Set(fLifterLowerTwoTotes);
			}
			else if(iToteLoad == 3)
			{
				//really just grabbing ahold, not lifting
				lifterMotor->Set(fLifterLowerThreeTotes);
			}
			else
			{
				lifterMotor->Set(fLifterLowerNoTotes);
			}*/
			break;

		case COMMAND_CANLIFTER_RAISE_CAN:
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
			break;

		case COMMAND_CANLIFTER_LOWER_CAN:
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
			break;

		case COMMAND_CANLIFTER_STOP:
			if(bHover)
			{
				// stop but with enough juice to hold the carriage in place

				lifterMotor->Set(fLifterHoverNoTotes);
			}
			else if(!bMiddleHover)
			{
				// stop if we are not hovering with a tote

				lifterMotor->Set(fLifterStop);
			}

			pSafetyTimer->Reset();
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

	// if the middle sensor is tripped were we moving up or down or should we hover holding a tote?

	if(midDetect->Get())
	{
		midDetect->Reset();

		if(bMiddleHover)
		{
			// hover here with enough juice to hold steady

			if(iToteLoad == 1)
			{
				lifterMotor->Set(fLifterHoverOneTotes);
			}
			else if(iToteLoad == 2)
			{
				lifterMotor->Set(fLifterHoverTwoTotes);
			}
			else if(iToteLoad == 3)
			{
				lifterMotor->Set(fLifterHoverThreeTotes);
			}
			else
			{
				lifterMotor->Set(fLifterHoverNoTotes);
			}

			SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
		}
		else
		{
			// not holding a tote, just normal can up and down motions

			if(lifterMotor->Get() > 0.0)
			{
				// was on the way up

				bGoingUp = true;
				bGoingDown = false;
			}
			else if(lifterMotor->Get() < 0.0)
			{
				// was on the way down

				bGoingUp = false;
				bGoingDown = true;
			}
		}
	}

	// update the Smart Dashboard periodically to reduce traffic
	if (pRemoteUpdateTimer->Get() > 0.2)
	{
		pRemoteUpdateTimer->Reset();
		lifterHallEffectBottom = lifterMotor->IsRevLimitSwitchClosed();
		lifterHallEffectTop = lifterMotor->IsFwdLimitSwitchClosed();

		SmartDashboard::PutNumber("Lift Current", lifterMotor->GetOutputCurrent());
		SmartDashboard::PutBoolean("Lifter @ Top", lifterHallEffectTop);
		SmartDashboard::PutBoolean("Lifter @ Bottom", lifterHallEffectBottom);
		SmartDashboard::PutBoolean("Lifter Hover", bHover);
	}
}

bool CanLifter::CheckLifterCurrentOK() {
	if (lifterMotor->GetOutputCurrent() > fLifterMotorCurrentMax)
	{
		lifterMotor->Set(fLifterStop);
		return false;
	}
	return true;
}
