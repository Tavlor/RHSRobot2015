/** \file
 * Implementation of class to control can lifter.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the can lifter.
 *
 * Lifts the can onto the top of a stack of totes. It also has hall effect
 * sensors to stop motion at the top and bottom, as well as trigger other actions.
 * 	Raise: negative (with CIM)
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

	lifterMotor = new CANTalon(CAN_PALLET_JACK_BIN_LIFT);
	wpi_assert(lifterMotor);
	lifterMotor->SetVoltageRampRate(120.0);
	lifterMotor->ConfigNeutralMode(
			CANSpeedController::NeutralMode::kNeutralMode_Brake);
	lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

	wpi_assert(lifterMotor->IsAlive());

	clawMotor = new CANTalon(CAN_PALLET_JACK_CLAW);
	wpi_assert(clawMotor);
	lifterMotor->SetVoltageRampRate(120.0);
	clawMotor->ConfigNeutralMode(
			CANSpeedController::NeutralMode::kNeutralMode_Brake);
	clawMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

	wpi_assert(clawMotor->IsAlive());

	lifterHallEffectBottom = lifterMotor->IsRevLimitSwitchClosed();
	lifterHallEffectTop = lifterMotor->IsFwdLimitSwitchClosed();

	pSafetyTimer = new Timer();
	pSafetyTimer->Start();
	pUpdateTimer = new Timer();
	pUpdateTimer->Start();
	pClawTimer = new Timer();
	pClawTimer->Start();

	pTask = new Task(CANLIFTER_TASKNAME, (FUNCPTR) &CanLifter::StartTask,
			CANLIFTER_PRIORITY, CANLIFTER_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int) this);
}

CanLifter::~CanLifter() {
	delete (pTask);
	delete lifterMotor;
	delete clawMotor;
	delete pSafetyTimer;
}
;

void CanLifter::OnStateChange() {
	switch (localMessage.command)
	{
		case COMMAND_ROBOT_STATE_AUTONOMOUS:
		case COMMAND_ROBOT_STATE_TEST:
		case COMMAND_ROBOT_STATE_TELEOPERATED:
		case COMMAND_ROBOT_STATE_DISABLED:
		case COMMAND_ROBOT_STATE_UNKNOWN:
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
			lifterMotor->Set(localMessage.params.lifterSpeed);
			bHover = false;
			pSafetyTimer->Reset();
		break;

		case COMMAND_CANLIFTER_LOWER:
			lifterMotor->Set(-localMessage.params.lifterSpeed);
			bHover = false;
			pSafetyTimer->Reset();
		break;
		case COMMAND_CANLIFTER_HOVER:
			lifterMotor->Set(fLifterHover);
			bHover = true;
			pSafetyTimer->Reset();
		break;
		case COMMAND_CANLIFTER_STOP:
			if(bHover)
			{
				lifterMotor->Set(fLifterHover);
			}
			else
			{
			lifterMotor->Set(fLifterStop);
			}
			pSafetyTimer->Reset();
		break;

		case COMMAND_CLAW_OPEN:
			pClawTimer->Reset();
			if (clawMotor->GetOutputCurrent() < fClawMotorCurrentMax)
			{
				clawMotor->Set(fClawOpen);
			}
			else
			{
				clawMotor->Set(fClawStop);
			}
			pSafetyTimer->Reset();
		break;

		case COMMAND_CLAW_CLOSE:
			pClawTimer->Reset();
			if (clawMotor->GetOutputCurrent() < fClawMotorCurrentMax)
			{
				clawMotor->Set(fClawClose);
			}
			else
			{
				clawMotor->Set(fClawStop);
			}
			pSafetyTimer->Reset();
		break;

		case COMMAND_CLAW_STOP:
			clawMotor->Set(fClawStop);
			pSafetyTimer->Reset();
		break;

		default:
		break;
	}	//end of command switch

	// a timer is used to allow the claw to move
	if(pClawTimer->Get() > 2.0)
	{
		clawMotor->Set(fClawStop);
	}

	//if the connection times out, shut everything off
	if (pSafetyTimer->Get() > 30.0)
	{
		lifterMotor->Set(fLifterStop);
		clawMotor->Set(fClawStop);
		pSafetyTimer->Reset();
	}

	// update the Smart Dashboard periodically to reduce traffic
	if (pUpdateTimer->Get() > 0.2)
	{
		pUpdateTimer->Reset();
		SmartDashboard::PutNumber("Lift Current", lifterMotor->GetOutputCurrent());
		lifterHallEffectBottom = lifterMotor->IsRevLimitSwitchClosed();
		lifterHallEffectTop = lifterMotor->IsFwdLimitSwitchClosed();
		SmartDashboard::PutBoolean("Lifter @ Top", lifterHallEffectTop);
		SmartDashboard::PutBoolean("Lifter @ Bottom", lifterHallEffectBottom);
		SmartDashboard::PutBoolean("Lifter Hover", bHover);
	}
}

