/** \file
 * Implementation of class to control can lifter.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the can lifter.
 *
 * Lifts the can onto the top of a stack of totes. It also has hall effect
 * sensors to stop motion at the top and bottom, as well as trigger other actions.
 * 	Open: negative (with bag)
 *	Close: Positive (with bag)
 *
 */

//Please DO NOT modify the +/- motor value notes above with out FULL TESTING
#include "Claw.h"

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

Claw::Claw() :
		ComponentBase(CLAW_TASKNAME, CLAW_QUEUE, CLAW_PRIORITY) {
	clawMotor = new CANTalon(CAN_PALLET_JACK_CLAW);
	wpi_assert(clawMotor);
	clawMotor->ConfigNeutralMode(
			CANSpeedController::NeutralMode::kNeutralMode_Brake);
	clawMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
	clawMotor->SetVoltageRampRate(120.0);

	wpi_assert(clawMotor->IsAlive());

	pSafetyTimer = new Timer();
	pSafetyTimer->Start();
	pClawTimer = new Timer();
	pClawTimer->Start();

	pTask = new Task(CLAW_TASKNAME, (FUNCPTR) &Claw::StartTask,
			CLAW_PRIORITY, CLAW_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int) this);
}

Claw::~Claw() {
	delete (pTask);
	delete clawMotor;
	delete pSafetyTimer;
}
;

void Claw::OnStateChange() {
	switch (localMessage.command)
	{
		case COMMAND_ROBOT_STATE_AUTONOMOUS:
		case COMMAND_ROBOT_STATE_TEST:
		case COMMAND_ROBOT_STATE_TELEOPERATED:
		case COMMAND_ROBOT_STATE_DISABLED:
		case COMMAND_ROBOT_STATE_UNKNOWN:
		default:
			clawMotor->Set(fClawStop);
			pSafetyTimer->Reset();
		break;
	}
}

void Claw::Run() {
	switch (localMessage.command)
	{
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

	//current safety
	if (clawMotor->GetOutputCurrent() >= fClawMotorCurrentMax)
	{
		clawMotor->Set(fClawStop);
	}

	// a timer is used to allow the claw to move
	if(pClawTimer->Get() > fClawActionTimeLimit)
	{
		clawMotor->Set(fClawStop);
	}

	//if the connection times out, shut everything off
	if (pSafetyTimer->Get() > 30.0)
	{
		clawMotor->Set(fClawStop);
		pSafetyTimer->Reset();
	}

	SmartDashboard::PutNumber("Claw Current", TRUNC_THOU(clawMotor->GetOutputCurrent()));
}

