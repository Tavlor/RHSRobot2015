#include <ToteLifter.h>
//Robot
#include "ComponentBase.h"
#include "RobotParams.h"



ToteLifter::ToteLifter() : ComponentBase(TOTELIFTER_TASKNAME, TOTELIFTER_QUEUE, TOTELIFTER_PRIORITY) {
	bExtending = false;
	bRetracting = false;

	toteMotor = new CANTalon(CAN_PALLET_JACK_TOTE_LIFT);
	wpi_assert(toteMotor);
	toteMotor->SetVoltageRampRate(120.0);
	toteMotor->ConfigNeutralMode(
			CANSpeedController::NeutralMode::kNeutralMode_Brake);
	toteMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

	wpi_assert(toteMotor->IsAlive());

	pExtendTimer = new Timer();
	pExtendTimer->Start();
	pRetractTimer = new Timer();
	pRetractTimer->Start();

	pTask = new Task(TOTELIFTER_TASKNAME, (FUNCPTR) &ToteLifter::StartTask,
			TOTELIFTER_PRIORITY, TOTELIFTER_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int) this);
}

ToteLifter::~ToteLifter() {
	delete pTask;
	delete toteMotor;
	delete pExtendTimer;
	delete pRetractTimer;
}

void ToteLifter::OnStateChange()
{
	switch (localMessage.command)
	{
		case COMMAND_ROBOT_STATE_AUTONOMOUS:
			break;

		case COMMAND_ROBOT_STATE_TEST:
			break;

		case COMMAND_ROBOT_STATE_TELEOPERATED:
			toteMotor->Set(fToteStop);
			bExtending = false;
			bRetracting = false;
			break;

		case COMMAND_ROBOT_STATE_DISABLED:
			break;

		case COMMAND_ROBOT_STATE_UNKNOWN:
			break;

		default:
			toteMotor->Set(fToteStop);
			bExtending = false;
			bRetracting = false;
		break;
	}
}

void ToteLifter::Run()
{
	switch(localMessage.command)
	{
		case COMMAND_TOTELIFTER_EXTEND:
			toteMotor->Set(fToteExtend);
			bExtending = true;
			bRetracting = false;
			pExtendTimer->Reset();
			break;

		case COMMAND_TOTELIFTER_RETRACT:
			toteMotor->Set(fToteRetract);
			bRetracting = true;
			bExtending = false;
			pRetractTimer->Reset();
			break;

		case COMMAND_SYSTEM_MSGTIMEOUT:
			break;

		default:
			break;
	}

	if(bExtending && (pExtendTimer->Get() > fExtendTime))
	{
		toteMotor->Set(fToteStop);
		bExtending = false;
	}

	if(bRetracting && (pRetractTimer->Get() > fRetractTime))
	{
		toteMotor->Set(fToteStop);
		bRetracting = false;
	}
}
