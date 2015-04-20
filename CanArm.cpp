#include <CanArm.h>
//Robot
#include "ComponentBase.h"
#include "RobotParams.h"



CanArm::CanArm() : ComponentBase(CANARM_TASKNAME, CANARM_QUEUE, CANARM_PRIORITY) {

	armMotor = new CANTalon(CAN_PALLET_JACK_CAN_ARM);
	wpi_assert(armMotor);
	armMotor->SetVoltageRampRate(120.0);
	armMotor->ConfigNeutralMode(
			CANSpeedController::NeutralMode::kNeutralMode_Brake);
	armMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

	//pAutoTimer = new Timer(); IN COMPONENT BASE
	//pAutoTimer->Start();
	wpi_assert(armMotor->IsAlive());
	pTask = new Task(CANARM_TASKNAME, (FUNCPTR) &CanArm::StartTask,
			CANARM_PRIORITY, CANARM_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int) this);
}

CanArm::~CanArm() {
	delete pTask;
	delete armMotor;
}

void CanArm::OnStateChange()
{
	switch (localMessage.command)
	{
		case COMMAND_ROBOT_STATE_AUTONOMOUS:
			armMotor->ConfigNeutralMode(
						CANSpeedController::NeutralMode::kNeutralMode_Brake);
			break;

		case COMMAND_ROBOT_STATE_TEST:
			break;

		case COMMAND_ROBOT_STATE_TELEOPERATED:
			armMotor->ConfigNeutralMode(
				CANSpeedController::NeutralMode::kNeutralMode_Brake);
			armMotor->Set(0);
			break;

		case COMMAND_ROBOT_STATE_DISABLED:
			armMotor->ConfigNeutralMode(
				CANSpeedController::NeutralMode::kNeutralMode_Coast);
			break;

		case COMMAND_ROBOT_STATE_UNKNOWN:
			break;

		default:
			armMotor->Set(0);
		break;
	}
}

void CanArm::Run()
{
	switch(localMessage.command)
	{
		case COMMAND_CANARM_OPEN:
			pAutoTimer->Reset();
			bOpening = true;
			bClosing = false;
			printf("%s opening time %f\n", __FILE__, localMessage.params.autonomous.timeout);
			armMotor->Set(fOpen);
			break;

		case COMMAND_CANARM_CLOSE:
			pAutoTimer->Reset();
			bOpening = false;
			bClosing = true;
			armMotor->Set(fClose);
			break;

		case COMMAND_CANARM_STOP:
			bOpening = false;
			bClosing = false;
			armMotor->Set(0);
			break;

		case COMMAND_SYSTEM_MSGTIMEOUT:
			armMotor->Set(0);
			break;

		default:
			break;
	}

	SmartDashboard::PutNumber("Arm Current", TRUNC_THOU(armMotor->GetOutputCurrent()));

	if (ISAUTO)
	{
		if(bOpening)
		{
			armMotor->Set(fOpen);
//			if(!CheckArmCurrentOK() || pAutoTimer->Get() > localMessage.params.autonomous.timeout)
//			{
//				SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
//				bOpening = false;
//			}
			if(pAutoTimer->Get() >= localMessage.params.autonomous.timeout)
			{
				//SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
				bOpening = false;
			}
		}
		else if(bClosing)
		{
			armMotor->Set(fClose);
			if(pAutoTimer->Get() >= localMessage.params.autonomous.timeout)
			{
				//SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
				bClosing = false;
			}
		}
		else
		{
			armMotor->Set(0);
		}
	}
	else
	{
		armMotor->Set(0);
	}
}

bool CanArm::CheckArmCurrentOK() {
	if (armMotor->GetOutputCurrent() > fArmMotorCurrentMax)
	{
		armMotor->Set(0);
		return false;
	}
	return true;
}
