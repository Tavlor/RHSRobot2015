/** \file
 * Implementation of class to control tote conveyor on the pallet jack.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the pallet jack's conveyor.
 *
 * The task receives messages form the main robot class and runs the conveyor
 * forward or backwards.  A beam break sensor is used to position the totes.
 */

#include "Conveyor.h"
#include "WPILib.h"

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

Conveyor::Conveyor() :
		ComponentBase(CONVEYOR_TASKNAME, CONVEYOR_QUEUE, CONVEYOR_PRIORITY) {
	bBackStopEnable = true;

	conveyorMotor = new CANTalon(CAN_PALLET_JACK_CONVEYOR);
	wpi_assert(conveyorMotor);
	conveyorMotor->SetControlMode(CANSpeedController::kPercentVbus);
	conveyorMotor->SetVoltageRampRate(24.0);
	conveyorMotor->ConfigFwdLimitSwitchNormallyOpen(false);
	conveyorMotor->ConfigRevLimitSwitchNormallyOpen(false);
	conveyorMotor->ConfigLimitMode(
			CANSpeedController::kLimitMode_SwitchInputsOnly);

	wpi_assert(conveyorMotor->IsAlive());

	//pAutoTimer = new Timer();IN COMPONENT BASE
	//pAutoTimer->Start();
	pTask = new Task(CONVEYOR_TASKNAME, (FUNCPTR) &Conveyor::StartTask,
			CONVEYOR_PRIORITY, CONVEYOR_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int) this);
}

Conveyor::~Conveyor() {
	delete (pTask);
}

void Conveyor::OnStateChange() {
	switch (localMessage.command)
	{
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
		conveyorMotor->Set(0.0);
		conveyorMotor->ConfigLimitMode(
				CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
		conveyorMotor->ConfigNeutralMode(
				CANSpeedController::NeutralMode::kNeutralMode_Brake);
		break;
	case COMMAND_ROBOT_STATE_TEST:
		conveyorMotor->Set(0.0);
		break;
	case COMMAND_ROBOT_STATE_TELEOPERATED:
		conveyorMotor->Set(0.0);
		conveyorMotor->ConfigLimitMode(
				CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
		conveyorMotor->ConfigNeutralMode(
				CANSpeedController::NeutralMode::kNeutralMode_Brake);//Coast);
		break;
	case COMMAND_ROBOT_STATE_DISABLED:
		conveyorMotor->Set(0.0);
		break;
	case COMMAND_ROBOT_STATE_UNKNOWN:
		conveyorMotor->Set(0.0);
		break;
	default:
		conveyorMotor->Set(0.0);
		break;
	}
}

void Conveyor::Run() {
	switch (localMessage.command)
	//Reads the message command
	{
	case COMMAND_CONVEYOR_RUN_FWD:
		//SmartDashboard::PutString("Conveyor CMD", "CONVEYOR_RUN_FWD");
		conveyorMotor->ConfigLimitMode(
				CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
		conveyorMotor->Set(fConveyorSpeedFwd);
		break;

	case COMMAND_CONVEYOR_RUN_BCK:
		//FIXME Does backstop disable forward stuff as well?
		//runs conveyor towards the claw
		//SmartDashboard::PutString("Conveyor CMD", "CONVEYOR_RUN_BCK");
		//if(localMessage.params.conveyorParams.bButtonWentDownEvent)
		//{
		if (!conveyorMotor->IsFwdLimitSwitchClosed())
		{
			//if there is a tote blocking the back, see if bBackStopEnable is false
			if (!bBackStopEnable)
			{
				//printf("button down: closed, disable switch\n");
				conveyorMotor->ConfigLimitMode(
						CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
				conveyorMotor->Set(fConveyorSpeed);
			}
		}
		//if there is no tote, use limits
		else
		{
			//printf("button down: open, enable switch\n");
			conveyorMotor->ConfigLimitMode(
					CANSpeedController::kLimitMode_SwitchInputsOnly);
			conveyorMotor->Set(fConveyorSpeed);
			bBackStopEnable = true;
		}
		//}
		//conveyorMotor->Set(fConveyorSpeed);
		break;

	case COMMAND_CONVEYOR_SET_BACK:
		//used when you don't need the frills of "RUN_BACK"
		conveyorMotor->Set(fConveyorSpeed);
		break;

	case COMMAND_CONVEYOR_STOP:
		//SmartDashboard::PutString("Conveyor CMD", "CONVEYOR_STOP");
		conveyorMotor->Set(0.0);
		bBackStopEnable = false;
		break;

	//AUTONOMOUS CASES
	case COMMAND_CONVEYOR_SEEK_TOTE_FRONT:
		pAutoTimer->Reset();
		responseCommand = COMMAND_AUTONOMOUS_RESPONSE_OK;
		while (conveyorMotor->IsRevLimitSwitchClosed() && ISAUTO)
		{
			//TIMEOUT
			if(pAutoTimer->Get() > localMessage.params.autonomous.timeout)
			{
				responseCommand = COMMAND_AUTONOMOUS_RESPONSE_ERROR;
				conveyorMotor->Set(0);
				break;
			}
			conveyorMotor->Set(fLoadSpeed);
		}
		SendCommandResponse(responseCommand);
		break;

	case COMMAND_CONVEYOR_SEEK_TOTE_BACK:
		pAutoTimer->Reset();
		responseCommand = COMMAND_AUTONOMOUS_RESPONSE_OK;
		while (conveyorMotor->IsFwdLimitSwitchClosed() && ISAUTO)
		{
			//TIMEOUT
			if(pAutoTimer->Get() > localMessage.params.autonomous.timeout)
			{
				responseCommand = COMMAND_AUTONOMOUS_RESPONSE_ERROR;
				conveyorMotor->Set(0);
				break;
			}
			conveyorMotor->Set(-fLoadSpeed);
		}
		SendCommandResponse(responseCommand);
		break;

	case COMMAND_CONVEYOR_FRONTLOAD_TOTE:
		pAutoTimer->Reset();
		responseCommand = COMMAND_AUTONOMOUS_RESPONSE_OK;
		//convey backwards until back sensor
		while (conveyorMotor->IsFwdLimitSwitchClosed() && ISAUTO)
		{
			//TIMEOUT
			if(pAutoTimer->Get() > localMessage.params.autonomous.timeout)
			{
				responseCommand = COMMAND_AUTONOMOUS_RESPONSE_ERROR;
				conveyorMotor->Set(0);
				break;
			}
			conveyorMotor->Set(fLoadSpeed);
		}
		conveyorMotor->Set(0);
		SendCommandResponse(responseCommand);
		break;

	case COMMAND_CONVEYOR_BACKLOAD_TOTE:
		pAutoTimer->Reset();
		responseCommand = COMMAND_AUTONOMOUS_RESPONSE_OK;
		//convey forwards until forward sensor
		while (conveyorMotor->IsRevLimitSwitchClosed() && ISAUTO)
		{
			//TIMEOUT
			if(pAutoTimer->Get() > localMessage.params.autonomous.timeout)
			{
				responseCommand = COMMAND_AUTONOMOUS_RESPONSE_ERROR;
				conveyorMotor->Set(0);
				break;
			}
			conveyorMotor->Set(-fLoadSpeed);
		}
		conveyorMotor->Set(0);
		SendCommandResponse(responseCommand);
		break;

	case COMMAND_CONVEYOR_SHIFTTOTES_FWD:
		pAutoTimer->Reset();
		//move the stack forward until the back sensor is unblocked
		while (!conveyorMotor->IsFwdLimitSwitchClosed() && ISAUTO
				&& pAutoTimer->Get()< 3.0)
		{
			conveyorMotor->Set(-fShiftSpeed);
		}
		conveyorMotor->Set(0);
		break;

	case COMMAND_CONVEYOR_SHIFTTOTES_BCK:
		pAutoTimer->Reset();
		//move the stack forward until the back sensor is blocked
		while (conveyorMotor->IsFwdLimitSwitchClosed() && ISAUTO
				&& pAutoTimer->Get()< 3.0)
		{
			conveyorMotor->Set(fShiftSpeed);
		}
		conveyorMotor->Set(0);
		break;

	case COMMAND_CONVEYOR_PUSHTOTES_BCK:
		pAutoTimer->Reset();

		while (ISAUTO && pAutoTimer->Get()< 0.25)
		{
			conveyorMotor->Set(fShiftSpeed);
			//conveyorMotor->Set(fPushSpeed);
		}
		conveyorMotor->Set(0);
		break;

	case COMMAND_CONVEYOR_WAIT_FRONT_BEAM:
		bReplyFrontSensor = true;
		break;

	case COMMAND_CONVEYOR_WAIT_BACK_BEAM:
		bReplyBackSensor = true;
		break;

	case COMMAND_CONVEYOR_DEPOSITTOTES_BCK:
		conveyorMotor->ConfigLimitMode(
				CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
		responseCommand = COMMAND_AUTONOMOUS_RESPONSE_OK;
		//move the stack backwards until the both sensors are unblocked
		while (ISAUTO && (!conveyorMotor->IsRevLimitSwitchClosed()
				|| !conveyorMotor->IsFwdLimitSwitchClosed()))
		{
			conveyorMotor->Set(fDepositSpeed);
		}
		conveyorMotor->Set(0);
		SendCommandResponse(responseCommand);
		break;

	case COMMAND_SYSTEM_MSGTIMEOUT:
		//SmartDashboard::PutString("Conveyor CMD", "SYSTEM_MSGTIMEOUT");
	default:
		break;
	}

	if(bReplyFrontSensor)
	{
		if(!conveyorMotor->IsRevLimitSwitchClosed())
		{
			SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
			bReplyFrontSensor = false;
		}
	}
	if(bReplyBackSensor)
	{
		if(!conveyorMotor->IsFwdLimitSwitchClosed())
		{
			SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
			bReplyBackSensor = false;
		}
	}


	//Put out information
	if (pRemoteUpdateTimer->Get() > 0.2)
	{
		pRemoteUpdateTimer->Reset();
		SmartDashboard::PutBoolean("Pallet Jack Front IR",
				!conveyorMotor->IsRevLimitSwitchClosed());
		SmartDashboard::PutBoolean("Pallet Jack Back IR",
				!conveyorMotor->IsFwdLimitSwitchClosed());
	}
	//this needs to be realtime
	SmartDashboard::PutBoolean("Back Stop Enabled",bBackStopEnable);
}

bool Conveyor::RevLimitSwitchClosed()
{
	return conveyorMotor->IsRevLimitSwitchClosed();
}

bool Conveyor::FwdLimitSwitchClosed()
{
	return conveyorMotor->IsFwdLimitSwitchClosed();
}

