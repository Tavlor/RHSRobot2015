/**  Implementation of class to control tote conveyor on the pallet jack.
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
		ComponentBase(CONVEYOR_TASKNAME, CONVEYOR_QUEUE,
				CONVEYOR_PRIORITY) {
	conveyorMotor = new CANTalon(CAN_PALLET_JACK_CONVEYOR);
	wpi_assert(conveyorMotor);
	conveyorMotor->SetControlMode(CANSpeedController::kPercentVbus);
	conveyorMotor->SetVoltageRampRate(120.0);
	conveyorMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

	intakeLeftMotor = new CANTalon(CAN_PALLET_JACK_INTAKE_VERTICAL_LEFT);
	wpi_assert(intakeLeftMotor);
	intakeLeftMotor->SetControlMode(CANSpeedController::kPercentVbus);

	intakeRightMotor = new CANTalon(CAN_PALLET_JACK_INTAKE_VERTICAL_RIGHT);
	wpi_assert(intakeRightMotor);
	intakeRightMotor->SetControlMode(CANSpeedController::kPercentVbus);

	wpi_assert(conveyorMotor->IsAlive());
	wpi_assert(intakeLeftMotor->IsAlive());
	wpi_assert(intakeRightMotor->IsAlive());

	pTask = new Task(CONVEYOR_TASKNAME, (FUNCPTR) &Conveyor::StartTask,
			CONVEYOR_PRIORITY, CONVEYOR_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int)this);
}

Conveyor::~Conveyor() {
	delete(pTask);
}

void Conveyor::OnStateChange() {
	switch (localMessage.command) {
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
		conveyorMotor->Set(0.0);
		intakeLeftMotor->Set(0.0);
		intakeRightMotor->Set(0.0);
		break;

	case COMMAND_ROBOT_STATE_TEST:
		conveyorMotor->Set(0.0);
		intakeLeftMotor->Set(0.0);
		intakeRightMotor->Set(0.0);
		break;

	case COMMAND_ROBOT_STATE_TELEOPERATED:
		conveyorMotor->Set(0.0);
		intakeLeftMotor->Set(0.0);
		intakeRightMotor->Set(0.0);
		break;

	case COMMAND_ROBOT_STATE_DISABLED:
		conveyorMotor->Set(0.0);
		intakeLeftMotor->Set(0.0);
		intakeRightMotor->Set(0.0);
		break;

	case COMMAND_ROBOT_STATE_UNKNOWN:
		conveyorMotor->Set(0.0);
		intakeLeftMotor->Set(0.0);
		intakeRightMotor->Set(0.0);
		break;

	default:
		conveyorMotor->Set(0.0);
		intakeLeftMotor->Set(0.0);
		intakeRightMotor->Set(0.0);
		break;
	}
}

void Conveyor::Run() {
	switch (localMessage.command)			//Reads the message command
	{
	case COMMAND_CONVEYOR_RUN_FWD:
		conveyorMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
		conveyorMotor->Set(-fConveyorSpeed);
		break;
	case COMMAND_CONVEYOR_RUN_BCK:
		conveyorMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
		conveyorMotor->Set(fConveyorSpeed);
		break;
	case COMMAND_CONVEYOR_STOP:
		conveyorMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
		conveyorMotor->Set(0.0);
		break;

	case COMMAND_CONVEYOR_INTAKELEFT_IN:
		intakeLeftMotor->Set(-fIntakeSpeed);
		break;

	case COMMAND_CONVEYOR_INTAKELEFT_OUT:
		intakeLeftMotor->Set(fIntakeSpeed);
		break;

	case COMMAND_CONVEYOR_INTAKELEFT_STOP:
		intakeLeftMotor->Set(0.0);
		break;

	case COMMAND_CONVEYOR_INTAKERIGHT_IN:
		intakeRightMotor->Set(fIntakeSpeed);
		break;

	case COMMAND_CONVEYOR_INTAKERIGHT_OUT:
		intakeRightMotor->Set(-fIntakeSpeed);
		break;

	case COMMAND_CONVEYOR_INTAKERIGHT_STOP:
		intakeRightMotor->Set(0.0);
		break;

	case COMMAND_CONVEYOR_INTAKEBOTH_IN:
		intakeLeftMotor->Set(-fIntakeSpeed);
		intakeRightMotor->Set(fIntakeSpeed);
		break;

	case COMMAND_CONVEYOR_INTAKEBOTH_OUT:
		intakeLeftMotor->Set(fIntakeSpeed);
		intakeRightMotor->Set(-fIntakeSpeed);
		break;

	case COMMAND_CONVEYOR_INTAKEBOTH_STOP:
		intakeLeftMotor->Set(0.0);
		intakeRightMotor->Set(0.0);
		break;

	case COMMAND_CONVEYOR_RUNALL_FWD:
		conveyorMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
		conveyorMotor->Set(-fConveyorSpeed);
		intakeLeftMotor->Set(fIntakeSpeed);
		intakeRightMotor->Set(-fIntakeSpeed);
		break;

	case COMMAND_CONVEYOR_RUNALL_BCK:
		conveyorMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
		conveyorMotor->Set(fConveyorSpeed);
		intakeLeftMotor->Set(-fIntakeSpeed);
		intakeRightMotor->Set(fIntakeSpeed);
		break;

	case COMMAND_CONVEYOR_RUNALL_STOP:
		conveyorMotor->Set(0.0);
		intakeLeftMotor->Set(0.0);
		intakeRightMotor->Set(0.0);
		break;

	case COMMAND_CONVEYOR_CANADJUST_BOTH:
		SmartDashboard::PutString("Conveyor CMD",
				"COMMAND_CONVEYOR_CANADJUST_BOTH");
		conveyorMotor->Set(fConveyorSpeed);
		intakeLeftMotor->Set(fAdjustSpeed);
		intakeRightMotor->Set(-fAdjustSpeed);
		break;

	case COMMAND_CONVEYOR_CANADJUST_LEFT:
		SmartDashboard::PutString("Conveyor CMD",
				"COMMAND_CONVEYOR_CANADJUST_LEFT");
		conveyorMotor->Set(fConveyorSpeed);
		intakeLeftMotor->Set(fAdjustSpeed);
		intakeRightMotor->Set(fIntakeSpeed);
		break;

	case COMMAND_CONVEYOR_CANADJUST_RIGHT:
		SmartDashboard::PutString("Conveyor CMD",
				"COMMAND_CONVEYOR_CANADJUST_RIGHT");
		conveyorMotor->Set(fConveyorSpeed);
		intakeLeftMotor->Set(-fIntakeSpeed);
		intakeRightMotor->Set(-fAdjustSpeed);
		break;
	case COMMAND_SYSTEM_MSGTIMEOUT:
		SmartDashboard::PutString("Conveyor CMD",
				"COMMAND_SYSTEM_MSGTIMEOUT");
	default:
		break;
	}
}

