/** \file
 * Implementation of class to control can lifter on the cube.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the cube's can lifter.
 *
 * The task receives messages from the main robot class and raises or lowers
 * the can lifter. Hall effect sensors  are used to stop the motion of the
 * lifter at the desired points.
 */

#include "CanLifter.h"
#include "WPILib.h"

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

CanLifter::CanLifter()
: ComponentBase(CANLIFTER_TASKNAME, CANLIFTER_QUEUE, CANLIFTER_PRIORITY)
{
	lifterMotor = new CANTalon(CAN_CUBE_BIN_LIFT);
	wpi_assert(lifterMotor);
	lifterMotor->SetVoltageRampRate(120.0);
	lifterMotor->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

	wpi_assert(lifterMotor->IsAlive());

	pTask = new Task(CANLIFTER_TASKNAME, (FUNCPTR) &CanLifter::StartTask,
			CANLIFTER_PRIORITY, CANLIFTER_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int)this);
};

CanLifter::~CanLifter()
{
	delete(pTask);
};

void CanLifter::OnStateChange()
{
	switch (localMessage.command)
	{
		case COMMAND_ROBOT_STATE_AUTONOMOUS:
			lifterMotor->Set(0.0);
			break;

		case COMMAND_ROBOT_STATE_TEST:
			lifterMotor->Set(0.0);
			break;

		case COMMAND_ROBOT_STATE_TELEOPERATED:
			lifterMotor->Set(0.0);
			break;

		case COMMAND_ROBOT_STATE_DISABLED:
			lifterMotor->Set(0.0);
			break;

		case COMMAND_ROBOT_STATE_UNKNOWN:
			lifterMotor->Set(0.0);
			break;

		default:
			lifterMotor->Set(0.0);
			break;
	}
};

void CanLifter::Run()
{
	switch(localMessage.command)
	{
		case COMMAND_CANLIFTER_RAISE:
			lifterMotor->Set(1.0);
			break;

		case COMMAND_CANLIFTER_LOWER:
			lifterMotor->Set(-1.0);
			break;

		case COMMAND_CANLIFTER_STOP:
			lifterMotor->Set(0.0);
			break;

		default:
			break;
		}

	//TODO: add timeout support for lifter motor just in case the sensors fail


};
