/**  Implementation of class to control tote lifter on the pallet jack.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the pallet jack's tote lifter.
 *
 * The task receives messages form the main robot class and raises or lowers
 * the tote lifter which "clicks" into place, thus the name.  Hall effect sensors
 * are used to stop the motion of the "clicker" at the desired points.
 */

#include "JackClicker.h"
#include "WPILib.h"

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

JackClicker::JackClicker()
: ComponentBase(JACKCLICKER_TASKNAME, JACKCLICKER_QUEUE, JACKCLICKER_PRIORITY)
{
	clickerMotor = new CANTalon(CAN_PALLET_JACK_TOTE_LIFT);
	wpi_assert(clickerMotor);
	clickerMotor->SetVoltageRampRate(120.0);
	clickerMotor->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	clickerMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

	pTask = new Task(JACKCLICKER_TASKNAME, (FUNCPTR) &JackClicker::StartTask,
			JACKCLICKER_PRIORITY, JACKCLICKER_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int)this);
};

JackClicker::~JackClicker()
{
	delete(pTask);
};

void JackClicker::OnStateChange()
{
	switch (localMessage.command)
	{
		case COMMAND_ROBOT_STATE_AUTONOMOUS:
			clickerMotor->Set(0.0);
			break;

		case COMMAND_ROBOT_STATE_TEST:
			clickerMotor->Set(0.0);
			break;

		case COMMAND_ROBOT_STATE_TELEOPERATED:
			clickerMotor->Set(0.0);
			break;

		case COMMAND_ROBOT_STATE_DISABLED:
			clickerMotor->Set(0.0);
			break;

		case COMMAND_ROBOT_STATE_UNKNOWN:
			clickerMotor->Set(0.0);
			break;

		default:
			clickerMotor->Set(0.0);
			break;
	}
};

void JackClicker::Run()
{
	switch(localMessage.command)
	{
		case COMMAND_JACKCLICKER_RAISE:
			clickerMotor->Set(0.10);		// the spring will help it up
			break;

		case COMMAND_JACKCLICKER_LOWER:
			clickerMotor->Set(-1.0);
			break;

		case COMMAND_JACKCLICKER_STOP:
			clickerMotor->Set(0.0);
			break;

		default:
			break;
		}

	//TODO: add timeout support for tote motor just in case the sensors fail


};
