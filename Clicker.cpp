/**  Implementation of class to control tote lifter on the cube.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the cube's tote lifter.
 *
 * The task receives messages from the main robot class and raises or lowers
 * the tote lifter which "clicks" into place, thus the name. Hall effect sensors
 * are used to stop the motion of the "clicker" at the desired points.
 */

#include "WPILib.h"

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

//Local
#include "Clicker.h"

Clicker::Clicker()
: ComponentBase(CLICKER_TASKNAME, CLICKER_QUEUE, CLICKER_PRIORITY)
{
	bEnableAutoCycle = false;
	bAutoCubeIntake = false;

	// run the clicker motor in braking mode till it hits a limit switch

	clickerMotor = new CANTalon(CAN_CUBE_CLICKER);
	wpi_assert(clickerMotor);
	clickerMotor->SetVoltageRampRate(120.0);
	clickerMotor->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	clickerMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

	// run the intake motor unless a tote has broken the beam

	intakeMotor = new CANTalon(CAN_CUBE_INTAKE);
	//intakeMotor->ConfigRevLimitSwitchNormallyOpen(false);
	//intakeMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

	pTask = new Task(CLICKER_TASKNAME, (FUNCPTR) &Clicker::StartTask,
			CLICKER_PRIORITY, CLICKER_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int)this);
};

Clicker::~Clicker()
{
	delete(pTask);
};

void Clicker::OnStateChange()
{
	switch (localMessage.command)
	{
		case COMMAND_ROBOT_STATE_AUTONOMOUS:
			clickerMotor->Set(0.0);
			intakeMotor->Set(0.0);
			bEnableAutoCycle = false;
			break;

		case COMMAND_ROBOT_STATE_TEST:
			clickerMotor->Set(0.0);
			intakeMotor->Set(0.0);
			bEnableAutoCycle = false;
			break;

		case COMMAND_ROBOT_STATE_TELEOPERATED:
			clickerMotor->Set(0.0);
			intakeMotor->Set(0.0);
			bEnableAutoCycle = false;
			break;

		case COMMAND_ROBOT_STATE_DISABLED:
			clickerMotor->Set(0.0);
			intakeMotor->Set(0.0);
			bEnableAutoCycle = false;
			break;

		case COMMAND_ROBOT_STATE_UNKNOWN:
			clickerMotor->Set(0.0);
			intakeMotor->Set(0.0);
			bEnableAutoCycle = false;
			break;

		default:
			clickerMotor->Set(0.0);
			intakeMotor->Set(0.0);
			bEnableAutoCycle = false;
			break;
	}
};

void Clicker::Run()
{
	switch(localMessage.command)			//Reads the message command
	{
		case COMMAND_CUBECLICKER_RAISE:
			clickerMotor->Set(0.25);		// the spring will help it up
			break;

		case COMMAND_CUBECLICKER_LOWER:
			clickerMotor->Set(-1.0);
			break;

		case COMMAND_CUBECLICKER_STOP:
			clickerMotor->Set(0.0);
			break;

		case COMMAND_CUBEINTAKE_RUN:
			bAutoCubeIntake = true;
			break;

		case COMMAND_CUBEINTAKE_STOP:
			bAutoCubeIntake = false;
			break;

		case COMMAND_CUBEAUTOCYCLE_START:
			bEnableAutoCycle = true;
			break;

		case COMMAND_CUBEAUTOCYCLE_STOP:
			bEnableAutoCycle = false;
			break;

		default:
			break;
		}

	// this should work when done from the Talon but it does not - weird

	if(bAutoCubeIntake)
	{
		if(intakeMotor->IsRevLimitSwitchClosed())
		{
			intakeMotor->Set(-0.50);
		}
		else
		{
			intakeMotor->Set(-0.50);
		}
	}

	//TODO: add timeout support for clicker motor just in case the sensors fail

	//TODO: add state machine for auto cycling

};
