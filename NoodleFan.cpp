/** \file
 * Implementation of class to control noodle fan.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the device used to control the anti-noodle fan.
 *
 * Spins a fan to blow away dangerous noodles. The driver can toggle its behavior.
 *
 */

#include <NoodleFan.h>
//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

NoodleFan::NoodleFan() :
		ComponentBase(NOODLEFAN_TASKNAME, NOODLEFAN_QUEUE, NOODLEFAN_PRIORITY) {

	fanMotor = new CANTalon(CAN_PALLET_JACK_NOODLE_FAN);
	wpi_assert(fanMotor);
	fanMotor->SetVoltageRampRate(120.0);

	fanMotor->ConfigNeutralMode(
			CANSpeedController::NeutralMode::kNeutralMode_Brake);

	wpi_assert(fanMotor->IsAlive());
	pTask = new Task(NOODLEFAN_TASKNAME, (FUNCPTR) &NoodleFan::StartTask,
			NOODLEFAN_PRIORITY, NOODLEFAN_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int) this);
}

NoodleFan::~NoodleFan() {
	delete pTask;
	delete fanMotor;
}

void NoodleFan::OnStateChange() {
	switch (localMessage.command)
	{
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
		fanMotor->Set(0);
		bBlowing = false;
		break;

	case COMMAND_ROBOT_STATE_TEST:
		fanMotor->Set(0);
		bBlowing = false;
		break;

	case COMMAND_ROBOT_STATE_TELEOPERATED:
		fanMotor->Set(0);
		bBlowing = false;
		break;

	case COMMAND_ROBOT_STATE_DISABLED:
		fanMotor->Set(0);
		bBlowing = false;
		break;

	case COMMAND_ROBOT_STATE_UNKNOWN:
		fanMotor->Set(0);
		bBlowing = false;
		break;

	default:
		fanMotor->Set(0);
		bBlowing = false;
		break;
	}
}

void NoodleFan::Run() {
	switch (localMessage.command)
	{
	case COMMAND_NOODLEFAN_START:
		//Noodles == RobotDeath; get rid of them!
		bBlowing = true;
		fanMotor->Set(fPower);
		break;

	case COMMAND_NOODLEFAN_STOP:
		bBlowing = false;
		fanMotor->Set(0);
		break;

	case COMMAND_NOODLEFAN_TOGGLE:
		bBlowing = !bBlowing;
		//will be set below
		break;

	case COMMAND_SYSTEM_MSGTIMEOUT:
		break;

	default:
		break;
	}

	if (bBlowing)
	{
		fanMotor->Set(fPower);
	}
	else
	{
		fanMotor->Set(0);
	}
}

