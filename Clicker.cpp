/**  Implementation of class to control tote lifter on the cube.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the cube's tote lifter.
 *
 * The task receives messages from the main robot class and raises or lowers
 * the tote lifter which "clicks" into place, thus the name. Hall effect sensors
 * are used to stop the motion of the "clicker" at the desired points.
 */

#include "Clicker.h"
#include "WPILib.h"

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

Clicker::Clicker() :
		ComponentBase(CLICKER_TASKNAME, CLICKER_QUEUE, CLICKER_PRIORITY) {
	bEnableAutoCycle = false;
	bAutoCubeIntake = false;

	// run the clicker motor in braking mode till it hits a limit switch

	clickerMotor = new CANTalon(CAN_CUBE_CLICKER);
	wpi_assert(clickerMotor);
	clickerMotor->SetVoltageRampRate(120.0);
	clickerMotor->ConfigNeutralMode(
			CANSpeedController::NeutralMode::kNeutralMode_Brake);
	clickerMotor->ConfigLimitMode(
			CANSpeedController::kLimitMode_SwitchInputsOnly);

	// run the intake motor unless a tote has broken the beam

	intakeMotor = new CANTalon(CAN_CUBE_INTAKE);
	intakeMotor->ConfigRevLimitSwitchNormallyOpen(false);
	intakeMotor->ConfigLimitMode(
			CANSpeedController::kLimitMode_SwitchInputsOnly);

	lifterMotor = new CANTalon(CAN_CUBE_BIN_LIFT);
	wpi_assert(lifterMotor);
	lifterMotor->SetVoltageRampRate(120.0);
	lifterMotor->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

	clickerLastState = STATE_CLICKER_TOP;
	lifterLastState = STATE_LIFTER_BOTTOM;

	pTask = new Task(CLICKER_TASKNAME, (FUNCPTR) &Clicker::StartTask,
			CLICKER_PRIORITY, CLICKER_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int) this);
}
;

Clicker::~Clicker() {
	delete (pTask);
	delete clickerMotor;
	delete intakeMotor;
	delete lifterMotor;
}
;

void Clicker::OnStateChange() {
	switch(localMessage.command) {
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
}
;

void Clicker::Run() {
	switch(localMessage.command)			//Reads the message command
	{
	case COMMAND_CUBECLICKER_RAISE:
		if(!bEnableAutoCycle) clickerMotor->Set(0.25);// the spring will help it up
		break;

	case COMMAND_CUBECLICKER_LOWER:
		if(!bEnableAutoCycle) clickerMotor->Set(-1.0);
		break;

	case COMMAND_CUBECLICKER_STOP:
		if(!bEnableAutoCycle) clickerMotor->Set(0.0);
		break;

	case COMMAND_CUBEINTAKE_RUN:
		if(!bEnableAutoCycle) bAutoCubeIntake = true;
		break;

	case COMMAND_CUBEINTAKE_STOP:
		if(!bEnableAutoCycle) bAutoCubeIntake = false;
		break;

	case COMMAND_CUBEAUTOCYCLE_START:
		bEnableAutoCycle = true;
		clickerLastState = STATE_CLICKER_TOP;
		lifterLastState = STATE_LIFTER_BOTTOM;
		Top();
		break;

	case COMMAND_CUBEAUTOCYCLE_STOP:
		bEnableAutoCycle = false;
		clickerLastState = STATE_CLICKER_TOP;
		lifterLastState = STATE_LIFTER_BOTTOM;
		break;

	default:
		break;
	}
	SmartDashboard::PutBoolean("Cube Intake Toggle", bAutoCubeIntake);
	// Backup
	//if(bAutoCubeIntake)
	//{
	//	if(intakeMotor->IsRevLimitSwitchClosed())
	//	{
	//		intakeMotor->Set(0.0);
	//	}
	//	else
	//	{
	//		intakeMotor->Set(-0.50);
	//	}
	//}

	//TODO: add timeout support for clicker motor just in case the sensors fail

	//TODO: add state machine for auto cycling
	bool irsens = !intakeMotor->IsRevLimitSwitchClosed();//true if the sensor is blocked/tripped
	//bool clickerHallEffectTop = clickerMotor->IsRevLimitSwitchClosed();
	bool clickerHallEffectBottom = clickerMotor->IsRevLimitSwitchClosed();
	bool clickerHallEffectTop = clickerMotor->IsFwdLimitSwitchClosed();
	bool lifterHallEffectBottom = lifterMotor->IsRevLimitSwitchClosed();
	bool lifterHallEffectTop = lifterMotor->IsFwdLimitSwitchClosed();

	SmartDashboard::PutBoolean("IR", irsens);
	SmartDashboard::PutBoolean("TopClick", clickerHallEffectTop);
	SmartDashboard::PutBoolean("BottomCLick", clickerHallEffectBottom);

	switch(clickerLastState) {
	case STATE_CLICKER_BOTTOM:
		SmartDashboard::PutString("CUBE CLICKER STATE", "BOTTOM");
		break;
	case STATE_CLICKER_TOP:
		SmartDashboard::PutString("CUBE CLICKER STATE", "TOP");
		break;
	case STATE_CLICKER_LOWER:
		SmartDashboard::PutString("CUBE CLICKER STATE", "LOWER");
		break;
	case STATE_CLICKER_RAISE:
		SmartDashboard::PutString("CUBE CLICKER STATE", "RAISE");
		break;
	}

	if(bEnableAutoCycle) {
		switch(clickerLastState) {

		case STATE_CLICKER_TOP:
			if(irsens) {
				clickerLastState = STATE_CLICKER_LOWER;
				Lower();
			}
			else {
				clickerLastState = STATE_CLICKER_TOP;
				//Top();
			}
			break;

		case STATE_CLICKER_LOWER:
			if(!clickerHallEffectBottom) {
				clickerLastState = STATE_CLICKER_LOWER;
				//Lower();
			}
			else {
				clickerLastState = STATE_CLICKER_BOTTOM;
				Bottom();
			}
			break;

		case STATE_CLICKER_BOTTOM:
			if(iNumOfTotes == CUBECLICKER_MAX_TOTES - 1) {//waits until can is on top - can hall effect
				clickerLastState = STATE_CLICKER_BOTTOMHOLD;
				lifterLastState = STATE_LIFTER_RAISE;

			}
			if(iNumOfTotes == CUBECLICKER_MAX_TOTES) {//waits until totes removed - IR
				clickerLastState = STATE_CLICKER_BOTTOMHOLD;
			}
			else {
				iNumOfTotes++;
				clickerLastState = STATE_CLICKER_RAISE;
				Raise();
			}
			break;

		case STATE_CLICKER_BOTTOMHOLD:

			if(!irsens) {
				Reset();
			}
			else if (lifterLastState == STATE_LIFTER_TOP && iNumOfTotes == 5){
				iNumOfTotes++;
				clickerLastState = STATE_CLICKER_RAISE;
			}

			else
			{
				//TODO add bottom hold functions
			}

			break;

		case STATE_CLICKER_RAISE:
			if(!clickerHallEffectTop) {
				clickerLastState = STATE_CLICKER_RAISE;
				//Raise();
			}
			else {
				clickerLastState = STATE_CLICKER_TOP;
				Top();
			}
			break;
		}
	}
	switch(lifterLastState) {
	case STATE_LIFTER_BOTTOM:
		lifterMotor->Set(0.0);
		break;
	case STATE_LIFTER_RAISE:
		if(lifterHallEffectTop)
			lifterLastState = STATE_LIFTER_TOP;
		lifterMotor->Set(-0.3);
		break;
	case STATE_LIFTER_TOP:
		lifterMotor->Set(0.0);
		break;
	case STATE_LIFTER_LOWER:
		if(lifterHallEffectBottom)
			lifterLastState = STATE_LIFTER_BOTTOM;
		lifterMotor->Set(0.3);
		break;

	}
	SmartDashboard::PutNumber("Number of Totes in Cube", iNumOfTotes);
}
;

void Clicker::Top() {
	clickerMotor->Set(0);
	intakeMotor->Set(-0.5);
}
void Clicker::Bottom() {
	clickerMotor->Set(0);
	intakeMotor->Set(0);
}
void Clicker::Raise() {
	clickerMotor->Set(0.25);
	intakeMotor->Set(-0.5);
}
void Clicker::Lower() {
	clickerMotor->Set(-1.0);
	intakeMotor->Set(0);
}
void Clicker::Reset() {
	iNumOfTotes = 0;
	if(clickerLastState != STATE_CLICKER_TOP) {
		clickerLastState = STATE_CLICKER_RAISE;
	}
	if(lifterLastState != STATE_LIFTER_BOTTOM) {
		lifterLastState = STATE_LIFTER_LOWER;
	}
}
