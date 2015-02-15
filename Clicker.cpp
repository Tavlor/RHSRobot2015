/**  Implementation of class to control functions on the cube.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the cube's components.
 *
 * The task receives messages from the main robot class and controls three components:
 *
 * THE CLICKER
 * Raises or lowers the tote lifter which "clicks" into place, thus the name.
 * Hall effect sensors are used to stop the motion of the "clicker" at the desired points.
 *	Raise:negative
 *	Lower:positive
 *
 * THE CAN LIFT
 * Lifts the can onto the top of a stack of totes. It also has hall effect
 * sensors to stop motion at the top and bottom, as well as trigger other actions.
 * 	Raise: positive
 *	Lower: negative
 *
 * THE INTAKE
 * A roller which pulls totes into the cube from the chute (yes, chute). It is triggered
 * by an IR beam break sensor.
 *	Run: negative
 */
//Please DO NOT modify the +/- motor value notes above with out FULL TESTING
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
	lifterMotor->ConfigNeutralMode(
			CANSpeedController::NeutralMode::kNeutralMode_Brake);
	lifterMotor->ConfigLimitMode(
			CANSpeedController::kLimitMode_SwitchInputsOnly);

	wpi_assert(clickerMotor->IsAlive());
	wpi_assert(intakeMotor->IsAlive());
	wpi_assert(lifterMotor->IsAlive());

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

/* CANLIFTER
 * 	Raise: positive
 *	Lower: negative
 * CLICKER
 *	Raise:negative
 *	Lower:positive
 * INTAKE
 *	Run: negative
 */
void Clicker::Run() {
	switch(localMessage.command)			//Reads the message command
	{
	case COMMAND_CUBECLICKER_RAISE:
		SmartDashboard::PutString("Cube Command", "COMMAND_CUBECLICKER_RAISE");
		if(!bEnableAutoCycle) clickerMotor->Set(fClickerRaise);	// the spring will help it up
		break;

	case COMMAND_CUBECLICKER_LOWER:
		SmartDashboard::PutString("Cube Command", "COMMAND_CUBECLICKER_LOWER");
		if(!bEnableAutoCycle) clickerMotor->Set(fClickerLower);
		break;
	case COMMAND_CANLIFTER_RAISE:
		SmartDashboard::PutString("Cube Command", "COMMAND_CANLIFTER_RAISE");
		if(!bEnableAutoCycle) lifterMotor->Set(fLifterRaise);
		break;

	case COMMAND_CANLIFTER_LOWER:
		SmartDashboard::PutString("Cube Command", "COMMAND_CANLIFTER_LOWER");
		if(!bEnableAutoCycle) lifterMotor->Set(fLifterLower);
		break;
	case COMMAND_CUBEINTAKE_RUN:
		SmartDashboard::PutString("Cube Command", "COMMAND_CUBEINTAKE_RUN");
		intakeMotor->Set(fIntake);
		bAutoCubeIntake = true;
		break;

	case COMMAND_CUBEINTAKE_STOP:
		if(!bEnableAutoCycle) {
			bAutoCubeIntake = false;
			intakeMotor->Set(0.0);
		}
		break;
	case COMMAND_CUBE_STOP:
		SmartDashboard::PutString("Cube Command", "COMMAND_CUBE_STOP");
		if(!bEnableAutoCycle) {
			intakeMotor->Set(0.0);
			clickerMotor->Set(0.0);
			lifterMotor->Set(0.0);
		}
		break;
	default:
		break;
	}
	SmartDashboard::PutNumber("Lifter Motor", lifterMotor->Get());
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
	//		intakeMotor->Set(fIntake);
	//	}
	//}

	//TODO: add timeout support for clicker motor just in case the sensors fail

	//TODO: add state machine for auto cycling
	bool irsens = !intakeMotor->IsRevLimitSwitchClosed();//true if the sensor is blocked/tripped
	//bool clickerHallEffectTop = clickerMotor->IsRevLimitSwitchClosed();
	bool clickerHallEffectBottom = clickerMotor->IsFwdLimitSwitchClosed();
	bool clickerHallEffectTop = clickerMotor->IsRevLimitSwitchClosed();
	bool lifterHallEffectBottom = lifterMotor->IsRevLimitSwitchClosed();
	bool lifterHallEffectTop = lifterMotor->IsFwdLimitSwitchClosed();

	SmartDashboard::PutBoolean("Cube Autocycle", bEnableAutoCycle);
	SmartDashboard::PutBoolean("Cube IR", irsens);
	SmartDashboard::PutBoolean("Clicker @ Top", clickerHallEffectTop);
	SmartDashboard::PutBoolean("Clicker @ Bottom", clickerHallEffectBottom);
	SmartDashboard::PutBoolean("Lifter @ Top", lifterHallEffectTop);
	SmartDashboard::PutBoolean("Lifter @ Bottom", lifterHallEffectBottom);
	SmartDashboard::PutNumber("Lifter Voltage", lifterMotor->GetBusVoltage());
	SmartDashboard::PutNumber("Lifter Voltage", clickerMotor->GetBusVoltage());
	SmartDashboard::PutNumber("Lifter Current", lifterMotor->GetOutputCurrent());
	SmartDashboard::PutNumber("Clicker Current", clickerMotor->GetOutputCurrent());



	if(bEnableAutoCycle) {
		switch(clickerLastState) {

		case STATE_CLICKER_TOP:
			SmartDashboard::PutString("Cube Clicker State", "TOP");
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
			SmartDashboard::PutString("Cube Clicker State", "LOWER");
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
			SmartDashboard::PutString("Cube Clicker State", "BOTTOM");
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
			SmartDashboard::PutString("Cube Clicker State", "BOTTOMHOLD");

			if(!irsens) {
				Reset();
			}
			else if(lifterLastState == STATE_LIFTER_TOP && iNumOfTotes == 5) {
				iNumOfTotes++;
				clickerLastState = STATE_CLICKER_RAISE;
			}

			else {
				BottomHold(clickerHallEffectBottom);
			}

			break;

		case STATE_CLICKER_RAISE:
			SmartDashboard::PutString("Cube Clicker State", "RAISE");
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
		switch(lifterLastState) {
		case STATE_LIFTER_BOTTOM:
			lifterMotor->Set(0.0);
			break;
		case STATE_LIFTER_RAISE:
			if(lifterHallEffectTop) lifterLastState = STATE_LIFTER_TOP;
			lifterMotor->Set(fLifterRaise);
			break;
		case STATE_LIFTER_TOP:
			lifterMotor->Set(0.0);
			break;
		case STATE_LIFTER_LOWER:
			if(lifterHallEffectBottom) lifterLastState = STATE_LIFTER_BOTTOM;
			lifterMotor->Set(fLifterRaise);
			break;
		}
	}
	SmartDashboard::PutNumber("Number of Totes in Cube", iNumOfTotes);
}
;

void Clicker::Top() {
	clickerMotor->Set(0);
	intakeMotor->Set(fIntake);
}
void Clicker::Bottom() {
	clickerMotor->Set(0);
	intakeMotor->Set(0);
}
void Clicker::BottomHold(bool hallEffect) {
	if(hallEffect) clickerMotor->Set(.25);	//drop
	else clickerMotor->Set(0);	//release
}
void Clicker::Raise() {
	clickerMotor->Set(fClickerRaise);
	intakeMotor->Set(fIntake);
}
void Clicker::Lower() {
	clickerMotor->Set(fClickerLower);
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
