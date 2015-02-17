/** \file
 * Implementation of class to control functions on the cube.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the cube's components.
 *
 * The task receives messages from the main robot class and controls three components:
 *
 * THE CLICKER
 * Raises or lowers the tote lifter which "clicks" into place, thus the name.
 * Hall effect sensors are used to stop the motion of the "clicker" at the desired points.
 *	Raise:positive (with CIM)
 *	Lower:negative (with CIM)
 *
 * THE CAN LIFT
 * Lifts the can onto the top of a stack of totes. It also has hall effect
 * sensors to stop motion at the top and bottom, as well as trigger other actions.
 * 	Raise: negative (with CIM)
 *	Lower: negative (with CIM)
 *
 * THE INTAKE
 * A roller which pulls totes into the cube from the chute (yes, chute). It is triggered
 * by an IR beam break sensor.
 *	Run: negative (with BAG)
 */

//Please DO NOT modify the +/- motor value notes above with out FULL TESTING
#include "Cube.h"
#include "WPILib.h"

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

Cube::Cube() :
		ComponentBase(CUBE_TASKNAME, CUBE_QUEUE, CUBE_PRIORITY) {

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
	lifterLastState = STATE_LIFTER_TOP;
	bEnableAutoCycle = false;
	fClickerPaused = 0.0;
	fLifterPaused = 0.0;

	pSafetyTimer = new Timer();
	pSafetyTimer->Start();

	pTask = new Task(CUBE_TASKNAME, (FUNCPTR) &Cube::StartTask,
			CUBE_PRIORITY, CUBE_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int) this);
}

Cube::~Cube() {
	delete (pTask);
	delete clickerMotor;
	delete intakeMotor;
	delete lifterMotor;
	delete pSafetyTimer;
}
;

void Cube::OnStateChange() {
	switch(localMessage.command) {
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
	case COMMAND_ROBOT_STATE_TEST:
	case COMMAND_ROBOT_STATE_TELEOPERATED:
	case COMMAND_ROBOT_STATE_DISABLED:
	case COMMAND_ROBOT_STATE_UNKNOWN:
	default:
		intakeMotor->Set(fIntakeStop);
		clickerMotor->Set(fClickerStop);
		lifterMotor->Set(fLifterStop);
		pSafetyTimer->Reset();
		bEnableAutoCycle = false;
		break;
	}
}

void Cube::Run()
{
	printf("Cube - Run called - %f", 5);
	switch(localMessage.command)			//Reads the message command
	{
		case COMMAND_CUBECLICKER_RAISE:
			if(!bEnableAutoCycle) {
				SmartDashboard::PutString("Cube CMD", "COMMAND_CUBECLICKER_RAISE");

				clickerMotor->Set(fClickerRaise);	// the spring will help it up
				pSafetyTimer->Reset();
			}
			break;

		case COMMAND_CUBECLICKER_LOWER:
			if(!bEnableAutoCycle) {
				SmartDashboard::PutString("Cube CMD", "COMMAND_CUBECLICKER_LOWER");

				clickerMotor->Set(fClickerLower);
				pSafetyTimer->Reset();
			}
			break;

		case COMMAND_CUBECLICKER_STOP:
			if(!bEnableAutoCycle) {
				SmartDashboard::PutString("Cube CMD", "COMMAND_CUBECLICKER_STOP");

				clickerMotor->Set(fClickerStop);
				pSafetyTimer->Reset();
			}
			break;

		case COMMAND_CANLIFTER_RAISE:
			if(!bEnableAutoCycle) {
				SmartDashboard::PutString("Cube CMD", "COMMAND_CANLIFTER_RAISE");

				lifterMotor->Set(fLifterRaise);
				pSafetyTimer->Reset();
			}
			break;

		case COMMAND_CANLIFTER_LOWER:
			if(!bEnableAutoCycle) {
				SmartDashboard::PutString("Cube CMD", "COMMAND_CANLIFTER_LOWER");

				lifterMotor->Set(fLifterLower);
				pSafetyTimer->Reset();
			}
			break;

		case COMMAND_CANLIFTER_STOP:
			if(!bEnableAutoCycle) {
				SmartDashboard::PutString("Cube CMD", "COMMAND_CANLIFTER_STOP");

				lifterMotor->Set(fLifterStop);
				pSafetyTimer->Reset();
			}
			break;

		case COMMAND_CUBEINTAKE_RUN:
			if(!bEnableAutoCycle) {
				SmartDashboard::PutString("Cube CMD", "COMMAND_CUBEINTAKE_RUN");

				intakeMotor->Set(fIntakeRun);
				pSafetyTimer->Reset();
			}
			break;

		case COMMAND_CUBEINTAKE_STOP:
			if(!bEnableAutoCycle) {
				SmartDashboard::PutString("Cube CMD", "COMMAND_CUBEINTAKE_STOP");

				intakeMotor->Set(fIntakeStop);
				pSafetyTimer->Reset();
			}
			break;

		case COMMAND_CUBE_STOP:
			SmartDashboard::PutString("Cube CMD", "COMMAND_CUBE_STOP");

			if(!bEnableAutoCycle) {
				intakeMotor->Set(fIntakeStop);
				clickerMotor->Set(fClickerStop);
				lifterMotor->Set(fLifterStop);
				fClickerPaused = 0.0;
				fLifterPaused = 0.0;
				pSafetyTimer->Reset();
			}
			break;

		case COMMAND_CUBEAUTOCYCLE_START:
			if(!bEnableAutoCycle)
			{
				SmartDashboard::PutString("Cube CMD", "COMMAND_CUBEAUTOCYCLE_START");
				bEnableAutoCycle = true;
				fClickerPaused = 0.0;
				fLifterPaused = 0.0;
				clickerLastState = STATE_CLICKER_TOP;
				lifterLastState = STATE_LIFTER_LOWER;
				intakeMotor->Set(fIntakeRun);
				pSafetyTimer->Reset();
			}
			break;

		case COMMAND_CUBEAUTOCYCLE_STOP:
			if(bEnableAutoCycle)
			{
				SmartDashboard::PutString("Cube CMD", "COMMAND_CUBEAUTOCYCLE_STOP");
				bEnableAutoCycle = false;
				intakeMotor->Set(fIntakeStop);
				clickerMotor->Set(fClickerStop);
				lifterMotor->Set(fLifterStop);
				pSafetyTimer->Reset();
			}
			break;

		case COMMAND_CUBEAUTOCYCLE_PAUSE:
			if(bEnableAutoCycle)
			{
				SmartDashboard::PutString("Cube CMD", "COMMAND_CUBEAUTOCYCLE_PAUSE");
				bEnableAutoCycle = false;
				fClickerPaused = intakeMotor->Get();
				fLifterPaused = lifterMotor->Get();
				intakeMotor->Set(fIntakeStop);
				clickerMotor->Set(fClickerStop);
				lifterMotor->Set(fLifterStop);
				pSafetyTimer->Reset();
			}
			break;

		case COMMAND_CUBEAUTOCYCLE_RESUME:
			if(bEnableAutoCycle) {
				SmartDashboard::PutString("Cube CMD", "COMMAND_CUBEAUTOCYCLE_RESUME");

				clickerMotor->Set(fClickerPaused);
				lifterMotor->Set(fLifterPaused);
				intakeMotor->Set(fIntakeRun);
				bEnableAutoCycle = true;
				pSafetyTimer->Reset();
			}
			break;

		case COMMAND_SYSTEM_MSGTIMEOUT:
			SmartDashboard::PutString("Cube CMD", "COMMAND_SYSTEM_MSGTIMEOUT");
			break;

		default:
			break;
	}	//end of command switch

	if(pSafetyTimer->Get() > 30.0)
	{
		intakeMotor->Set(fIntakeStop);
		clickerMotor->Set(fClickerStop);
		lifterMotor->Set(fLifterStop);
		pSafetyTimer->Reset();
	}

	SmartDashboard::PutBoolean("Cube Autocycle", bEnableAutoCycle);

	//Autocycle loop

	if(bEnableAutoCycle) {
		bool irBlocked = !intakeMotor->IsRevLimitSwitchClosed();
		bool clickerHallEffectBottom = clickerMotor->IsRevLimitSwitchClosed();
		bool clickerHallEffectTop = clickerMotor->IsFwdLimitSwitchClosed();
		bool lifterHallEffectBottom = lifterMotor->IsFwdLimitSwitchClosed();
		bool lifterHallEffectTop = lifterMotor->IsRevLimitSwitchClosed();

		switch(clickerLastState) {

		case STATE_CLICKER_TOP:
			SmartDashboard::PutString("Cube Clicker State", "TOP");
			if(irBlocked) {
				clickerLastState = STATE_CLICKER_LOWER;
				clickerMotor->Set(fClickerLower);
			}
			else {
				clickerLastState = STATE_CLICKER_TOP;
				clickerMotor->Set(0);
			}
			break;

		case STATE_CLICKER_LOWER:
			SmartDashboard::PutString("Cube Clicker State", "LOWER");
			if(!clickerHallEffectBottom) {
				clickerLastState = STATE_CLICKER_LOWER;
				clickerMotor->Set(fClickerLower);
			}
			else {
				clickerLastState = STATE_CLICKER_BOTTOM;
				clickerMotor->Set(0);
			}
			break;

		case STATE_CLICKER_BOTTOM:
			SmartDashboard::PutString("Cube Clicker State", "BOTTOM");
			iNumOfTotes++;
			//printf("There are now %i totes in the cube.\n", iNumOfTotes);
			if(iNumOfTotes == 5) {//waits until can is on top - can hall effect
				clickerLastState = STATE_CLICKER_BOTTOMHOLD;
				lifterLastState = STATE_LIFTER_RAISE;

			}
			else if(iNumOfTotes == 6) {	//waits until totes removed - IR
				clickerLastState = STATE_CLICKER_BOTTOMHOLD;
			}
			else {
				clickerLastState = STATE_CLICKER_RAISE;
				clickerMotor->Set(fClickerRaise);
			}
			break;

		case STATE_CLICKER_BOTTOMHOLD:
			SmartDashboard::PutString("Cube Clicker State", "BOTTOMHOLD");

			if(!irBlocked && iNumOfTotes == 6) {	//if all totes removed
				//printf("Totes removed from cube.\n");
				//StopAuto();
			}
			else if(lifterLastState == STATE_LIFTER_TOP && iNumOfTotes == 5) {
				//printf("Can is at the top.\n");
				clickerLastState = STATE_CLICKER_RAISE;
			}
			else {
				clickerMotor->Set(fClickerLower);
			}
			break;

		case STATE_CLICKER_RAISE:
			SmartDashboard::PutString("Cube Clicker State", "RAISE");
			if(!clickerHallEffectTop) {
				clickerLastState = STATE_CLICKER_RAISE;
				clickerMotor->Set(fClickerRaise);
			}
			else {
				clickerLastState = STATE_CLICKER_TOP;
				clickerMotor->Set(0);
			}
			break;
		}	//End of clicker state machine

		//Begin of lifter state machine
		switch(lifterLastState) {
		case STATE_LIFTER_BOTTOM:
			SmartDashboard::PutString("Cube Lifter State", "BOTTOM");
			lifterMotor->Set(0.0);
			break;
		case STATE_LIFTER_RAISE:
			SmartDashboard::PutString("Cube Lifter State", "RAISE");

			if(lifterHallEffectTop)
			{
				lifterLastState = STATE_LIFTER_TOP;
			}

			lifterMotor->Set(fLifterRaise);
			break;
		case STATE_LIFTER_TOP:
			SmartDashboard::PutString("Cube Lifter State", "TOP");

			lifterMotor->Set(0.0);
			break;
		case STATE_LIFTER_LOWER:
			SmartDashboard::PutString("Cube Lifter State", "LOWER");

			if(lifterHallEffectBottom)
			{
				lifterLastState = STATE_LIFTER_BOTTOM;
			}

			lifterMotor->Set(fLifterLower);
			break;
		}	//End of lifter state machine
	}

	SmartDashboard::PutNumber("Number of Totes in Cube", iNumOfTotes);
}
