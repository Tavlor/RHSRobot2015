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
	//clickerMotor->SetVoltageRampRate(120.0);
	clickerMotor->ConfigNeutralMode(
			CANSpeedController::NeutralMode::kNeutralMode_Brake);
	clickerMotor->ConfigLimitMode(
			CANSpeedController::kLimitMode_SwitchInputsOnly);

	// run the intake motor unless a tote has broken the beam

	intakeMotor = new CANTalon(CAN_CUBE_INTAKE);
	wpi_assert(intakeMotor);
	intakeMotor->ConfigRevLimitSwitchNormallyOpen(false);
	intakeMotor->ConfigLimitMode(
			CANSpeedController::kLimitMode_SwitchInputsOnly);

	lifterMotor = new CANTalon(CAN_CUBE_BIN_LIFT);
	wpi_assert(lifterMotor);
	//lifterMotor->SetVoltageRampRate(120.0);
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
	bOkToRaiseCan = false;
	fClickerPaused = 0.0;
	fLifterPaused = 0.0;

	irBlocked = !intakeMotor->IsRevLimitSwitchClosed();
	clickerHallEffectBottom = clickerMotor->IsRevLimitSwitchClosed();
	clickerHallEffectTop = clickerMotor->IsFwdLimitSwitchClosed();
	lifterHallEffectBottom = lifterMotor->IsFwdLimitSwitchClosed();
	lifterHallEffectTop = lifterMotor->IsRevLimitSwitchClosed();

	pSafetyTimer = new Timer();
	pSafetyTimer->Start();
	pAutoTimer = new Timer();
	pAutoTimer->Start();
	pRemoteUpdateTimer = new Timer();
	pRemoteUpdateTimer->Start();
	pInterCycleTimer = new Timer();
	pInterCycleTimer->Start();

	pTask = new Task(CUBE_TASKNAME, (FUNCPTR) &Cube::StartTask, CUBE_PRIORITY,
			CUBE_STACKSIZE);
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
		intakeMotor->Set(fIntakeRun);//should always run in teleop
		break;
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

void Cube::Run() {
	switch(localMessage.command)			//Reads the message command
	{
	case COMMAND_CUBECLICKER_RAISE:
		if(!bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Clicker Operation", "RAISE");
			//SmartDashboard::PutString("Cube CMD", "CUBECLICKER_RAISE");

			clickerMotor->Set(fClickerRaise);	// the spring will help it up
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBECLICKER_LOWER:
		if(!bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Clicker Operation", "LOWER");
			//SmartDashboard::PutString("Cube CMD", "CUBECLICKER_LOWER");

			clickerMotor->Set(fClickerLower);
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBECLICKER_STOP:
		if(!bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Clicker Operation", "STOP");
			//SmartDashboard::PutString("Cube CMD", "CUBECLICKER_STOP");

			clickerMotor->Set(fClickerStop);
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CANLIFTER_RAISE:
		if(!bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Lifter Operation", "RAISE");
			//SmartDashboard::PutString("Cube CMD", "CANLIFTER_RAISE");

			lifterMotor->Set(fLifterRaise);
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CANLIFTER_LOWER:
		if(!bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Lifter Operation", "LOWER");
			//SmartDashboard::PutString("Cube CMD", "CANLIFTER_LOWER");

			lifterMotor->Set(fLifterLower);
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CANLIFTER_STOP:
		if(!bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Lifter Operation", "STOP");
			//SmartDashboard::PutString("Cube CMD", "CANLIFTER_STOP");

			lifterMotor->Set(fLifterStop);
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBEINTAKE_RUN:
		if(!bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Intake Operation", "RUN");
			//SmartDashboard::PutString("Cube CMD", "CUBEINTAKE_RUN");

			intakeMotor->Set(fIntakeRun);
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBEINTAKE_STOP:
		if(!bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Intake Operation", "STOP");
			//SmartDashboard::PutString("Cube CMD", "CUBEINTAKE_STOP");

			//intakeMotor->Set(fIntakeStop);
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBE_STOP:
		//SmartDashboard::PutString("Cube CMD", "CUBE_STOP");

		if(!bEnableAutoCycle)
		{
			//intakeMotor->Set(fIntakeStop);
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
			//SmartDashboard::PutString("Cube CMD", "CUBEAUTOCYCLE_START");
			bEnableAutoCycle = true;
			bOkToRaiseCan = false;
			fClickerPaused = 0.0;
			fLifterPaused = 0.0;
			clickerLastState = STATE_CLICKER_RAISE;
			lifterLastState = STATE_LIFTER_LOWER;
			intakeMotor->Set(fIntakeRun);
			iNumOfTotes = 0;
			SmartDashboard::PutNumber("Number of Totes in Cube", iNumOfTotes);
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBEAUTOCYCLE_STOP:
		if(bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Cube CMD", "CUBEAUTOCYCLE_STOP");
			bEnableAutoCycle = false;
			bOkToRaiseCan = false;
			//intakeMotor->Set(fIntakeStop);
			clickerMotor->Set(fClickerStop);
			lifterMotor->Set(fLifterStop);
			iNumOfTotes = 0;
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBEAUTOCYCLE_PAUSE:
		if(bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Cube CMD", "CUBEAUTOCYCLE_PAUSE");
			bEnableAutoCycle = false;
			fClickerPaused = clickerMotor->Get();
			fLifterPaused = lifterMotor->Get();
			//intakeMotor->Set(fIntakeStop);
			clickerMotor->Set(fClickerStop);
			lifterMotor->Set(fLifterStop);
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBEAUTOCYCLE_RESUME:
		if(bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Cube CMD", "CUBEAUTOCYCLE_RESUME");
			clickerMotor->Set(fClickerPaused);
			lifterMotor->Set(fLifterPaused);
			intakeMotor->Set(fIntakeRun);
			bEnableAutoCycle = true;
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBEAUTOCYCLE_OKTORAISECAN:
		bOkToRaiseCan = true;
		break;

	case COMMAND_CUBEAUTOCYCLE_INCREMENT_COUNT:
		if((iNumOfTotes >= 0) && (iNumOfTotes <= 4))
		{
			iNumOfTotes++;
		}
		SmartDashboard::PutNumber("Number of Totes in Cube", iNumOfTotes);
		break;

	case COMMAND_CUBEAUTOCYCLE_DECREMENT_COUNT:
		if((iNumOfTotes > 0) && (iNumOfTotes <= 4))
		{
			iNumOfTotes--;
		}
		SmartDashboard::PutNumber("Number of Totes in Cube", iNumOfTotes);
		break;

	case COMMAND_SYSTEM_MSGTIMEOUT:
		//SmartDashboard::PutString("Cube CMD", "SYSTEM_MSGTIMEOUT");
		break;

	default:
		break;
	}	//end of command switch

	if(pSafetyTimer->Get() > 30.0)
	{
		//intakeMotor->Set(fIntakeStop);
		clickerMotor->Set(fClickerStop);
		lifterMotor->Set(fLifterStop);
		pSafetyTimer->Reset();
	}

	// update the remote indicators periodically so we do not create too much CAN traffic

	if((pRemoteUpdateTimer->Get() > 0.5) && !bEnableAutoCycle)
	{
		pRemoteUpdateTimer->Reset();
		irBlocked = !intakeMotor->IsRevLimitSwitchClosed();
		clickerHallEffectBottom = clickerMotor->IsRevLimitSwitchClosed();
		clickerHallEffectTop = clickerMotor->IsFwdLimitSwitchClosed();
		lifterHallEffectBottom = lifterMotor->IsFwdLimitSwitchClosed();
		lifterHallEffectTop = lifterMotor->IsRevLimitSwitchClosed();
		SmartDashboard::PutBoolean("Cube IR", irBlocked);
		SmartDashboard::PutBoolean("Lifter @ Top", lifterHallEffectTop);
		SmartDashboard::PutBoolean("Lifter @ Bottom", lifterHallEffectBottom);
		SmartDashboard::PutBoolean("Clicker @ Top", clickerHallEffectTop);
		SmartDashboard::PutBoolean("Clicker @ Bottom", clickerHallEffectBottom);
		//SmartDashboard::PutNumber("Clicker Voltage", clickerMotor->GetBusVoltage());
		//SmartDashboard::PutNumber("Lifter Voltage", lifterMotor->GetBusVoltage());
		SmartDashboard::PutBoolean("Cube Autocycle", bEnableAutoCycle);
	}

	// run the state machine at a regular rate so we do not create too much CAN traffic

	if(bEnableAutoCycle && (pAutoTimer->Get() > 0.02))
	{
		pAutoTimer->Reset();
		pSafetyTimer->Reset();
		//Voltage reporting
		//SmartDashboard::PutNumber("Clicker Voltage", clickerMotor->GetBusVoltage());
		//SmartDashboard::PutNumber("Lifter Voltage", lifterMotor->GetBusVoltage());
		SmartDashboard::PutBoolean("Cube Autocycle", bEnableAutoCycle);

		switch(clickerLastState) {

		case STATE_CLICKER_TOP:
			//SmartDashboard::PutString("Cube Clicker State", "TOP");
			irBlocked = !intakeMotor->IsRevLimitSwitchClosed();
			SmartDashboard::PutBoolean("Cube IR", irBlocked);

			if(irBlocked)
			{
				// if the beam is broken, lower the clicker

				if(++iNumOfTotes == 5)
				{
					// let the can lifter raise if the driver hits a key

					lifterLastState = STATE_LIFTER_WAITTILLRAISE;
					bOkToRaiseCan = false;
				}

				SmartDashboard::PutNumber("Number of Totes in Cube", iNumOfTotes);

				clickerLastState = STATE_CLICKER_LOWER;
				clickerMotor->Set(fClickerLower);
			}
			else
			{
				// the beam is open, keep the clicker up (must apply power)
				clickerLastState = STATE_CLICKER_TOP;
				clickerMotor->Set(fClickerTopHold);
			}
			break;

		case STATE_CLICKER_LOWER:
			//SmartDashboard::PutString("Cube Clicker State", "LOWER");
			clickerHallEffectBottom = clickerMotor->IsRevLimitSwitchClosed();
			SmartDashboard::PutBoolean("Clicker @ Bottom", clickerHallEffectBottom);

			if(!clickerHallEffectBottom)
			{
				// we have not yet reached the bottom, leave the motor on

				clickerLastState = STATE_CLICKER_LOWER;
				clickerMotor->Set(fClickerLower);
			}
			else
			{
				// we are at the bottom, turn the motor off

				clickerLastState = STATE_CLICKER_BOTTOM;
				clickerMotor->Set(0);
			}
			break;

		case STATE_CLICKER_BOTTOM:
			//SmartDashboard::PutString("Cube Clicker State", "BOTTOM");

			if(iNumOfTotes == 5)
			{
				//waits until can is on top - can hall effect

				clickerLastState = STATE_CLICKER_BOTTOMHOLD;
			}
			else if(iNumOfTotes == 6)
			{
				//waits until totes removed - IR

				clickerLastState = STATE_CLICKER_BOTTOMHOLD;
			}
			else
			{
				//  raise clicker so we can load another tote

				clickerLastState = STATE_CLICKER_RAISE;
				clickerMotor->Set(fClickerRaise);
			}
			break;

		case STATE_CLICKER_BOTTOMHOLD:
			//SmartDashboard::PutString("Cube Clicker State", "BOTTOMHOLD");
			irBlocked = !intakeMotor->IsRevLimitSwitchClosed();
			SmartDashboard::PutBoolean("Cube IR", irBlocked);

			if(!irBlocked && iNumOfTotes == 6)
			{
				// if all totes removed, start again

				iNumOfTotes = 0;
				SmartDashboard::PutNumber("Number of Totes in Cube", iNumOfTotes);
				clickerLastState = STATE_CLICKER_DELAYAFTERCYLE;
				pInterCycleTimer->Reset();
			}
			else if((lifterLastState == STATE_LIFTER_TOP) && (iNumOfTotes == 5))
			{
				clickerLastState = STATE_CLICKER_RAISE;
			}
			else
			{
				// leave clicker down

				clickerMotor->Set(fClickerLower);
			}
			break;

		case STATE_CLICKER_DELAYAFTERCYLE:
			if(pInterCycleTimer->Get() > 2.5)
			{
				clickerLastState = STATE_CLICKER_RAISE;
				lifterLastState = STATE_LIFTER_LOWER;
			}
			break;

		case STATE_CLICKER_RAISE:
			//SmartDashboard::PutString("Cube Clicker State", "RAISE");
			clickerHallEffectTop = clickerMotor->IsFwdLimitSwitchClosed();
			SmartDashboard::PutBoolean("Clicker @ Top", clickerHallEffectTop);

			if(!clickerHallEffectTop)
			{
				// not at the top yet, keep moving

				clickerLastState = STATE_CLICKER_RAISE;
				clickerMotor->Set(fClickerRaise);
			}
			else
			{
				// at the top now, turn the motor off

				clickerLastState = STATE_CLICKER_TOP;
				clickerMotor->Set(0);
			}
			break;
		}	//End of clicker state machine


		//Begin of lifter state machine
		switch(lifterLastState) {
		case STATE_LIFTER_BOTTOM:
			//SmartDashboard::PutString("Cube Lifter State", "BOTTOM");
			lifterMotor->Set(0.0);
			break;

		case STATE_LIFTER_RAISE:
			//SmartDashboard::PutString("Cube Lifter State", "RAISE");
			lifterHallEffectTop = lifterMotor->IsRevLimitSwitchClosed();
			SmartDashboard::PutBoolean("Lifter @ Top", lifterHallEffectTop);

			if(lifterHallEffectTop)
			{
				// we are at the top now

				lifterLastState = STATE_LIFTER_TOP;
				lifterMotor->Set(0.0);
			}
			else
			{
				// keep moving up

				lifterMotor->Set(fLifterRaise);
			}
			break;

		case STATE_LIFTER_WAITTILLRAISE:
			if(bOkToRaiseCan)
			{
				// the driver has the can in place, go on up
				bOkToRaiseCan = false;
				lifterLastState = STATE_LIFTER_RAISE;
			}
			break;

		case STATE_LIFTER_TOP:
			//SmartDashboard::PutString("Cube Lifter State", "TOP");

			// just stay here till the clicker state machine catches up

			lifterMotor->Set(0.0);
			break;

		case STATE_LIFTER_LOWER:
			//SmartDashboard::PutString("Cube Lifter State", "LOWER");
			lifterHallEffectBottom = lifterMotor->IsFwdLimitSwitchClosed();
			SmartDashboard::PutBoolean("Lifter @ Bottom", lifterHallEffectBottom);

			if(lifterHallEffectBottom)
			{
				// we are at the bottom, turn it off

				lifterLastState = STATE_LIFTER_BOTTOM;
				lifterMotor->Set(0.0);
			}
			else
			{
				// we have not arrive yet, keep going down

				lifterMotor->Set(fLifterLower);
			}
			break;
		}	//End of lifter state machine
	}
}

