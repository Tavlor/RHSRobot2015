/** \file
 * Implementation of class to control functions on the cube.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the cube's components.
 *
 * The task receives messages from the main robot class and controls two components:
 *
 * THE CLICKER
 * Raises or lowers the tote lifter which "clicks" into place, thus the name.
 * Hall effect sensors are used to stop the motion of the "clicker" at the desired points.
 *	Raise:positive (with CIM)
 *	Lower:negative (with CIM)
 *
 * THE INTAKE
 * A roller which pulls totes into the cube from the chute. It is triggered
 * by an IR beam break sensor.
 *	Run: negative (with BAG)
 */

//Please DO NOT modify the +/- motor value notes above with out FULL TESTING
#include "Cube.h"

Cube::Cube() :
		ComponentBase(CUBE_TASKNAME, CUBE_QUEUE, CUBE_PRIORITY) {

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

	wpi_assert(clickerMotor->IsAlive());
	wpi_assert(intakeMotor->IsAlive());

	clickerLastState = STATE_CLICKER_TOP;
	bEnableAutoCycle = false;
	bPrepareToRemove = false;
	fClickerPaused = 0.0;
	bFirstTote = true;

	irBlocked = !intakeMotor->IsRevLimitSwitchClosed();
	clickerHallEffectBottom = clickerMotor->IsRevLimitSwitchClosed();
	clickerHallEffectTop = clickerMotor->IsFwdLimitSwitchClosed();
	topBlocked = (clickerMotor->GetPinStateQuadB() == 1);

	pSafetyTimer = new Timer();
	pSafetyTimer->Start();
	//pAutoTimer = new Timer();IN COMPONENT BASE
	//pAutoTimer->Start();
	pInterCycleTimer = new Timer();
	pInterCycleTimer->Start();
	pGateTimer = new Timer();
	pGateTimer->Start();

	pTask = new Task(CUBE_TASKNAME, (FUNCPTR) &Cube::StartTask, CUBE_PRIORITY,
			CUBE_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int) this);
}

Cube::~Cube() {
	delete (pTask);
	delete clickerMotor;
	delete intakeMotor;
	delete pSafetyTimer;
}
;

void Cube::OnStateChange() {
	//be sure that bEnableAutoCycle will stick - is is possible to drop into DISABLED during auto
	switch(localMessage.command) {
	case COMMAND_ROBOT_STATE_TELEOPERATED:
		bEnableAutoCycle = true;
		intakeMotor->Set(fIntakeRun);		//should always run in teleop
		fClickerPaused = 0.0;
		clickerLastState = STATE_CLICKER_RAISE;
		intakeMotor->Set(fIntakeRun);
		pSafetyTimer->Reset();
		break;
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
		intakeMotor->Set(0);
		pSafetyTimer->Reset();
		break;
	case COMMAND_ROBOT_STATE_DISABLED:
		pSafetyTimer->Reset();
		break;
	case COMMAND_ROBOT_STATE_TEST:
	case COMMAND_ROBOT_STATE_UNKNOWN:
	default:
		pSafetyTimer->Reset();
		break;
	}
}

/** \class Cube
 * Cube behavior: Intake will always run unless the IR sensor is covered.
 * The clicker can either be controlled by the second remote or will
 * cycle through its state machine automatically.
 * Autocycle should be started in the autonomous code.
 */
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

	case COMMAND_CUBEINTAKE_RUN:
		if(!bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Cube CMD", "CUBEINTAKE_RUN");

			intakeMotor->Set(fIntakeRun);
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBEINTAKE_STOP:
		if(!bEnableAutoCycle)
		{
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
			fClickerPaused = 0.0;
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBEAUTOCYCLE_START:
		if(!bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Cube CMD", "CUBEAUTOCYCLE_START");
			bEnableAutoCycle = true;
			fClickerPaused = 0.0;
			clickerLastState = STATE_CLICKER_RAISE;
			intakeMotor->Set(fIntakeRun);
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBEAUTOCYCLE_STOP:
		if(bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Cube CMD", "CUBEAUTOCYCLE_STOP");
			bEnableAutoCycle = false;
			//intakeMotor->Set(fIntakeStop);
			clickerMotor->Set(fClickerStop);
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBEAUTOCYCLE_PAUSE:
		if(bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Cube CMD", "CUBEAUTOCYCLE_PAUSE");
			bEnableAutoCycle = false;
			fClickerPaused = clickerMotor->Get();
			//intakeMotor->Set(fIntakeStop);
			clickerMotor->Set(fClickerStop);
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBEAUTOCYCLE_RESUME:
		if(bEnableAutoCycle)
		{
			//SmartDashboard::PutString("Cube CMD", "CUBEAUTOCYCLE_RESUME");
			clickerMotor->Set(fClickerPaused);
			intakeMotor->Set(fIntakeRun);
			bEnableAutoCycle = true;
			pSafetyTimer->Reset();
		}
		break;

	case COMMAND_CUBEAUTOCYCLE_HOLD:
		if(bEnableAutoCycle)
		{
			bPrepareToRemove = true;
		}
		break;

	case COMMAND_CUBEAUTOCYCLE_RELEASE:
		bPrepareToRemove = false;
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
		pSafetyTimer->Reset();
	}

	// update the remote indicators periodically so we do not create too much CAN traffic
	// or Smart Dashboard traffic either, for that matter (This is really what kills)
	// NOTE: this segment runs only if we are NOT autocycling
	if((pRemoteUpdateTimer->Get() > 0.15))//&& !bEnableAutoCycle)
	{
		pRemoteUpdateTimer->Reset();
		irBlocked = !intakeMotor->IsRevLimitSwitchClosed();
		clickerHallEffectBottom = clickerMotor->IsRevLimitSwitchClosed();
		clickerHallEffectTop = clickerMotor->IsFwdLimitSwitchClosed();
		topBlocked = (clickerMotor->GetPinStateQuadB() == 1);

		SmartDashboard::PutBoolean("Last Tote IR", topBlocked);
		SmartDashboard::PutBoolean("Cube IR", irBlocked);
		SmartDashboard::PutBoolean("Clicker @ Top", clickerHallEffectTop);
		SmartDashboard::PutBoolean("Clicker @ Bottom", clickerHallEffectBottom);
		SmartDashboard::PutNumber("Clicker Voltage", clickerMotor->GetBusVoltage());
		SmartDashboard::PutBoolean("Cube Autocycle", bEnableAutoCycle);
	}

	// run the state machine at a regular rate so we do not create too much CAN traffic

	if(bEnableAutoCycle && (pAutoTimer->Get() > 0.02))
	{
		pAutoTimer->Reset();
		pSafetyTimer->Reset();
		SmartDashboard::PutBoolean("Cube Autocycle", bEnableAutoCycle);
		SmartDashboard::PutNumber("Clicker Voltage", clickerMotor->GetBusVoltage());

		switch(clickerLastState) {

		case STATE_CLICKER_TOP:
			//SmartDashboard::PutString("Cube Clicker State", "TOP");
			irBlocked = !intakeMotor->IsRevLimitSwitchClosed();
			SmartDashboard::PutBoolean("Cube IR", irBlocked);

			topBlocked = (clickerMotor->GetPinStateQuadB() == 1);
			SmartDashboard::PutBoolean("Last Tote IR", topBlocked);

			if(topBlocked && irBlocked){
				bPrepareToRemove = true;
			}

			if(irBlocked || bPrepareToRemove)
			{
				// if the beam is broken (ie tote detected), lower the clicker

				if(bFirstTote)
				{
				  clickerMotor->Set(fClickerTopHold);
				  clickerLastState = STATE_CLICKER_FIRSTTOTEDELAY;
				}
				else
				{
					pGateTimer->Reset();
					clickerLastState = STATE_CLICKER_GATEDELAY;
					clickerMotor->Set(fClickerLower);
				}
			}
			else
			{
				// the beam is open, keep the clicker up (must apply power)
				clickerLastState = STATE_CLICKER_TOP;
				clickerMotor->Set(fClickerTopHold);
			}
			break;

		case STATE_CLICKER_GATEDELAY:
			if(pGateTimer->Get() > 0.25)
			{
				clickerLastState = STATE_CLICKER_LOWER;
			}
			break;

		case STATE_CLICKER_FIRSTTOTEDELAY:
			irBlocked = !intakeMotor->IsRevLimitSwitchClosed();

			if(!irBlocked)
			{
				bFirstTote = false;
				clickerLastState = STATE_CLICKER_TOP;
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

			if(bPrepareToRemove)
			{
				clickerLastState = STATE_CLICKER_BOTTOMHOLD;
			}
			else
			{
				clickerLastState = STATE_CLICKER_RAISE;
				clickerMotor->Set(fClickerRaise);
			}
			break;

		case STATE_CLICKER_BOTTOMHOLD:
			//SmartDashboard::PutString("Cube Clicker State", "BOTTOMHOLD");
			irBlocked = !intakeMotor->IsRevLimitSwitchClosed();
			SmartDashboard::PutBoolean("Cube IR", irBlocked);

			if(!irBlocked)
			{
				// if all totes removed, start again

				bFirstTote = true;
				bPrepareToRemove = false;
				clickerLastState = STATE_CLICKER_DELAYAFTERCYLE;
				//Do not raise the clicker right away, risk of clipping stack
				pInterCycleTimer->Reset();
			}
			else
			{
				// leave clicker down

				clickerMotor->Set(fClickerLower);
			}
			break;

		case STATE_CLICKER_DELAYAFTERCYLE:
			if(pInterCycleTimer->Get() > fDelayAfterCycle)
			{
				clickerLastState = STATE_CLICKER_RAISE;
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
	}
}

