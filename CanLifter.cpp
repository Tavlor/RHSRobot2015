/** \file
 * Implementation of class to control can lifter.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the can lifter.
 *
 * Lifts the can onto the top of a stack of totes. It also has hall effect
 * sensors to stop motion at the top and bottom, as well as trigger other actions.
 * 	Raise: positive (with CIM)
 *	Lower: negative (with CIM)
 *
 */

//Please DO NOT modify the +/- motor value notes above with out FULL TESTING
#include "CanLifter.h"

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

CanLifter::CanLifter() :
		ComponentBase(CANLIFTER_TASKNAME, CANLIFTER_QUEUE, CANLIFTER_PRIORITY) {

	lifterMotor = new CANTalon(CAN_PALLET_JACK_BIN_LIFT);
	wpi_assert(lifterMotor);
	//lifterMotor->SetVoltageRampRate(120.0);
	lifterMotor->ConfigNeutralMode(
			CANSpeedController::NeutralMode::kNeutralMode_Brake);
	// MrB we moved the sensors so this doesn't do anything
	//lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
	lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);

	wpi_assert(lifterMotor->IsAlive());

	hoverHallEffect = new DigitalInput(DIO_CANLIFTER_HOVER_HALL_EFFECT);

	hoverHallDetect = new Counter(hoverHallEffect);
	hoverHallDetect->Reset();
	/*upperHall = new DigitalInput(DIO_CANLIFTER_UPPER_HALL_EFFECT);
	lowerHall = new DigitalInput(DIO_CANLIFTER_LOWER_HALL_EFFECT);

	upperDetect = new Counter(upperHall);
	upperDetect->Reset();
	lowerDetect = new Counter(lowerHall);
	lowerDetect->Reset();*/

	pSafetyTimer = new Timer();
	pSafetyTimer->Start();
	//pAutoTimer = new Timer();IN COMPONENT BASE
	//pAutoTimer->Start();

	pTask = new Task(CANLIFTER_TASKNAME, (FUNCPTR) &CanLifter::StartTask,
			CANLIFTER_PRIORITY, CANLIFTER_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int) this);
}

CanLifter::~CanLifter() {
	delete (pTask);
	delete lifterMotor;
	delete pSafetyTimer;
}
;

void CanLifter::OnStateChange() {
	switch (localMessage.command)
	{
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
		lifterMotor->Set(fLifterStop);
		bHoverEnabled = false;
		bHoverWhileStopped = false;
		pSafetyTimer->Reset();
		//lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
		break;
	case COMMAND_ROBOT_STATE_TEST:
		lifterMotor->Set(fLifterStop);
		bHoverEnabled = false;
		bHoverWhileStopped = false;
		pSafetyTimer->Reset();
		//lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
		break;
	case COMMAND_ROBOT_STATE_TELEOPERATED:
		bHoverEnabled = false;
		bHoverWhileStopped = false;
		lifterMotor->Set(fLifterStop);
		pSafetyTimer->Reset();
		//lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
		break;
	case COMMAND_ROBOT_STATE_DISABLED:
		lifterMotor->Set(fLifterStop);
		bHoverEnabled = false;
		bHoverWhileStopped = false;
		pSafetyTimer->Reset();
		break;
	case COMMAND_ROBOT_STATE_UNKNOWN:
		lifterMotor->Set(fLifterStop);
		bHoverEnabled = false;
		bHoverWhileStopped = false;
		pSafetyTimer->Reset();
		break;
	default:
		lifterMotor->Set(fLifterStop);
		bHoverEnabled = false;
		bHoverWhileStopped = false;
		pSafetyTimer->Reset();
		break;
	}
}

void CanLifter::Run() {
	switch (localMessage.command)
	{
	case COMMAND_CANLIFTER_RAISE:
		if (!hoverHallEffect->Get() || hoverHallDetect->Get() > 0)
		{
			//if the lifter trips the halleffect or it was recorded, see if bHoverEnabled is false
			hoverHallDetect->Reset();
			if (!bHoverEnabled)
			{
				//if so, drive up
				LifterCurrentLimitDrive(
						fLifterUpMult
								* localMessage.params.canLifterParams.lifterSpeed);
			}
			else
			{
				//otherwise, set to hover power and enable stop-hovering
				lifterMotor->Set(fLifterHover);
				bHoverWhileStopped = true;
			}
		}

		//if not tripped, drive like normal, but enable hovering for next time the hall is tripped
		else
		{
			LifterCurrentLimitDrive(
					fLifterUpMult * localMessage.params.canLifterParams.lifterSpeed);
			bHoverEnabled = true;
			bHoverWhileStopped = false;
		}
		bGoingUp = true;
		bGoingDown = false;
		pSafetyTimer->Reset();
		break;

		case COMMAND_CANLIFTER_LOWER:
			LifterCurrentLimitDrive(fLifterDownMult * localMessage.params.canLifterParams.lifterSpeed);
			//bHoverEnabled = false;
			bHoverWhileStopped = false;
			pSafetyTimer->Reset();
			break;

#if 0
		case COMMAND_CANLIFTER_HOVER:
			lifterMotor->Set(fLifterHover);
			bHovering = true;
			pSafetyTimer->Reset();
			break;

		case COMMAND_CANLIFTER_RAISE_TOTES:
			//to load pos
			//lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
			bHoverEnabled = true;
			bLowerHover = false;
			pSafetyTimer->Reset();
			upperDetect->Reset();
			iToteLoad = localMessage.params.canLifterParams.iNumTotes;

			// start out slow then accelerate after we have the handle

			while(ISAUTO && (pSafetyTimer->Get() < 0.25))
			{
				lifterMotor->Set(fLifterStartRaise);
				Wait(0.02);
			}

			while(ISAUTO && (upperDetect->Get() == 0) && (pSafetyTimer->Get() < 2.00))
			{
				lifterMotor->Set(fLifterRaise);
				Wait(0.02);
			}

			upperDetect->Reset();
			//hovering, so don't stop it
			lifterMotor->Set(fLifterHover);
			// MrB - we need to do other things while lifting
			printf("%s Totes Raised\n", __FILE__);
			SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
			break;

		case COMMAND_CANLIFTER_LOWER_TOTES:
			//lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
			bHoverEnabled = false;
			bLowerHover = true;
			pSafetyTimer->Reset();
			lowerDetect->Reset();

			while(ISAUTO && (lowerDetect->Get() == 0))
			{
				lifterMotor->Set(fLifterLower);
				Wait(0.02);
			}

			lowerDetect->Reset();
			lifterMotor->Set(fLifterHover);
			printf("%s Totes Lowered\n", __FILE__);
			SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
			break;

		case COMMAND_CANLIFTER_START_RAISE_TOTES:
			//lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);
			bHoverEnabled = true;
			bLowerHover = false;
			pSafetyTimer->Reset();
			iToteLoad = localMessage.params.canLifterParams.iNumTotes;
			lifterMotor->Set(fLifterStartRaise);
			break;

		case COMMAND_CANLIFTER_CLAW_TO_TOP:
			//lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
			bHoverEnabled = false;
			bLowerHover = false;

			while(ISAUTO)
			{
				if (lifterMotor->GetOutputCurrent() > fLifterMotorCurrentMaxOneCan)
				{
					lifterMotor->Set(fLifterHover);
					break;
				}

				SmartDashboard::PutNumber("Lift Current", lifterMotor->GetOutputCurrent());
				lifterMotor->Set(fLifterRaise);
				Wait(0.02);
			}

			SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
			pSafetyTimer->Reset();
			break;

		case COMMAND_CANLIFTER_CLAW_TO_BOTTOM:
			bHoverEnabled = false;
			bLowerHover = false;
			//	lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);

			while(ISAUTO && LifterCurrentLimitDrive(fLifterLower))
			{
				SmartDashboard::PutNumber("Lift Current", lifterMotor->GetOutputCurrent());
				Wait(0.02);
			}

			SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
			pSafetyTimer->Reset();
			break;

		/*case COMMAND_CANLIFTER_RAISE_CAN:
			//to hook pos
			bHover = false;
			lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SrxDisableSwitchInputs);
			pSafetyTimer->Reset();
			break;*/
		case COMMAND_CANLIFTER_RAISE_LOMID:
			bLowerHover = true;
			bHoverEnabled = false;
			//lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

			while(ISAUTO && (lowerDetect->Get()== 0))
			{
				lifterMotor->Set(fLifterRaiseLoMid);
				Wait(0.02);
			}

			lowerDetect->Reset();

			//go above the halleffect
			/*while(ISAUTO && BOTTOMHALLEFFECT)
			{
				lifterMotor->Set(fLifterRaise);
			}*/
			printf("%s Lomid Reached\n", __FILE__);
			lifterMotor->Set(fLifterHover);
			pSafetyTimer->Reset();
			break;

		case COMMAND_CANLIFTER_LOWER_HIMID:
			bHoverEnabled = true;
			bLowerHover = false;
			//lifterMotor->ConfigLimitMode(CANSpeedController::kLimitMode_SwitchInputsOnly);

			while(ISAUTO && (upperDetect->Get() == 0))
			{
				lifterMotor->Set(fLifterLower);
				Wait(0.02);
			}

			upperDetect->Reset();
			printf("%s Himid Reached\n", __FILE__);
			pSafetyTimer->Reset();
			break;
#endif

		case COMMAND_CANLIFTER_STOP:
			if(bHoverWhileStopped)
			{
				lifterMotor->Set(fLifterHover);
				bHoverEnabled = false;//disable hovering automaticly
			}
			else
			{
			lifterMotor->Set(fLifterStop);
			}
			bGoingUp = false;
			bGoingDown = false;
			pSafetyTimer->Reset();
			break;

		default:
			break;

	}	//end of command switch

	//if the connection times out, shut everything off
	if (pSafetyTimer->Get() > 30.0)
	{
		lifterMotor->Set(fLifterStop);
		pSafetyTimer->Reset();
	}

	//check that the lifter is not hurting itself
	if (lifterMotor->GetOutputCurrent() > fLifterMotorCurrentMax)
	{
		lifterMotor->Set(fLifterStop);
	}

	// if the middle sensor is tripped were we moving up or down or should we hover holding a tote?

	/*if(midDetect->Get())
	{
		midDetect->Reset();*/

	/*if(!bHoverEnabled)
	{
		bHovering = false;
	}
	if(bHoverEnabled && TOPHALLEFFECT)
	{
		bHovering = true;
	}*/

	/*if(bHovering)
	{
		if(ISAUTO)//if it's auto, we adjust for tote count
		{
			// hover here with enough juice to hold steady
			switch(iToteLoad)
			{
				case 1:
				lifterMotor->Set(fLifterHoverOneTotes);
				break;
				case 2:
				lifterMotor->Set(fLifterHoverTwoTotes);
				break;
				case 3:
				lifterMotor->Set(fLifterHoverThreeTotes);
				break;
				default:
				lifterMotor->Set(fLifterHover);
				break;
			}
		}
		else
		{
			lifterMotor->Set(fLifterHover);
		}
		//SendCommandResponse(COMMAND_AUTONOMOUS_RESPONSE_OK);
	}
	else if(ISAUTO && bLowerHover)
	{
		lifterMotor->Set(fLifterHoverNoTotes);
	}*/
	else
	{
		// not hovering, just normal can up and down motions

		if(lifterMotor->Get() < 0.0)
		{
			// was on the way up

			bGoingUp = true;
			bGoingDown = false;
		}
		else if(lifterMotor->Get() > 0.0)
		{
			// was on the way down

			bGoingUp = false;
			bGoingDown = true;
		}
	}

	// update the Smart Dashboard periodically to reduce traffic
	//if (pRemoteUpdateTimer->Get() > 0.2)
	{
		pRemoteUpdateTimer->Reset();

		SmartDashboard::PutNumber("Lift Current", lifterMotor->GetOutputCurrent());
		//SmartDashboard::PutBoolean("Lifter @ Top", !upperHall->Get());
		//SmartDashboard::PutBoolean("Lifter @ Bottom", !lowerHall->Get());
		SmartDashboard::PutBoolean("Lifter @ Hall Effect", !hoverHallEffect->Get());
		SmartDashboard::PutBoolean("Lifter Hover Enabled", bHoverEnabled);
		SmartDashboard::PutBoolean("Lifter Hover while Stopped", bHoverWhileStopped);
		SmartDashboard::PutBoolean("Lifter Raising", bGoingUp);
		SmartDashboard::PutBoolean("Lifter Lowering", bGoingDown);
	}
}

bool CanLifter::LifterCurrentLimitDrive(float speed) {
	if (lifterMotor->GetOutputCurrent() > fLifterMotorCurrentMax)
	{
		lifterMotor->Set(fLifterStop);
		return false;
	}
	lifterMotor->Set(speed);
	return true;
}
