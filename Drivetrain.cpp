/** \file
 * Implementation of class to drive the pallet jack.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the pallet jack's wheels.
 *
 * The task receives messages form the main robot class and runs the wheels.
 * Special commands use a gyro and quadrature encoder to drive straight X feet
 * or to turn X degrees.
 *
 * Motor orientations:
 * left +
 * right -
 */

//Local
#include "Drivetrain.h"			//For the local header file

#include <math.h>
#include <assert.h>

#include <string>
#include <iostream>
#include <algorithm>

#include "ComponentBase.h"
#include "RobotParams.h"
using namespace std;

Drivetrain::Drivetrain() :
		ComponentBase(DRIVETRAIN_TASKNAME, DRIVETRAIN_QUEUE,
				DRIVETRAIN_PRIORITY) {

	leftMotor = new CANTalon(CAN_DRIVETRAIN_LEFT_MOTOR);
	rightMotor = new CANTalon(CAN_DRIVETRAIN_RIGHT_MOTOR);

	wpi_assert(leftMotor && rightMotor);

	leftMotor->SetControlMode(CANSpeedController::kPercentVbus);
	rightMotor->SetControlMode(CANSpeedController::kPercentVbus);
	leftMotor->SetVoltageRampRate(120.0);
	rightMotor->SetVoltageRampRate(120.0);

	wpi_assert(leftMotor->IsAlive());
	wpi_assert(rightMotor->IsAlive());

	toteSensor = new DigitalInput(DIO_DRIVETRAIN_BEAM_BREAK);

	pAutoTimer = new Timer();
		pAutoTimer->Start();

	gyro = new ADXRS453Z;
	//encoder = new Encoder(0, 1, false, Encoder::k4X);
	//encoder->SetDistancePerPulse(fEncoderRatio); //diameter*pi/encoder_resolution
	wpi_assert(gyro);
	//wpi_assert(encoder);
	gyro->Start();

	pTask = new Task(DRIVETRAIN_TASKNAME, (FUNCPTR) &Drivetrain::StartTask,
			DRIVETRAIN_PRIORITY, DRIVETRAIN_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int) this);
}

Drivetrain::~Drivetrain()			//Destructor
{
	delete (pTask);
	delete leftMotor;
	delete rightMotor;
	delete gyro;
	//delete encoder;
}

void Drivetrain::OnStateChange()			//Handles state changes
{
	switch(localMessage.command) {
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
		//restore motor values
		leftMotor->Set(left);
		rightMotor->Set(right);
		//gyro->Reset();
		//encoder->Reset();
		//gyro should be reset by a message from autonomous
		break;

	case COMMAND_ROBOT_STATE_TEST:
		leftMotor->Set(0.0);
		rightMotor->Set(0.0);
		break;

	case COMMAND_ROBOT_STATE_TELEOPERATED:
		leftMotor->Set(0.0);
		rightMotor->Set(0.0);
		break;

	case COMMAND_ROBOT_STATE_DISABLED:
		leftMotor->Set(0.0);
		rightMotor->Set(0.0);
		break;

	case COMMAND_ROBOT_STATE_UNKNOWN:
		leftMotor->Set(0.0);
		rightMotor->Set(0.0);
		break;

	default:
		leftMotor->Set(0.0);
		rightMotor->Set(0.0);
		break;
	}
}

///left + , right -
void Drivetrain::Run() {
	switch(localMessage.command) {
	case COMMAND_DRIVETRAIN_DRIVE_TANK:
		//SmartDashboard::PutString("Drivetrain CMD", "DRIVETRAIN_DRIVE_TANK");
		//speed reduction will be controlled by RhsRobot. Power curve is done with raw joystick value
		leftMotor->Set(localMessage.params.tankDrive.left);
		rightMotor->Set(-localMessage.params.tankDrive.right);
		break;
	case COMMAND_DRIVETRAIN_DRIVE_ARCADE:
		//SmartDashboard::PutString("Drivetrain CMD", "DRIVETRAIN_DRIVE_ARCADE");
		ArcadeDrive(localMessage.params.arcadeDrive.x,
				localMessage.params.arcadeDrive.y);
		break;

	case COMMAND_DRIVETRAIN_DRIVE_STRAIGHT:
		//SmartDashboard::PutString("Drivetrain CMD", "DRIVETRAIN_DRIVE_STRAIGHT");
		StraightDrive(localMessage.params.autonomous.driveSpeed);
		break;

	case COMMAND_AUTONOMOUS_RUN:	//when auto starts
		//SmartDashboard::PutString("Drivetrain CMD", "AUTONOMOUS_RUN");
		//reset stored values
		left = 0;
		right = 0;
		pAutoTimer->Reset();
		gyro->Zero();
		break;

	case COMMAND_AUTONOMOUS_COMPLETE:
		//SmartDashboard::PutString("Drivetrain CMD", "AUTONOMOUS_COMPLETE");
		//reset all auto variables
		left = 0;
		right = 0;
		leftMotor->Set(left);
		rightMotor->Set(right);
		gyro->Zero();
	break;

	case COMMAND_DRIVETRAIN_AUTO_MOVE:
		//SmartDashboard::PutString("Drivetrain CMD", "DRIVETRAIN_DRIVE_AUTO_MOVE");
		//store sent
		left = localMessage.params.tankDrive.left;
		right = -localMessage.params.tankDrive.right;
		leftMotor->Set(left);
		rightMotor->Set(right);
		break;

	case COMMAND_DRIVETRAIN_TURN:
		//SmartDashboard::PutString("Drivetrain CMD", "DRIVETRAIN_TURN");
		Turn(localMessage.params.autonomous.turnAngle,localMessage.params.autonomous.timeout);
		break;

	case COMMAND_DRIVETRAIN_SEEK_TOTE:
		SeekTote(localMessage.params.autonomous.timeout);
		break;

		case COMMAND_DRIVETRAIN_STOP:
			//SmartDashboard::PutString("Drivetrain CMD", "DRIVETRAIN_STOP");
			//reset all auto variables
			left = 0;
			right = 0;
			leftMotor->Set(left);
			rightMotor->Set(right);
			gyro->Reset();
		break;

	case COMMAND_SYSTEM_MSGTIMEOUT:
		//SmartDashboard::PutString("Drivetrain CMD", "SYSTEM_MSGTIMEOUT");
	default:
		break;
	}
	/*if(bIsAuto)
	{
		leftMotor->Set(left);
		rightMotor->Set(right);
	}*/
	//Put out information
	SmartDashboard::PutNumber("Gyro Angle", gyro->GetAngle());
}

void Drivetrain::ArcadeDrive(float x, float y) {
	leftMotor->Set(y + x / 2);
	rightMotor->Set(-(y - x / 2));
}
void Drivetrain::MeasuredMove(float speed, float targetDist) {
#if 0
	gyro->Reset();
	encoder->Reset();
	bool isFinished = false;
	while(!isFinished)
	{
		float coveredDist = encoder->GetDistance();
		float remainingDist = targetDist - coveredDist;
		float adjustment = gyro->GetAngle() / recoverStrength;
		//glorified arcade drive
		if(targetDist > 0 && remainingDist > distError)
		{
			//if headed in positive direction
			left = (1 + adjustment) * speed;
			right = (-1 + adjustment) * speed;
		}
		else if(targetDist < 0 && remainingDist < -distError)
		{
			//if headed in negative direction
			left = (-1 + adjustment) * speed;
			right = (1 + adjustment) * speed;
		}
		else
		{
			left = 0;
			right = 0;
			isFinished = true;
		}
		leftMotor->Set(left);
		rightMotor->Set(right);
		SmartDashboard::PutNumber("Covered Distance", coveredDist);
		SmartDashboard::PutNumber("Remaining Distance", remainingDist);
		SmartDashboard::PutNumber("Angle Adjustment", adjustment);
	}
	SmartDashboard::PutString("Covered Distance", "Not operating");
	SmartDashboard::PutString("Remaining Distance", "Not operating");
	SmartDashboard::PutString("Angle Adjustment", "Not operating");
	printf("Finished moving %f inches", targetDist);
#endif
}

void Drivetrain::SeekTote(float timeout)
{
	RobotMessage replyMessage;
		while (pAutoTimer->Get() < timeout && RobotBase::getInstance().IsAutonomous())
		{
			if(toteSensor->Get())
			{
				printf("Tote reached.\n");
				break;
			}
			StraightDrive(fToteSeekSpeed);
		}
		leftMotor->Set(0);
		rightMotor->Set(0);

		replyMessage.command = COMMAND_SYSTEM_OK;
		//Send a message back to auto to tell it that code is done.
		int replyPipe = open(localMessage.replyQ, O_WRONLY);
		wpi_assert(replyPipe > 0);

		write(replyPipe, (char*) &replyMessage, sizeof(RobotMessage));
		close(replyPipe);
}

void Drivetrain::Turn(float targetAngle, float timeout) {
#if 1
	RobotMessage replyMessage;
	//gyro->Reset();
	targetAngle += gyro->GetAngle();
	pAutoTimer->Reset();
	//Resetting the gyro takes up to 15 seconds.
	while (pAutoTimer->Get() < timeout && RobotBase::getInstance().IsAutonomous())
	{
		//if you don't disable this during non-auto, it will keep trying to turn during teleop. Not fun.
		float degreesLeft = targetAngle - gyro->GetAngle();

		if (degreesLeft < angleError && degreesLeft > -angleError)
		{
			break;
		}

		float motorValue = std::min(degreesLeft * .3f, turnSpeedLimit);
		leftMotor->Set(motorValue);
		rightMotor->Set(motorValue);
		SmartDashboard::PutNumber("Remaining Degrees", degreesLeft);
		SmartDashboard::PutNumber("Turn Speed", motorValue);
	}
	leftMotor->Set(0);
	rightMotor->Set(0);
	SmartDashboard::PutNumber("Remaining Degrees", 0.0);
	SmartDashboard::PutNumber("Turn Speed", 0.0);
	printf("Finished turning %f degrees\n", targetAngle);

	replyMessage.command = COMMAND_SYSTEM_OK;

	//Send a message back to auto to tell it that code is done.
	int replyPipe = open(localMessage.replyQ, O_WRONLY);
	wpi_assert(replyPipe > 0);

	write(replyPipe, (char*) &replyMessage, sizeof(RobotMessage));
	close(replyPipe);


#endif
}

void Drivetrain::StraightDrive(float speed) {
	if(speed > 1.0)
	{
		speed = 1;
	}
	else if(speed < -1.0)
	{
		speed = -1;
	}

	float adjustment = std::min(gyro->GetAngle(), 30.0f) * recoverStrength;
	//glorified arcade drive
	if (speed > 0)
	{
		//if headed in positive direction
		left = (1 + adjustment) * speed;
		right = (-1 + adjustment) * speed;
	}
	else if (speed < 0)
	{
		//if headed in negative direction
		left = (-1 + adjustment) * speed;
		right = (1 + adjustment) * speed;
	}
	leftMotor->Set(left);
	rightMotor->Set(right);
	SmartDashboard::PutNumber("Angle Adjustment", adjustment);
}
