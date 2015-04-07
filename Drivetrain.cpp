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
	wpi_assert(gyro);
	gyro->Start();

	//encoder = new Encoder(0, 1, false, Encoder::k4X);
	//encoder->SetDistancePerPulse(fEncoderRatio); //diameter*pi/encoder_resolution
	//wpi_assert(encoder);

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
		//gyro->Zero();
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
		StraightDrive(localMessage.params.autonomous.driveSpeed, localMessage.params.autonomous.driveTime);
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
		SeekTote(localMessage.params.autonomous.timein,localMessage.params.autonomous.timeout);
		break;

		case COMMAND_DRIVETRAIN_STOP:
			//SmartDashboard::PutString("Drivetrain CMD", "DRIVETRAIN_STOP");
			//reset all auto variables
			left = 0;
			right = 0;
			leftMotor->Set(left);
			rightMotor->Set(right);
			gyro->Zero();
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
	if (pRemoteUpdateTimer->Get() > 0.2)
	{
		pRemoteUpdateTimer->Reset();
		SmartDashboard::PutBoolean("Tote Detector", toteSensor->Get());
		SmartDashboard::PutNumber("Gyro Angle", gyro->GetAngle());
	}
}

void Drivetrain::ArcadeDrive(float x, float y) {
	leftMotor->Set(y + x / 2);
	rightMotor->Set(-(y - x / 2));
}
void Drivetrain::MeasuredMove(float speed, float targetDist) {
#if 0
	gyro->Zero();
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

void Drivetrain::Turn(float targetAngle, float timeout) {
	//TODO: if needed, you can check the rate at which the gyro angle changes
	//to figure out if the robot is moving slow enough to exit.
	MessageCommand command = COMMAND_AUTONOMOUS_RESPONSE_ERROR;
	targetAngle += gyro->GetAngle();
	pAutoTimer->Reset();

	while (pAutoTimer->Get() < timeout
			&& RobotBase::getInstance().IsAutonomous())
	{
		//if you don't disable this during non-auto, it will keep trying to turn during teleop. Not fun.
		float degreesLeft = targetAngle - gyro->GetAngle();

		printf("Turning target %f current %f\n", targetAngle, gyro->GetAngle());

		if ((degreesLeft < angleError) && (degreesLeft > -angleError))
		{
			break;
		}

		float motorValue = degreesLeft * turnAngleSpeedMultiplyer;

		ABLIMIT(motorValue, turnSpeedLimit);

		leftMotor->Set(motorValue);
		rightMotor->Set(motorValue);

		SmartDashboard::PutNumber("Remaining Degrees", degreesLeft);
		SmartDashboard::PutNumber("Turn Speed", motorValue);
	}

	leftMotor->Set(0);
	rightMotor->Set(0);
	command = COMMAND_AUTONOMOUS_RESPONSE_OK;
	SendCommandResponse(command);

	SmartDashboard::PutNumber("Remaining Degrees", 0.0);
	SmartDashboard::PutNumber("Turn Speed", 0.0);
	printf("Finished turning %f degrees\n", targetAngle);
}

void Drivetrain::SeekTote(float timein, float timeout) {
	MessageCommand command = COMMAND_AUTONOMOUS_RESPONSE_ERROR;
	pAutoTimer->Reset();

	while ((pAutoTimer->Get() < timeout)
			&& RobotBase::getInstance().IsAutonomous())
	{
		if (toteSensor->Get() && pAutoTimer->Get() > timein)
		{
			printf("Tote reached.\n");
			command = COMMAND_AUTONOMOUS_RESPONSE_OK;
			break;
		}
		StraightDriveLoop(fToteSeekSpeed);
		Wait(0.01);
	}

	left = 0;
	right = 0;
	leftMotor->Set(0.0);
	rightMotor->Set(0.0);

	SendCommandResponse(command);
}

void Drivetrain::StraightDrive(float speed, float time) {
	MessageCommand command = COMMAND_AUTONOMOUS_RESPONSE_OK;
	pAutoTimer->Reset();
	//DO NOT RESET THE GYRO EVER. only zeroing.
	gyro->Zero();

	while ((pAutoTimer->Get() < time)
			&& RobotBase::getInstance().IsAutonomous())
	{
		StraightDriveLoop(speed);
		Wait(0.01);
	}

	left = 0;
	right = 0;
	leftMotor->Set(0.0);
	rightMotor->Set(0.0);

	SendCommandResponse(command);
}


void Drivetrain::StraightDriveLoop(float speed) {

	//keep the reference angle between -30 and 30 degrees
	//float angle = std::max(std::min(gyro->GetAngle(),fMaxRecoverAngle),-fMaxRecoverAngle);
	float adjustment = gyro->GetAngle() * recoverStrength;
	//glorified arcade drive
	if (speed > 0.0)
	{
		//if headed in positive direction
		left = (1.0 + adjustment) * speed;
		right = (-1.0 + adjustment) * speed;
	}
	else if (speed < 0.0)
	{
		//if headed in negative direction
		left = (1.0 + adjustment) * speed;
		right = (-1.0 + adjustment) * speed;
	}
	else
	{
		left = 0.0;
		right = 0.0;
	}

	//printf("left %0.03f right %0.03f adjust %0.03f\n", left, right, adjustment);

	ABLIMIT(left, 1.0);
	ABLIMIT(right, 1.0);

	leftMotor->Set(left);
	rightMotor->Set(right);

	if (pRemoteUpdateTimer->Get() > 0.2)
	{
		pRemoteUpdateTimer->Reset();
		SmartDashboard::PutBoolean("Tote Detector", toteSensor->Get());
		SmartDashboard::PutNumber("Angle Adjustment", adjustment);
		SmartDashboard::PutNumber("Gyro Angle", gyro->GetAngle());
	}
}

bool Drivetrain::GetToteSensor()
{
	return toteSensor->Get();
}
