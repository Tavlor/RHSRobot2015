/** \file
 * Implementation of class to drive the pallet jack.
 *
 * This class is derived from the standard Component base class and includes
 * initialization for the devices used to control the pallet jack's wheels.
 *
 * The task receives messages form the main robot class and runs the wheels.
 * Special commands use a gyro and quadrature encoder to drive straight X feet
 * or to turn X degrees.
 */

//Local
#include "Drivetrain.h"			//For the local header file

#include <math.h>
#include <assert.h>

#include <string>
#include <iostream>

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

	gyro = new ADXRS453Z;
	encoder = new Encoder(0, 1, false, Encoder::k4X);
	encoder->SetDistancePerPulse(0.0061359); //diameter*pi/encoder_resolution
	wpi_assert(gyro);
	wpi_assert(encoder);
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
	delete encoder;
}

void Drivetrain::OnStateChange()			//Handles state changes
{
	switch(localMessage.command) {
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
		leftMotor->Set(0.0);
		rightMotor->Set(0.0);
		gyro->Reset();
		encoder->Reset();
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

void Drivetrain::Run() {
	switch(localMessage.command) {
	case COMMAND_DRIVETRAIN_DRIVE_TANK:
		SmartDashboard::PutString("Drivetrain CMD",
				"COMMAND_DRIVETRAIN_DRIVE_TANK");
		leftMotor->Set(localMessage.params.tankDrive.left / 2.0);//(pow(localMessage.params.tankDrive.left, 3));
		rightMotor->Set(-localMessage.params.tankDrive.right / 2.0);//(-pow(localMessage.params.tankDrive.right, 3));
		break;
	case COMMAND_DRIVETRAIN_DRIVE_ARCADE:
		SmartDashboard::PutString("Drivetrain CMD",
				"COMMAND_DRIVETRAIN_DRIVE_ARCADE");
		ArcadeDrive(localMessage.params.arcadeDrive.x,
				localMessage.params.arcadeDrive.y);
		break;
	case COMMAND_DRIVETRAIN_DRIVE_STRAIGHT:
		SmartDashboard::PutString("Drivetrain CMD",
				"COMMAND_DRIVETRAIN_DRIVE_STRAIGHT");
		MeasuredMove(localMessage.params.autonomous.driveSpeed,
				localMessage.params.autonomous.driveDistance);
		break;

	case COMMAND_DRIVETRAIN_TURN:
		SmartDashboard::PutString("Drivetrain CMD", "COMMAND_DRIVETRAIN_TURN");
		Turn(localMessage.params.autonomous.turnSpeed,
				localMessage.params.autonomous.turnAngle);
		break;

	case COMMAND_SYSTEM_MSGTIMEOUT:
		SmartDashboard::PutString("Drivetrain CMD",
				"COMMAND_SYSTEM_MSGTIMEOUT");
	default:
		break;
	}
	//Put out information
	SmartDashboard::PutNumber("Gyro Angle", gyro->GetAngle());
	lastCommand = localMessage.command;
}

void Drivetrain::ArcadeDrive(float x, float y) {
	leftMotor->Set(y + x / 2);
	rightMotor->Set(-(y - x / 2));
}
void Drivetrain::MeasuredMove(float speed, float targetDist) {
	gyro->Reset();
	encoder->Reset();
	bool isFinished = false;
	while(!isFinished) {
		float coveredDist = encoder->GetDistance();
		float remainingDist = targetDist - coveredDist;
		float adjustment = gyro->GetAngle() / recoverStrength;
		//glorified arcade drive
		if(targetDist > 0 && remainingDist > distError) {
			//if headed in positive direction
			left = (1 + adjustment) * speed;
			right = (-1 + adjustment) * speed;
		}
		else if(targetDist < 0 && remainingDist < -distError) {
			//if headed in negative direction
			left = (-1 + adjustment) * speed;
			right = (1 + adjustment) * speed;
		}
		else {
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
}

void Drivetrain::Turn(float speed, float targetAngle) {
	gyro->Reset();
	bool isFinished = false;
	while(!isFinished)
	{
	float degreesLeft = targetAngle - gyro->GetAngle();
	float motorValue = degreesLeft*.3*speed;
	leftMotor->Set(motorValue);
	rightMotor->Set(motorValue);
	SmartDashboard::PutNumber("Remaining Degrees", degreesLeft);
	SmartDashboard::PutNumber("Turn Speed", motorValue);
	}
	SmartDashboard::PutString("Remaining Degrees", "Not operating");
	SmartDashboard::PutString("Turn Speed", "Not operating");
	printf("Finished turning %f degrees", targetAngle);
}
