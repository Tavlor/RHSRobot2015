/**  Implementation of class to drive the pallet jack.
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

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

//Built-In
#include <math.h>
#include <assert.h>

#include <string>
#include <iostream>
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

	gyro = new ADXRS453Z;
	encoder = new Encoder(0, 1, false, Encoder::k4X);
	encoder->SetDistancePerPulse(0.0061359); //diameter*pi/encoder_resolution
	wpi_assert(gyro);
	wpi_assert(encoder);
	gyro->Start();

	pTask = new Task(DRIVETRAIN_TASKNAME, (FUNCPTR) &Drivetrain::StartTask,
			DRIVETRAIN_PRIORITY, DRIVETRAIN_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int)this);
}

Drivetrain::~Drivetrain()			//Destructor
{
	delete(pTask);
	delete leftMotor;
	delete rightMotor;
}

void Drivetrain::OnStateChange()			//Handles state changes
{
	switch (localMessage.command) {
	case COMMAND_ROBOT_STATE_AUTONOMOUS:
		leftMotor->Set(0.0);
		rightMotor->Set(0.0);
		initialAngle = gyro->GetAngle();
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
	switch (localMessage.command) {
	case COMMAND_DRIVETRAIN_DRIVE_TANK:
		SmartDashboard::PutString("Drivetrain CMD",
				"COMMAND_DRIVETRAIN_DRIVE_TANK");
		leftMotor->Set(pow(localMessage.params.tankDrive.left, 3));
		rightMotor->Set(-pow(localMessage.params.tankDrive.right, 3));
		break;
	case COMMAND_DRIVETRAIN_DRIVE_ARCADE:
		SmartDashboard::PutString("Drivetrain CMD",
				"COMMAND_DRIVETRAIN_DRIVE_ARCADE");
		leftMotor->Set(.5*(localMessage.params.arcadeDrive.y + localMessage.params.arcadeDrive.x));
		rightMotor->Set(-.5*(localMessage.params.arcadeDrive.y - localMessage.params.arcadeDrive.x));
		break;
	/*case COMMAND_DRIVETRAIN_INIT_STRAIGHT:
		SmartDashboard::PutString("Drivetrain CMD",
				"COMMAND_DRIVETRAIN_INIT_STRAIGHT");
		break;*/
	case COMMAND_DRIVETRAIN_DRIVE_STRAIGHT:
		SmartDashboard::PutString("Drivetrain CMD",
				"COMMAND_DRIVETRAIN_DRIVE_STRAIGHT");
		DriveStraight(120);//localMessage.params.straightDistance);
		break;

	case COMMAND_SYSTEM_MSGTIMEOUT:
		SmartDashboard::PutString("Drivetrain CMD",
				"COMMAND_SYSTEM_MSGTIMEOUT");
	default:
		break;
	}
	//Put out information
	SmartDashboard::PutNumber("Covered Distance", coveredDist);
	SmartDashboard::PutNumber("Left Drive Motor Voltage", leftMotor->GetOutputVoltage());
	SmartDashboard::PutNumber("Right Drive Motor Voltage", rightMotor->GetOutputVoltage());

}

void Drivetrain::DriveStraight(float targetDist) {
	coveredDist = encoder->GetDistance();
	SmartDashboard::PutNumber("Remaining Distance", targetDist - coveredDist);
	SmartDashboard::PutNumber("Gyro Angle", gyro->GetAngle());
	SmartDashboard::PutNumber("Angle Error", errorAngle);
	errorAngle = initialAngle - gyro->GetAngle();
	if (coveredDist < targetDist) {
		//remember, you are reverting the error
		left = (1 + errorAngle/45) * .3;
		right = (-1 + errorAngle/45) * .3;
	} else {
		left = 0;
		right = 0;
	}
	leftMotor->Set(left);
	rightMotor->Set(right);
}
float Drivetrain::LimitMotor(float motorValue, float pastValue) //restrict motor acceleration to prevent destruction
		{
	/*use "motor->Get()" to get the previous PWM value
	 note: motorValue is usually based on raw joystick reading,
	 while pastValue is the previous gain-limited reading */
	// if the motorValue is arbitrarily small, make it 0
	if (motorValue < JOYSTICK_DEADZONE && motorValue > -JOYSTICK_DEADZONE)
		motorValue = 0.0;

	/* restrict the acceleration
	 formula: if(change_in_motor_value > +-allowed_change)
	 return previous_value -+ allowed_change; */
	//positive acceleration
	if ((motorValue - pastValue) > MAX_GAIN_PER_MESSAGE)
		return pastValue + MAX_GAIN_PER_MESSAGE;
	//negative acceleration
	if ((motorValue - pastValue) < -MAX_GAIN_PER_MESSAGE)
		return pastValue - MAX_GAIN_PER_MESSAGE;
	return motorValue;
}
