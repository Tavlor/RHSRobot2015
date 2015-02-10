/**  Main robot class.
 *
 * The RhsRobot class is the main robot class. It inherits from RhsRobotBase and MUST define the Init() function, the Run() function, and
 * the OnStateChange() function.  Messages from the DS are processed and commands sent to the subsystems
 * that implement behaviors for each part for the robot.
 */

#include "WPILib.h"

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

//Robot
#include "RhsRobot.h"

RhsRobot::RhsRobot() {
	Controller_1 = NULL;
	Controller_2 = NULL;
	drivetrain = NULL;
	autonomous = NULL;
	conveyor = NULL;
	clicker = NULL;
	jackclicker = NULL;
	canlifter = NULL;

	bCubeIntakeButtonDown = false;
	bCubeIntakeRunning = false;

	iLoop = 0;
}

RhsRobot::~RhsRobot() {
	/*
	 * Free all allocated memory
	 * EXAMPLE: delete drivetrain;
	 */

	delete Controller_1;
	delete Controller_2;
	//delete autonomous;
	delete drivetrain;
	delete conveyor;
	delete clicker;
	delete jackclicker;
	delete canlifter;
}

void RhsRobot::Init()			//Initializes the robot
{
	/* 
	 * Set all pointers to null and then allocate memory and construct objects
	 * EXAMPLE:	drivetrain = NULL;
	 * 			drivetrain = new Drivetrain();
	 */
	Controller_1 = new Joystick(0);
	Controller_2 = new Joystick(1);
	drivetrain = new Drivetrain();
	conveyor = new Conveyor();
	clicker = new Clicker();
	jackclicker = new JackClicker();
	canlifter = new CanLifter();
	//autonomous = new Autonomous();
}

void RhsRobot::OnStateChange()			//Handles state changes
{
	/* 
	 * Alert all components of state changes by sending robotMessage.
	 * EXAMPLE:	if(drivetrain)
	 * 			{
	 * 				drivetrain->SendMessage(&robotMessage);
	 * 			}
	 */

	if (drivetrain) {
		drivetrain->SendMessage(&robotMessage);
	}

	if (conveyor) {
		conveyor->SendMessage(&robotMessage);
	}

	if (clicker) {
		clicker->SendMessage(&robotMessage);
	}

	if (jackclicker) {
		jackclicker->SendMessage(&robotMessage);
	}

	if (canlifter) {
		canlifter->SendMessage(&robotMessage);
	}
}

void RhsRobot::Run() {
	//SmartDashboard::PutString("ROBOT STATUS", "Running");
	/* Poll for control data and send messages to each subsystem. Surround blocks with if(component) so entire components can be disabled
	 * by commenting out their construction.
	 * EXAMPLE: if(drivetrain) 
	 * 			{ 
	 * 				//Check joysticks and send messages 
	 * 			}
	 */

	if (autonomous) {
		if (GetCurrentRobotState() == ROBOT_STATE_AUTONOMOUS) {
			// all messages to components will come from the autonomous task
			return;
		}
	}

	if (drivetrain) {
		if (GetCurrentRobotState() == ROBOT_STATE_AUTONOMOUS) {
			if (HasStateChanged())
				robotMessage.command = COMMAND_DRIVETRAIN_INIT_STRAIGHT;
			else {
				robotMessage.command = COMMAND_DRIVETRAIN_DRIVE_STRAIGHT;
				robotMessage.params.straightDistance = 600;			//in inches
			}
		} else {
			robotMessage.command = COMMAND_DRIVETRAIN_DRIVE_TANK;
			robotMessage.params.tankDrive.left = TANK_DRIVE_LEFT;
			robotMessage.params.tankDrive.right = TANK_DRIVE_RIGHT;
			/*robotMessage.command = COMMAND_DRIVETRAIN_DRIVE_ARCADE;
			 robotMessage.params.arcadeDrive.x = ARCADE_DRIVE_X;
			 robotMessage.params.arcadeDrive.y = ARCADE_DRIVE_Y;*/
		}
		drivetrain->SendMessage(&robotMessage);
	}

	if (conveyor) {
		//button press triggers action; if nothing happens, the ignore command is sent
		if (CONVEYOR_FWD) {
			robotMessage.command = COMMAND_CONVEYOR_RUNALL_FWD;
		} else if (CONVEYOR_BCK) {
			robotMessage.command = COMMAND_CONVEYOR_RUNALL_BCK;
		} else {
			robotMessage.command = COMMAND_CONVEYOR_STOP;
		}

		conveyor->SendMessage(&robotMessage);
	}
	if(clicker)
	{
		//TODO: assign final input controls to the clicker

		if(CLICKER_UP)
		{
			robotMessage.command = COMMAND_CUBECLICKER_RAISE;
		}
		else if(CLICKER_DOWN)
		{
			robotMessage.command = COMMAND_CUBECLICKER_LOWER;
		}
		else
		{
			robotMessage.command = COMMAND_CUBECLICKER_STOP;
		}

		clicker->SendMessage(&robotMessage);

		if(CUBEINTAKE_RUN && (bCubeIntakeButtonDown == false))
		{
			bCubeIntakeButtonDown = true;

			bCubeIntakeRunning = !bCubeIntakeRunning;
			robotMessage.command = COMMAND_CUBEINTAKE_RUN;

		}
		else
		{
			bCubeIntakeButtonDown = false;
		}

		if(bCubeIntakeRunning)
		{
			robotMessage.command = COMMAND_CUBEINTAKE_RUN;
		}
		else
		{
			robotMessage.command = COMMAND_CUBEINTAKE_STOP;
		}

		clicker->SendMessage(&robotMessage);
	}

	if (jackclicker) {
		//TODO: assign input controls to the pallet jack clicker
	}

	if (canlifter) {
		if (CAN_LIFT_RAISE) {
			robotMessage.command = COMMAND_CANLIFTER_RAISE;
		} else if (CAN_LIFT_LOWER) {
			robotMessage.command = COMMAND_CANLIFTER_RAISE;
		} else {
			robotMessage.command = COMMAND_IGNORE;
		}
		canlifter->SendMessage(&robotMessage);
	}

	iLoop++;
}

START_ROBOT_CLASS(RhsRobot)
