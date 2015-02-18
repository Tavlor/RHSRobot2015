/** \file
 * Main robot class.
 *
 * The RhsRobot class is the main robot class. It inherits from RhsRobotBase and MUST define the Init() function, the Run() function, and
 * the OnStateChange() function.  Messages from the DS are processed and commands sent to the subsystems
 * that implement behaviors for each part for the robot.
 */

#include "RhsRobot.h"
#include "WPILib.h"

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

RhsRobot::RhsRobot() {
	Controller_1 = NULL;
	Controller_2 = NULL;
	ControllerListen_1 = NULL;
	ControllerListen_2 = NULL;
	drivetrain = NULL;
	autonomous = NULL;
	conveyor = NULL;
	cube = NULL;
	jackclicker = NULL;

	bLastConveyorButtonDown = false;

	iLoop = 0;
}

RhsRobot::~RhsRobot() {
	std::vector<ComponentBase *>::iterator nextComponent = ComponentSet.begin();

	for(; nextComponent != ComponentSet.end(); ++nextComponent) {
		delete (*nextComponent);
	}

	delete Controller_1;
	delete Controller_2;
	delete ControllerListen_1;
	delete ControllerListen_2;
}

void RhsRobot::Init() {
	/* 
	 * Set all pointers to null and then allocate memory and construct objects
	 * EXAMPLE:	drivetrain = NULL;
	 * 			drivetrain = new Drivetrain();
	 */
	Controller_1 = new Joystick(0);
	Controller_2 = new Joystick(1);
	ControllerListen_1 = new JoystickListener(Controller_1);
	ControllerListen_2 = new JoystickListener(Controller_2);
	drivetrain = new Drivetrain();
	conveyor = new Conveyor();
	cube = new Cube();
	jackclicker = new JackClicker();
	//autonomous = new Autonomous();

	std::vector<ComponentBase *>::iterator nextComponent = ComponentSet.begin();

	if(drivetrain) {
		nextComponent = ComponentSet.insert(nextComponent, drivetrain);
	}

	if(conveyor) {
		nextComponent = ComponentSet.insert(nextComponent, conveyor);
	}

	if(cube) {
		nextComponent = ComponentSet.insert(nextComponent, cube);
	}

	if(jackclicker) {
		nextComponent = ComponentSet.insert(nextComponent, jackclicker);
	}

	if(autonomous) {
		nextComponent = ComponentSet.insert(nextComponent, autonomous);
	}
}

void RhsRobot::OnStateChange() {
	std::vector<ComponentBase *>::iterator nextComponent;

	for(nextComponent = ComponentSet.begin();
			nextComponent != ComponentSet.end(); ++nextComponent) {
		(*nextComponent)->SendMessage(&robotMessage);
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

	if(autonomous) {
		if(GetCurrentRobotState() == ROBOT_STATE_AUTONOMOUS) {
			robotMessage.command = COMMAND_AUTONOMOUS_RUN;
			autonomous->SendMessage(&robotMessage);
			// all messages to components will come from the autonomous task
			return;
		}
	}

	if(drivetrain) {
		robotMessage.command = COMMAND_DRIVETRAIN_DRIVE_TANK;
		robotMessage.params.tankDrive.left = TANK_DRIVE_LEFT;
		robotMessage.params.tankDrive.right = TANK_DRIVE_RIGHT;
		//robotMessage.command = COMMAND_DRIVETRAIN_DRIVE_ARCADE;
		//robotMessage.params.arcadeDrive.x = ARCADE_DRIVE_X;
		//robotMessage.params.arcadeDrive.y = ARCADE_DRIVE_Y;
		drivetrain->SendMessage(&robotMessage);
	}

	if(conveyor) {
		//button press triggers action; if nothing happens, the ignore command is sent
		if(CONVEYOR_FWD) { //only used for autonomous and depositing cans
			SmartDashboard::PutString("Conveyor Mode", "Output Front");
			robotMessage.command = COMMAND_CONVEYOR_RUNALL_FWD;

			if(!bLastConveyorButtonDown) {
				robotMessage.params.conveyorStates.bButtonWentDownEvent = true;
				bLastConveyorButtonDown = true;
			}
			else {
				robotMessage.params.conveyorStates.bButtonWentDownEvent = false;
			}
		}
		else if(CONVEYOR_BCK) { //used to intake and deposit totesif (CONVEYOR_ADJUST_LEFT > .1) {
			if(CONVEYOR_ADJUST_LEFT > .1 && CONVEYOR_ADJUST_RIGHT > .1) {
				SmartDashboard::PutString("Conveyor Mode", "Adjusting Both");
				robotMessage.command = COMMAND_CONVEYOR_CANADJUST_BOTH;
			}
			else if(CONVEYOR_ADJUST_LEFT > .1) {
				SmartDashboard::PutString("Conveyor Mode", "Adjusting Left");
				robotMessage.command = COMMAND_CONVEYOR_CANADJUST_LEFT;
			}
			else if(CONVEYOR_ADJUST_RIGHT > .1) {
				SmartDashboard::PutString("Conveyor Mode", "Adjusting Right");
				robotMessage.command = COMMAND_CONVEYOR_CANADJUST_RIGHT;
			}
			else {
				SmartDashboard::PutString("Conveyor Mode", "Intake Front");
				robotMessage.command = COMMAND_CONVEYOR_RUNALL_BCK;
			}

			if(!bLastConveyorButtonDown) {
				robotMessage.params.conveyorStates.bButtonWentDownEvent = true;
				bLastConveyorButtonDown = true;
			}
			else {
				robotMessage.params.conveyorStates.bButtonWentDownEvent = false;
			}
		}
		else {
			SmartDashboard::PutString("Conveyor Mode", "Stopped");
			robotMessage.command = COMMAND_CONVEYOR_RUNALL_STOP;
			robotMessage.params.conveyorStates.bButtonWentDownEvent = false;
			bLastConveyorButtonDown = false;
		}

		conveyor->SendMessage(&robotMessage);
	}

	if(cube)
		//TODO add a change-detector for cube commands to reduce number sent
	{
		if(ControllerListen_2->ButtonPressed(CUBEAUTO_START_ID))
		{
			robotMessage.command = COMMAND_CUBEAUTOCYCLE_START;
			cube->SendMessage(&robotMessage);
		}
		else if(ControllerListen_2->ButtonPressed(CUBEAUTO_STOP_ID))
		{
			robotMessage.command = COMMAND_CUBEAUTOCYCLE_STOP;
			cube->SendMessage(&robotMessage);
		}
		else if(ControllerListen_2->ButtonPressed(CUBEAUTO_PAUSE_ID))
		{
			robotMessage.command = COMMAND_CUBEAUTOCYCLE_PAUSE;
			cube->SendMessage(&robotMessage);
		}
		else if(ControllerListen_2->ButtonPressed(CUBEAUTO_RESUME_ID))
		{
			robotMessage.command = COMMAND_CUBEAUTOCYCLE_RESUME;
			cube->SendMessage(&robotMessage);
		}

		if(CUBECLICKER_RAISE)
		{
			robotMessage.command = COMMAND_CUBECLICKER_RAISE;
			cube->SendMessage(&robotMessage);
		}
		else if(CUBECLICKER_LOWER)
		{
			robotMessage.command = COMMAND_CUBECLICKER_LOWER;
			cube->SendMessage(&robotMessage);
		}
		else
		{
			robotMessage.command = COMMAND_CUBECLICKER_STOP;
			cube->SendMessage(&robotMessage);
		}


		if(CANLIFTER_RAISE)
		{
			robotMessage.command = COMMAND_CANLIFTER_RAISE;
			cube->SendMessage(&robotMessage);
		}
		else if(CANLIFTER_LOWER)
		{
			robotMessage.command = COMMAND_CANLIFTER_LOWER;
			cube->SendMessage(&robotMessage);
		}
		else
		{
			robotMessage.command = COMMAND_CANLIFTER_STOP;
			cube->SendMessage(&robotMessage);
		}

		if(CUBEINTAKE_RUN)
		{
			robotMessage.command = COMMAND_CUBEINTAKE_RUN;
			cube->SendMessage(&robotMessage);
		}
		else
		{
			robotMessage.command = COMMAND_CUBEINTAKE_STOP;
					cube->SendMessage(&robotMessage);
		}
	}

	if(jackclicker) {
		//TODO: assign input controls to the pallet jack clicker
	}

	ControllerListen_1->FinalUpdate();
	ControllerListen_2->FinalUpdate();
	iLoop++;
}

START_ROBOT_CLASS(RhsRobot)
