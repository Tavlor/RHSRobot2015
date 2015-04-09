/** \file
 *  Defines task parameters, hardware assignments and controller button/axis assignment.
 *
 * This header contains basic parameters for the robot. All parameters must be constants with internal
 * linkage, otherwise the One Definition Rule will be violated.
 */

#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

//Robot
#include "JoystickLayouts.h"			//For joystick layouts

//Robot Params
const char* const ROBOT_NAME =		"RhsRobot2015 Oklahoma";	//Formal name
const char* const ROBOT_NICKNAME =   "The Blues Brothers";		//Nickname
const char* const ROBOT_VERSION =	"2.0";						//Version

#define ABLIMIT(a,b) if(a > b) a = b; else if(a < -b) a = -b;

//Task Params - Defines component task priorites relative to the default priority.
//EXAMPLE: const int DRIVETRAIN_PRIORITY = DEFAULT_PRIORITY -2;
const int DEFAULT_PRIORITY = 150;
const int COMPONENT_PRIORITY 	= DEFAULT_PRIORITY;
const int DRIVETRAIN_PRIORITY 	= DEFAULT_PRIORITY;
const int AUTONOMOUS_PRIORITY 	= DEFAULT_PRIORITY;
const int AUTOEXEC_PRIORITY 	= DEFAULT_PRIORITY;
const int AUTOPARSER_PRIORITY 	= DEFAULT_PRIORITY;
const int CONVEYOR_PRIORITY 	= DEFAULT_PRIORITY;
const int CUBE_PRIORITY 		= DEFAULT_PRIORITY;
const int CANLIFTER_PRIORITY 	= DEFAULT_PRIORITY;
const int CLAW_PRIORITY 		= DEFAULT_PRIORITY;
const int CANARM_PRIORITY		= DEFAULT_PRIORITY;

//Task Names - Used when you view the task list but used by the operating system
//EXAMPLE: const char* DRIVETRAIN_TASKNAME = "tDrive";
const char* const COMPONENT_TASKNAME	= "tComponent";
const char* const DRIVETRAIN_TASKNAME	= "tDrive";
const char* const AUTONOMOUS_TASKNAME	= "tAuto";
const char* const AUTOEXEC_TASKNAME		= "tAutoEx";
const char* const AUTOPARSER_TASKNAME	= "tParse";
const char* const CONVEYOR_TASKNAME		= "tConveyor";
const char* const CUBE_TASKNAME			= "tCube";
const char* const CANLIFTER_TASKNAME	= "tCanLift";
const char* const CLAW_TASKNAME			= "tClaw";
const char* const CANARM_TASKNAME		= "tCanArm";

const int COMPONENT_STACKSIZE	= 0x10000;
const int DRIVETRAIN_STACKSIZE	= 0x10000;
const int AUTONOMOUS_STACKSIZE	= 0x10000;
const int AUTOEXEC_STACKSIZE	= 0x10000;
const int AUTOPARSER_STACKSIZE	= 0x10000;
const int CONVEYOR_STACKSIZE	= 0x10000;
const int CUBE_STACKSIZE		= 0x10000;
const int CANLIFTER_STACKSIZE	= 0x10000;
const int CLAW_STACKSIZE		= 0x10000;
const int CANARM_STACKSIZE		= 0x10000;

//TODO change these variables throughout the code to PIPE or whatever instead  of QUEUE
//Queue Names - Used when you want to open the message queue for any task
//NOTE: 2015 - we use pipes instead of queues
//EXAMPLE: const char* DRIVETRAIN_TASKNAME = "tDrive";
const char* const COMPONENT_QUEUE 	= "/tmp/qComp";
const char* const DRIVETRAIN_QUEUE 	= "/tmp/qDrive";
const char* const AUTONOMOUS_QUEUE 	= "/tmp/qAuto";
const char* const AUTOPARSER_QUEUE 	= "/tmp/qParse";
const char* const CONVEYOR_QUEUE	= "/tmp/qConvey";
const char* const CUBE_QUEUE		= "/tmp/qCube";
const char* const CANLIFTER_QUEUE	= "/tmp/qCanLift";
const char* const CLAW_QUEUE		= "/tmp/qClaw";
const char* const CANARM_QUEUE		= "/tmp/qCanArm";

//PWM Channels - Assigns names to PWM ports 1-10 on the Roborio
//EXAMPLE: const int PWM_DRIVETRAIN_FRONT_LEFT_MOTOR = 1;
const int PWM_DRIVETRAIN_LEFT_MOTOR = 1;
const int PWM_DRIVETRAIN_RIGHT_MOTOR = 0;

//CAN IDs - Assigns names to the various CAN IDs
//EXAMPLE: const int CAN_PDB = 0;
/** \page motorID Motor Controller IDs
 * \verbatim
0 - PDB
1 - left drive motor
2 - right drive motor
3 - conveyor
4 - claw
5 - bin lift
6 - can arm
7 - ~~
8 - cube intake roller
9 - cube clicker
 \endverbatim
 */
const int CAN_PDB = 0;
const int CAN_DRIVETRAIN_LEFT_MOTOR = 1;
const int CAN_DRIVETRAIN_RIGHT_MOTOR = 2;
const int CAN_PALLET_JACK_CONVEYOR = 3;
const int CAN_PALLET_JACK_CLAW = 4;
const int CAN_PALLET_JACK_BIN_LIFT = 5;
const int CAN_PALLET_JACK_CAN_ARM = 6;
const int CAN_CUBE_INTAKE = 8;
const int CAN_CUBE_CLICKER = 9;

//Relay Channels - Assigns names to Relay ports 1-8 on the Roborio
//EXAMPLE: const int RLY_COMPRESSOR = 1;

const int CUBECLICKER_MAX_TOTES = 6;

//Digital I/O - Assigns names to Digital I/O ports 1-14 on the Roborio
//EXAMPLE: const int DIO_DRIVETRAIN_BEAM_BREAK = 0;
const int DIO_DRIVETRAIN_BEAM_BREAK = 0;
const int DIO_CANLIFTER_MID_HALL_EFFECT = 1;

//Solenoid - Assigns names to Solenoid ports 1-8 on the 9403
//EXAMPLE: const int SOL_DRIVETRAIN_SOLENOID_SHIFT_IN = 1;

//I2C - Assigns names to I2C ports 1-2 on the Roborio
//EXAMPLE: const int IO2C_AUTO_ACCEL = 1;
const int IO2C_AUTO_ACCEL = 1;

//Analog I/O - Assigns names to Analog I/O ports 1-8 on Anal;og Breakout Module
//EXAMPLE: const int AIO_BATTERY = 8;

//Joystick Input Device Counts - used by the listener to watch buttons and axis
const int JOYSTICK_BUTTON_COUNT = 10;
const int JOYSTICK_AXIS_COUNT = 5;

//POV IDs - Assign names to the 9 POV positions: -1 to 7
//EXAMPLE: const int POV_STILL = -1;
const int POV_STILL = -1;

//Primary Controller Mapping - Assigns action to buttons or axes on the first joystick
#undef	USE_X3D_FOR_CONTROLLER_1
#undef	USE_XBOX_FOR_CONTROLLER_1
#define	USE_L310_FOR_CONTROLLER_1

//Secondary Controller Mapping - Assigns action to buttons or axes on the second joystick
#undef	USE_X3D_FOR_CONTROLLER_2
#undef 	USE_XBOX_FOR_CONTROLLER_2
#define USE_L310_FOR_CONTROLLER_2

#ifdef USE_XBOX_FOR_CONTROLLER_1
#endif
/** \page joysticks Joystick Layouts
 * \verbatim
 	 +++++ Controller 1 +++++
 * 	A Button					Enable CanLifter hover
 * 	B Button					~~
 * 	X Button					Hold Cube clicker at bottom to remove totes
 * 	Y Button					Release Cube clicker from hold
 * 	Start Button				Start Cube autocycle
 * 	Back Button					Stop Cube autocycle
 * 	Left Bumper					Run Conveyor forward
 * 	Right Bumper				Run Conveyor backwards - to claw
 * 	Left Thumbstick Button		Close CanLifter claw
 * 	Right Thumbstick Button		Open CanLifter claw
 * 	Left Thumbstick				Left tank, Arcade
 * 	Right Thumbstick			Right tank
 * 	D-pad						~~
 * 	Left Trigger				Lower CanLifter
 * 	RightTrigger				Raise CanLifter
 *
 	 +++++ Controller 2 +++++
 * 	A Button					~~
 * 	B Button					~~
 * 	X Button					Hold Cube clicker at bottom to remove totes
 * 	Y Button					Release Cube clicker from hold
 * 	Start Button				Start Cube autocycle
 * 	Back Button					Stop Cube autocycle
 * 	Left Bumper					~~
 * 	Right Bumper				~~
 * 	Left Thumbstick Button		~~
 * 	Right Thumbstick Button		~~
 * 	Left Thumbstick				~~
 * 	Right Thumbstick			Raise/lower Cube clicker
 * 	D-pad						~~
 * 	Left Trigger				~~
 * 	RightTrigger				~~
 \endverbatim
 */
#ifdef USE_L310_FOR_CONTROLLER_1
//ID numbers for various buttons and axis
#define TANK_DRIVE_LEFT_ID			L310_THUMBSTICK_LEFT_Y
#define TANK_DRIVE_RIGHT_ID			L310_THUMBSTICK_RIGHT_Y
#define ARCADE_DRIVE_X_ID			L310_THUMBSTICK_LEFT_X
#define ARCADE_DRIVE_Y_ID			L310_THUMBSTICK_LEFT_Y
#define CONVEYOR_FWD_ID				L310_BUTTON_BUMPER_LEFT
#define CONVEYOR_BCK_ID				L310_BUTTON_BUMPER_RIGHT
#define CANLIFTER_RAISE_ID			L310_TRIGGER_LEFT
#define CANLIFTER_LOWER_ID			L310_TRIGGER_RIGHT
#define CANLIFTER_HOVER_ID			L310_BUTTON_A
#define CLAW_CLOSE_ID				L310_BUTTON_THUMB_LEFT
#define CLAW_OPEN_ID				L310_BUTTON_THUMB_RIGHT
#define CUBEAUTO_START_ID			L310_BUTTON_START	//Used on both controllers
#define CUBEAUTO_STOP_ID			L310_BUTTON_STOP	//Used on both controllers
#define CUBEAUTO_HOLD_ID			L310_BUTTON_X		//Used on both controllers
#define CUBEAUTO_RELEASE_ID			L310_BUTTON_Y		//Used on both controllers

//#define TANK_DRIVE_LEFT				pow(-Controller_1->GetRawAxis(L310_THUMBSTICK_LEFT_Y),3)
//#define TANK_DRIVE_RIGHT			pow(-Controller_1->GetRawAxis(L310_THUMBSTICK_RIGHT_Y),3)
#define TANK_DRIVE_LEFT				(-Controller_1->GetRawAxis(L310_THUMBSTICK_LEFT_Y))
#define TANK_DRIVE_RIGHT			(-Controller_1->GetRawAxis(L310_THUMBSTICK_RIGHT_Y))
#define ARCADE_DRIVE_X				Controller_1->GetRawAxis(L310_THUMBSTICK_LEFT_X)
#define ARCADE_DRIVE_Y				-Controller_1->GetRawAxis(L310_THUMBSTICK_LEFT_Y)
#define CONVEYOR_FWD				Controller_1->GetRawButton(L310_BUTTON_BUMPER_LEFT)
#define CONVEYOR_BCK				Controller_1->GetRawButton(L310_BUTTON_BUMPER_RIGHT)
//next 2 are temporary
#define CANARM_OPEN					false//Controller_1->GetRawButton(L310_BUTTON_BUMPER_LEFT)
#define CANARM_CLOSE				false//Controller_1->GetRawButton(L310_BUTTON_BUMPER_RIGHT)
#define CANLIFTER_RAISE				Controller_1->GetRawAxis(L310_TRIGGER_RIGHT)
#define CANLIFTER_LOWER				Controller_1->GetRawAxis(L310_TRIGGER_LEFT)
#define CLAW_CLOSE					Controller_1->GetRawButton(L310_BUTTON_THUMB_LEFT)
#define CLAW_OPEN					Controller_1->GetRawButton(L310_BUTTON_THUMB_RIGHT)
#endif // USE_L310_FOR_CONTROLLER_1

#ifdef USE_X3D_FOR_CONTROLLER_2
#endif // USE_X3D_FOR_CONTROLLER_2

#ifdef USE_XBOX_FOR_CONTROLLER_2
#endif // USE_XBOX_FOR_CONTROLLER_2

#ifdef USE_L310_FOR_CONTROLLER_2
//ID numbers for various buttons and axis
#define CUBEINTAKE_RUN_ID			L310_BUTTON_BUMPER_LEFT
#define CUBECLICKER_RAISE_ID		L310_THUMBSTICK_RIGHT_Y
#define CUBECLICKER_LOWER_ID		L310_THUMBSTICK_RIGHT_Y

#define CUBEINTAKE_RUN				Controller_2->GetRawButton(L310_BUTTON_BUMPER_LEFT)
#define CUBEAUTO_START				Controller_2->GetRawButton(L310_BUTTON_X)
#define CUBEAUTO_STOP				Controller_2->GetRawButton(L310_BUTTON_Y)
#define CUBECLICKER_RAISE			(-Controller_2->GetRawAxis(L310_THUMBSTICK_RIGHT_Y) > 0.65)
#define CUBECLICKER_LOWER			(-Controller_2->GetRawAxis(L310_THUMBSTICK_RIGHT_Y) < -0.65)
#endif // USE_L310_FOR_CONTROLLER_2

#endif //ROBOT_PARAMS_H
