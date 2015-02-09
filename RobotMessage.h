/**  Defines the messages we pass from task to task.
 *
 * The RobotMessage struct is a data structure used to pass information to the
 * robot's components. It is composed of a command that indicates the action to
 * be carried out and a union of params that contain additional data.
 */

#ifndef ROBOT_MESSAGE_H
#define ROBOT_MESSAGE_H

/**
\msc
arcgradient = 8;
robot [label="Main\nRobot"],
auto [label="Autonomous"],
check [label="Check\nList"],
drive [label="Drive\nTrain"],
conveyor [label="Conveyor"],
jclick [label="Pallet Jack\nClicker"],
cclick [label="Cube\nClicker"],
can [label="Pallet Jack\nCan Lifter"],
test [label="Component\nExample"];
robot=>* [label="SYSTEM_MSGTIMEOUT"];
robot=>* [label="SYSTEM_OK"];
robot=>* [label="SYSTEM_ERROR"];
robot=>* [label="STATE_DISABLED"];
robot=>* [label="STATE_AUTONOMOUS"];
robot=>* [label="STATE_TELEOPERATED"];
robot=>* [label="STATE_TEST"];
robot=>* [label="STATE_UNKNOWN"];
robot=>auto [label="RUN"];
robot=>check [label="RUN"];
robot=>conveyor [label="RUN_FWD"];
robot=>conveyor [label="RUN_BCK"];
robot=>conveyor [label="STOP"];
robot=>conveyor [label="INTAKELEFT_IN"];
robot=>conveyor [label="INTAKELEFT_OUT"];
robot=>conveyor [label="INTAKELEFT_STOP"];
robot=>conveyor [label="INTAKERIGHT_IN"];
robot=>conveyor [label="INTAKERIGHT_OUT"];
robot=>conveyor [label="INTAKERIGHT_STOP"];
robot=>conveyor [label="INTAKEBOTH_IN"];
robot=>conveyor [label="INTAKEBOTH_OUT"];
robot=>conveyor [label="INTAKEBOTH_STOP"];
robot=>conveyor [label="RUNALL_IN"];
robot=>conveyor [label="RUNALL_OUT"];
robot=>conveyor [label="RUNALL_STOP"];
robot=>jclick [label="RAISE"];
robot=>jclick [label="LOWER"];
robot=>jclick [label="STOP"];
robot=>cclick [label="RAISE"];
robot=>cclick [label="LOWER"];
robot=>cclick [label="STOP"];
robot=>cclick [label="CUBEINTAKE_RUN"];
robot=>cclick [label="CUBEINTAKE_STOP"];
robot=>cclick [label="CUBEAUTOCYCLE_START"];
robot=>cclick [label="CUBEAUTOCYCLE_STOP"];
robot=>can[label="RAISE"];
robot=>can[label="LOWER"];
robot=>can[label="STOP"];
robot=>test[label="TEST"];
\endmsc
*/

enum MessageCommand
{
	COMMAND_UNKNOWN,                  //!< COMMAND_UNKNOWN
	COMMAND_SYSTEM_MSGTIMEOUT,        //!< COMMAND_SYSTEM_MSGTIMEOUT
	COMMAND_SYSTEM_OK,                //!< COMMAND_SYSTEM_OK
	COMMAND_SYSTEM_ERROR,             //!< COMMAND_SYSTEM_ERROR

	COMMAND_ROBOT_STATE_DISABLED,     //!< COMMAND_ROBOT_STATE_DISABLED
	COMMAND_ROBOT_STATE_AUTONOMOUS,   //!< COMMAND_ROBOT_STATE_AUTONOMOUS
	COMMAND_ROBOT_STATE_TELEOPERATED, //!< COMMAND_ROBOT_STATE_TELEOPERATED
	COMMAND_ROBOT_STATE_TEST,         //!< COMMAND_ROBOT_STATE_TEST
	COMMAND_ROBOT_STATE_UNKNOWN,      //!< COMMAND_ROBOT_STATE_UNKNOWN

	COMMAND_AUTONOMOUS_RUN,           //!< COMMAND_AUTONOMOUS_RUN
	COMMAND_CHECKLIST_RUN,            //!< COMMAND_CHECKLIST_RUN

	COMMAND_DRIVETRAIN_DRIVE_TANK,    //!< COMMAND_DRIVETRAIN_DRIVE_TANK

	COMMAND_CONVEYOR_RUN_FWD,         //!< COMMAND_CONVEYOR_RUN_FWD
	COMMAND_CONVEYOR_RUN_BCK,         //!< COMMAND_CONVEYOR_RUN_BCK
	COMMAND_CONVEYOR_STOP,            //!< COMMAND_CONVEYOR_STOP
	COMMAND_CONVEYOR_INTAKELEFT_IN,   //!< COMMAND_CONVEYOR_INTAKELEFT_IN
	COMMAND_CONVEYOR_INTAKELEFT_OUT,  //!< COMMAND_CONVEYOR_INTAKELEFT_OUT
	COMMAND_CONVEYOR_INTAKELEFT_STOP, //!< COMMAND_CONVEYOR_INTAKELEFT_STOP
	COMMAND_CONVEYOR_INTAKERIGHT_IN,  //!< COMMAND_CONVEYOR_INTAKERIGHT_IN
	COMMAND_CONVEYOR_INTAKERIGHT_OUT, //!< COMMAND_CONVEYOR_INTAKERIGHT_OUT
	COMMAND_CONVEYOR_INTAKERIGHT_STOP,//!< COMMAND_CONVEYOR_INTAKERIGHT_STOP
	COMMAND_CONVEYOR_INTAKEBOTH_IN,   //!< COMMAND_CONVEYOR_INTAKEBOTH_IN
	COMMAND_CONVEYOR_INTAKEBOTH_OUT,  //!< COMMAND_CONVEYOR_INTAKEBOTH_OUT
	COMMAND_CONVEYOR_INTAKEBOTH_STOP, //!< COMMAND_CONVEYOR_INTAKEBOTH_STOP
	COMMAND_CONVEYOR_RUNALL_FWD,      //!< COMMAND_CONVEYOR_RUNALL_FWD
	COMMAND_CONVEYOR_RUNALL_BCK,      //!< COMMAND_CONVEYOR_RUNALL_BCK
	COMMAND_CONVEYOR_RUNALL_STOP,     //!< COMMAND_CONVEYOR_RUNALL_STOP

	COMMAND_JACKCLICKER_RAISE,        //!< COMMAND_JACKCLICKER_RAISE
	COMMAND_JACKCLICKER_LOWER,        //!< COMMAND_JACKCLICKER_LOWER
	COMMAND_JACKCLICKER_STOP,         //!< COMMAND_JACKCLICKER_STOP

	COMMAND_CUBECLICKER_RAISE,        //!< COMMAND_CUBECLICKER_RAISE
	COMMAND_CUBECLICKER_LOWER,        //!< COMMAND_CUBECLICKER_LOWER
	COMMAND_CUBECLICKER_STOP,         //!< COMMAND_CUBECLICKER_STOP
	COMMAND_CUBEINTAKE_RUN,           //!< COMMAND_CUBEINTAKE_RUN
	COMMAND_CUBEINTAKE_STOP,          //!< COMMAND_CUBEINTAKE_STOP
	COMMAND_CUBEAUTOCYCLE_START,      //!< COMMAND_CUBEAUTOCYCLE_START
	COMMAND_CUBEAUTOCYCLE_STOP,       //!< COMMAND_CUBEAUTOCYCLE_STOP

	COMMAND_CANLIFTER_RAISE,          //!< COMMAND_CANLIFTER_RAISE
	COMMAND_CANLIFTER_LOWER,          //!< COMMAND_CANLIFTER_LOWER
	COMMAND_CANLIFTER_STOP,           //!< COMMAND_CANLIFTER_STOP

	COMMAND_COMPONENT_TEST,           //!< COMMAND_COMPONENT_TEST

	COMMAND_LAST                      //!< COMMAND_LAST
};

struct TankDriveParams
{
	float left;
	float right;
};

struct AutonomousParams
{
	unsigned uMode;
	unsigned uDelay;
};

union MessageParams
{
	TankDriveParams tankDrive;	
	AutonomousParams autonomous;
};

struct RobotMessage
{
	MessageCommand command;
	int replyQ;
	MessageParams params;
};

#endif //ROBOT_MESSAGE_H
