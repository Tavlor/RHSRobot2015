/** \file
 *  Messages used for intertask communications
 */

/** Defines the messages we pass from task to task.
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
 cclick [label="Cube\nClicker"],
 can [label="Pallet Jack\nCan Lifter"],
 claw [label="Pallet Jack\nClaw"],
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
 robot=>drive [label="STOP"];
 robot=>drive [label="DRIVE_TANK"];
 robot=>drive [label="DRIVE_ARCADE"];
 auto=>drive [label="DRIVE_STRAIGHT"];
 auto=>drive [label="TURN"];
 auto=>drive [label="SEEK_TOTE"];
 drive=>auto [label="AUTONOMOUS_RESPONSE_OK"]
 drive=>auto [label="AUTONOMOUS_RESPONSE_ERROR"]
 robot=>conveyor [label="RUN_FWD"];
 robot=>conveyor [label="RUN_BCK"];
 robot=>conveyor [label="STOP"];
 auto=>conveyor [label="WATCH_TOTE_FRONT/BACK"];
 auto=>conveyor [label="FRONT/BACKLOAD_TOTE"];
 auto=>conveyor [label="SHIFTTOTES_FWD/BCK"];
 auto=>conveyor [label="DEPOSITTOTES_BCK"];
 robot=>cclick [label="RAISE"];
 robot=>cclick [label="LOWER"];
 robot=>cclick [label="STOP"];
 robot=>cclick [label="CUBEINTAKE_RUN"];
 robot=>cclick [label="CUBEINTAKE_STOP"];
 robot=>cclick [label="CUBEAUTOCYCLE_START"];
 robot=>cclick [label="CUBEAUTOCYCLE_STOP"];
 robot=>cclick [label="CUBEAUTOCYCLE_HOLD"];
 robot=>cclick [label="CUBEAUTOCYCLE_RELEASE"];
 robot=>can[label="RAISE"];
 robot=>can[label="LOWER"];
 robot=>can[label="STOP"];
 auto=>can[label="RAISE/LOWER_TOTES"];
 auto=>can[label="RAISE/LOWER_CLAW"];
 robot=>claw[label="OPEN"];
 robot=>claw[label="CLOSE"];
 robot=>test[label="TEST"];
 \endmsc

 */

enum MessageCommand {
	COMMAND_UNKNOWN,					//!< COMMAND_UNKNOWN
	COMMAND_SYSTEM_MSGTIMEOUT,			//!< COMMAND_SYSTEM_MSGTIMEOUT
	COMMAND_SYSTEM_OK,					//!< COMMAND_SYSTEM_OK
	COMMAND_SYSTEM_ERROR,				//!< COMMAND_SYSTEM_ERROR

	COMMAND_ROBOT_STATE_DISABLED,		//!< Tells all components that the robot is disabled
	COMMAND_ROBOT_STATE_AUTONOMOUS,		//!< Tells all components that the robot is in auto
	COMMAND_ROBOT_STATE_TELEOPERATED,	//!< Tells all components that the robot is in teleop
	COMMAND_ROBOT_STATE_TEST,			//!< Tells all components that the robot is in test
	COMMAND_ROBOT_STATE_UNKNOWN,		//!< Tells all components that the robot's state is unknown

	COMMAND_AUTONOMOUS_RUN,				//!< Tells Autonomous to run
	COMMAND_AUTONOMOUS_COMPLETE,		//!< Tells all components that Autonomous is done running the script
	COMMAND_AUTONOMOUS_RESPONSE_OK,		//!< Tells Autonomous that a command finished running successfully
	COMMAND_AUTONOMOUS_RESPONSE_ERROR,	//!< Tells Autonomous that a command had a error while running
	COMMAND_CHECKLIST_RUN,				//!< Tells CheckList to run

	COMMAND_DRIVETRAIN_STOP,			//!< Tells Drivetrain to stop moving
	COMMAND_DRIVETRAIN_DRIVE_TANK,		//!< Tells Drivetrain to use tank drive
	COMMAND_DRIVETRAIN_DRIVE_ARCADE,	//!< Tells Drivetrain to use arcade drive
	COMMAND_DRIVETRAIN_AUTO_MOVE,		//!< Tells Drivetrain to move motors, used by Autonomous
	COMMAND_DRIVETRAIN_DRIVE_STRAIGHT,	//!< Tells Drivetrain to drive straight, used by Autonomous
	COMMAND_DRIVETRAIN_TURN,			//!< Tells Drivetrain to turn, used by Autonomous
	COMMAND_DRIVETRAIN_SEEK_TOTE,		//!< Tells Drivetrain to seek the next tote, used by Autonomous
	COMMAND_DRIVETRAIN_START_DRIVE_FWD,	//!< Tells Drivetrain to front load the next tote, used by Autonomous
	COMMAND_DRIVETRAIN_START_DRIVE_BCK,	//!< Tells Drivetrain to back load the next tote, used by Autonomous
	COMMAND_DRIVETRAIN_START_KEEPALIGN,	//!< Tells Drivetrain to start keeping itself at constant alignment, used by Autonomous
	COMMAND_DRIVETRAIN_STOP_KEEPALIGN,	//!< Tells Drivetrain to stop keeping itself at constant alignment, used by Autonomous

	COMMAND_CONVEYOR_RUN_FWD,			//!< Tells Conveyor to run the rollers forward
	COMMAND_CONVEYOR_RUN_BCK,			//!< Tells Conveyor to run the rollers backwards - fancy operations
	COMMAND_CONVEYOR_SET_BACK,			//!< Tells Conveyor to set the rollers backwards - simple
	COMMAND_CONVEYOR_STOP,				//!< Tells Conveyor to stop the rollers
	COMMAND_CONVEYOR_SEEK_TOTE_FRONT,	//!< Tells Conveyor to watch for next tote in the front, used by Autonomous
	COMMAND_CONVEYOR_SEEK_TOTE_BACK,	//!< Tells Conveyor to watch for next tote in the back, used by Autonomous
	COMMAND_CONVEYOR_FRONTLOAD_TOTE,	//!< Tells Conveyor to front load the next tote, used by Autonomous
	COMMAND_CONVEYOR_BACKLOAD_TOTE,		//!< Tells Conveyor to back load the next tote, used by Autonomous
	COMMAND_CONVEYOR_SHIFTTOTES_FWD,	//!< Tells Conveyor to shift the totes to the front sensor, used by Autonomous
	COMMAND_CONVEYOR_SHIFTTOTES_BCK,	//!< Tells Conveyor to shift the totes to the back sensor, used by Autonomous
	COMMAND_CONVEYOR_PUSHTOTES_BCK,		//!< Tells Conveyor to push the totes into the stack hook, used by Autonomous
	COMMAND_CONVEYOR_DEPOSITTOTES_BCK,	//!< Tells Conveyor to dump the tote stack out the back, used by Autonomous
	COMMAND_CONVEYOR_WAIT_FRONT_BEAM,	//!< Tells Conveyor to dump the tote stack out the back, used by Autonomous
	COMMAND_CONVEYOR_WAIT_BACK_BEAM,	//!< Tells Conveyor to dump the tote stack out the back, used by Autonomous

	COMMAND_CANLIFTER_RAISE,			//!< Tells CanLifter to raise the lift
	COMMAND_CANLIFTER_LOWER,			//!< Tells CanLifter to lower the lift
	COMMAND_CANLIFTER_HOVER,			//!< Tells CanLifter to hold the lift where it is
	COMMAND_CANLIFTER_TOGGLE_HOVER,		//!< Tells CanLifter to toggle hovering
	COMMAND_CANLIFTER_RAISE_TOTES,		//!< Tells CanLifter to raise to loading position & hover, used by Autonomous
	COMMAND_CANLIFTER_LOWER_TOTES,		//!< Tells CanLifter to lower to hook position, used by Autonomous
	COMMAND_CANLIFTER_START_RAISE_TOTES,//!< Tells CanLifter to raise to loading position & hover, used by Autonomous
	COMMAND_CANLIFTER_CLAW_TO_TOP,		//!< Tells CanLifter to raise to top, used by Autonomous
	COMMAND_CANLIFTER_CLAW_TO_BOTTOM,	//!< Tells CanLifter to lower to bottom, used by Autonomous
	COMMAND_CANLIFTER_RAISE_LOMID,		//!< Tells CanLifter to raise to hall effect & hover, used by Autonomous
	COMMAND_CANLIFTER_LOWER_HIMID,		//!< Tells CanLifter to lower to hall effect & hover, used by Autonomous
	COMMAND_CANLIFTER_STOP,				//!< Tells CanLifter to stop the lift

	COMMAND_CLAW_OPEN,					//!< Tells CanLifter to open the claw
	COMMAND_CLAW_CLOSE,					//!< Tells CanLifter to close the claw
	COMMAND_CLAW_STOP,					//!< Tells CanLifter to stop the claw

	COMMAND_CANARM_OPEN,				//!< Tells CanArm to open the arm - push away a can
	COMMAND_CANARM_CLOSE,				//!< Tells CanArm to close the arm
	COMMAND_CANARM_STOP,				//!< Tells CanArm to stop

	COMMAND_CUBECLICKER_RAISE,			//!< Tells Cube to raise the clicker
	COMMAND_CUBECLICKER_LOWER,			//!< Tells Cube to lower the clicker
	COMMAND_CUBECLICKER_STOP,			//!< Tells Cube to stop the clicker
	COMMAND_CUBEINTAKE_RUN,				//!< Tells Cube to start the intake
	COMMAND_CUBEINTAKE_STOP,			//!< Tells Cube to stop the intake (not used)

	COMMAND_CUBEAUTOCYCLE_START,		//!< Tells Cube to start autocycle
	COMMAND_CUBEAUTOCYCLE_STOP,			//!< Tells Cube to stop autocycle
	COMMAND_CUBEAUTOCYCLE_PAUSE,		//!< Tells Cube to pause autocycle
	COMMAND_CUBEAUTOCYCLE_RESUME,		//!< Tells Cube to resume autocycle
	COMMAND_CUBEAUTOCYCLE_HOLD, 		//!< Tells Cube to prepare for pallet jack to remove stack
	COMMAND_CUBEAUTOCYCLE_RELEASE, 		//!< Tells Cube to continue normal autocycle
	COMMAND_CUBE_STOP,					//!< Tells Cube to stop all components

	COMMAND_COMPONENT_TEST,				//!< COMMAND_COMPONENT_TEST

	COMMAND_LAST                      //!< COMMAND_LAST 
};
///Used to deliver joystick readings to Drivetrain
struct TankDriveParams {
	float left;
	float right;
};

///Used to deliver joystick readings to Drivetrain
struct ArcadeDriveParams {
	float x;
	float y;
};

///Used to deliver joystick readings to Conveyor
struct ConveyorParams {
	bool bButtonWentDownEvent;
	float right;
	float intakeSpeed;
};

///Used to deliver can lifter params
struct CanLifterParams {
	float lifterSpeed;
	int iNumTotes;
};

///Used to deliver autonomous values to Drivetrain
struct AutonomousParams {
	unsigned uMode;
	unsigned uDelay;
	///how long a function can run, maximum
	float timeout;
	///how long until a function performs
	float timein;

	///used by drivetrain for straight driving
	float driveSpeed;
	float driveDistance;
	float turnAngle;
	float driveTime;
};

///Contains all the parameter structures contained in a message
union MessageParams {
	TankDriveParams tankDrive;
	ArcadeDriveParams arcadeDrive;
	ConveyorParams conveyorParams;
	CanLifterParams canLifterParams;
	AutonomousParams autonomous;
};

///A structure containing a command, a set of parameters, and a reply id, sent between components
struct RobotMessage {
	MessageCommand command;
	const char* replyQ;
	MessageParams params;
};

#endif //ROBOT_MESSAGE_H
