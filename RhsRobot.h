/** \file
 * Main robot class.
 *
 * The RhsRobot class is the main robot class. It inherits from RhsRobotBase and MUST define the Init() function, the Run() function, and
 * the OnStateChange() function.  Messages from the DS are processed and commands.
 */
/** \mainpage
 * 	Welcome to the documentation for team 1296's 2015 robot code! Here are some pages of interest:\n
 * 	\ref joysticks \n
 * 	\ref motorID \n
 * 	\ref Cube \n
 * 	\ref RobotMessage.h \n
 * 	\ref Component
 */
#ifndef RHS_ROBOT_H
#define RHS_ROBOT_H

#include <Cube.h>
#include "WPILib.h"

#include "Autonomous.h"
#include "Conveyor.h"
#include "Drivetrain.h"
#include "CanLifter.h"
#include "Claw.h"
#include "RhsRobotBase.h"
#include "JoystickListener.h"

class RhsRobot : public RhsRobotBase
{
public:
	RhsRobot();
	virtual ~RhsRobot();

private:
	Joystick* Controller_1;
	Joystick* Controller_2;
	JoystickListener* ControllerListen_1;
	JoystickListener* ControllerListen_2;
	Drivetrain* drivetrain;
	Autonomous* autonomous;
	Conveyor* conveyor;
	Cube* cube;
	CanLifter* canlifter;
	Claw* claw;

	std::vector <ComponentBase *> ComponentSet;
	
	void Init();
	void OnStateChange();
	void Run();
	bool CheckButtonPressed(bool, bool);
	bool CheckButtonReleased(bool, bool);

	bool bLastConveyorButtonDown;
	bool bCanlifterNearBottom; //used for speed changes in driving
	const float fDriveReduction = .5;
	const float fDriveMax = 0.85;
	int iLoop;
};

#endif //RHS_ROBOT_H
