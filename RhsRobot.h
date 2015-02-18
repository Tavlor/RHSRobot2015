/** \file
 * Main robot class.
 *
 * The RhsRobot class is the main robot class. It inherits from RhsRobotBase and MUST define the Init() function, the Run() function, and
 * the OnStateChange() function.  Messages from the DS are processed and commands.
 */

#ifndef RHS_ROBOT_H
#define RHS_ROBOT_H

#include <Cube.h>
#include "WPILib.h"

#include "Autonomous.h"
#include "Conveyor.h"
#include "Drivetrain.h"
#include "JackClicker.h"
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
	JackClicker* jackclicker;

	std::vector <ComponentBase *> ComponentSet;
	
	void Init();
	void OnStateChange();
	void Run();
	bool CheckButtonPressed(bool, bool);
	bool CheckButtonReleased(bool, bool);

	bool bLastConveyorButtonDown;
	int iLoop;
};

#endif //RHS_ROBOT_H
