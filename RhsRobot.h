/**  Main robot class.
 *
 * The RhsRobot class is the main robot class. It inherits from RhsRobotBase and MUST define the Init() function, the Run() function, and
 * the OnStateChange() function.  Messages from the DS are processed and commands.
 */

#ifndef RHS_ROBOT_H
#define RHS_ROBOT_H

#include "WPILib.h"
#include "RhsRobotBase.h"

//Robot
#include "Drivetrain.h"
#include "Autonomous.h"
#include "Conveyor.h"
#include "Clicker.h"
#include "JackClicker.h"
#include "CanLifter.h"

class RhsRobot : public RhsRobotBase
{
public:
	RhsRobot();
	virtual ~RhsRobot();

private:
	Joystick* Controller_1;
	Joystick* Controller_2;
	Drivetrain* drivetrain;
	Autonomous* autonomous;
	Conveyor* conveyor;
	Clicker* clicker;
	JackClicker* jackclicker;
	CanLifter* canlifter;
	
	void Init();
	void OnStateChange();
	void Run();

	int iLoop;
	// joystick wasPressed variables;
	bool bwpCubeIntakeButton;

	/* Example of proper button toggle
	 * if(BUTTON_A && !bwpA) {
	 * 	 bwpA = true;
	 * 	 Thingie->DoTask(42);
	 * }
	 * else if(!BUTTON_A && bwpA) {
	 *	 bwpA = false;
	 * }
	 */
};

#endif //RHS_ROBOT_H
