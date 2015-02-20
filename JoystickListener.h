/*
 * ButtonListener.h
 *
 *  Created on: Feb 17, 2015
 *      Author: Cyber
 */

#ifndef RHS_2015_WORKSPACE_2_SRC_JOYSTICKLISTENER_H_
#define RHS_2015_WORKSPACE_2_SRC_JOYSTICKLISTENER_H_

#include "WPILib.h"
class JoystickListener
{
public:
	JoystickListener(Joystick*);
	~JoystickListener();
	void FinalUpdate();///Call at the END of the run function
	bool ButtonPressed(unsigned int);
	bool ButtonReleased(unsigned int);
	bool AxisMoved(unsigned int);
	void SetAxisTolerance(float);
	float GetAxisTolerance();
private:
	Joystick *stick;
	std::vector<bool> buttonsDown;
	std::vector<float> axisValues;
	float axisTolerance;
};



#endif /* RHS_2015_WORKSPACE_2_SRC_JOYSTICKLISTENER_H_ */
