/*
 * ButtonListener.h
 *
 *  Created on: Feb 17, 2015
 *      Author: Cyber
 */

#ifndef SRC_JOYSTICKLISTENER_H_
#define SRC_JOYSTICKLISTENER_H_

#include "WPILib.h"
class JoystickListener
{
public:
	JoystickListener(Joystick*);
	~JoystickListener();
	void FinalUpdate();//Call at the END of
	bool ButtonPressed(int);
	bool ButtonReleased(int);
	bool AxisMoved(int);
	void SetAxisTolerance(float);
private:
	Joystick *stick;
	std::vector<bool> buttonsDown;
	std::vector<float> axisValues;
	float axisTolerance;
	int buttonCount;
	int axisCount;

	void ResetVectors();
};



#endif /* SRC_JOYSTICKLISTENER_H_ */
