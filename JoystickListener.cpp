/*
 * ButtonListener.cpp
 *
 *  Created on: Feb 17, 2015
 *      Author: Cyber
 */

#include "JoystickListener.h"
#include "WPILib.h"

JoystickListener::JoystickListener(Joystick *j) {
	stick = j;
	axisTolerance = .01;
	buttonCount = -1;
	axisCount = -1;
	ResetVectors();
}
JoystickListener::~JoystickListener() {

}
void JoystickListener::ResetVectors() {

	if(buttonCount != stick->GetButtonCount())
	{
		buttonCount = stick->GetButtonCount();
		for(int i = 0; i < buttonCount; i++)
		{
			buttonsDown.push_back(stick->GetRawButton(i));
		}
	}
	if(axisCount != stick->GetAxisCount())
	{
		axisCount = stick->GetAxisCount();
		for(int i = 0; i < axisCount; i++)
		{
			axisValues.push_back(stick->GetRawAxis(i));
		}
	}
}

void JoystickListener::FinalUpdate() {
	for(int i = 0; i < buttonCount; i++)
	{
		buttonsDown[i] = stick->GetRawButton(i);
	}
	for(int i = 0; i < axisCount; i++)
	{
		axisValues[i] = stick->GetRawAxis(i);
	}
}

bool JoystickListener::ButtonPressed(int button) {
	if(button < buttonCount && stick->GetRawButton(button)
			&& !buttonsDown[button])
	{
		return true;
	}
	return false;
}

bool JoystickListener::ButtonReleased(int button) {
	if(button < buttonCount && !stick->GetRawButton(button)
			&& buttonsDown[button])
	{
		return true;
	}
	return false;
}

bool JoystickListener::AxisMoved(int axis) {
	if(axis < axisCount
			&& abs(stick->GetRawAxis(axis) - axisValues[axis]) > axisTolerance)
	{
		return true;
	}
	return false;
}
void JoystickListener::SetAxisTolerance(float tolerance) {
	axisTolerance = tolerance;
}
