/**
 * Joystick Listener class.
 *
 * The JoystickListener class monitors the inputs of a joystick
 * and can be used to register when a button was pressed or released.
 *
 * the intput's ID is 1 greater than its representative location in
 * the vector, which is accounted for in the code.
 *
 * NOTE: the axis movement feature currently does not work.
 */

#include "JoystickListener.h"
#include "RobotParams.h"
#include "WPILib.h"

#include <cmath>

JoystickListener::JoystickListener(Joystick *j) {
	stick = j;
	axisTolerance = .0001;
	for(int i = 0; i < JOYSTICK_BUTTON_COUNT; i++)
	{
		buttonsDown.push_back(stick->GetRawButton(i + 1));
	}
	for(int i = 0; i < JOYSTICK_AXIS_COUNT; i++)
	{
		//the ID is 1 greater than the location in the vector
		axisValues.push_back(stick->GetRawAxis(i + 1));
	}
}
JoystickListener::~JoystickListener() {

}

void JoystickListener::FinalUpdate() {
	/// Be sure to call this at the END of the run function
	for(unsigned int i = 0; i <= buttonsDown.size(); i++)
	{
		buttonsDown[i] = stick->GetRawButton(i + 1);
	}
	for(unsigned int i = 0; i <= axisValues.size(); i++)
	{
		axisValues[i] = stick->GetRawAxis(i + 1);
	}
}

bool JoystickListener::ButtonPressed(unsigned int button) {
	if(button > 0 && button <= buttonsDown.size() && stick->GetRawButton(button)
			&& !buttonsDown[button - 1])
	{
		return true;
	}
	return false;
}

bool JoystickListener::ButtonReleased(unsigned int button) {
	if(button > 0 && button <= buttonsDown.size()
			&& !stick->GetRawButton(button) && buttonsDown[button - 1])
	{
		return true;
	}
	return false;
}

bool JoystickListener::AxisMoved(unsigned int axis) {
	if(axis > 0 && axis <= axisValues.size()
			&& std::abs(stick->GetRawAxis(axis) - axisValues[axis - 1])
					> axisTolerance)
	{
		return true;
	}
	return false;
}
void JoystickListener::SetAxisTolerance(float tolerance) {
	axisTolerance = tolerance;
}
float JoystickListener::GetAxisTolerance() {
	return axisTolerance;
}
