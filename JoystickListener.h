/**
 * The JoystickListener class monitors the inputs of a joystick
 * and can be used to register when a button was pressed or released.
 *
 * the input's ID is 1 greater than its representative location in
 * the vector, which is accounted for in the code.
 *
 * NOTE: the axis movement feature currently does not work.
 */

#ifndef JOYSTICKLISTENER_H_
#define JOYSTICKLISTENER_H_

#include "WPILib.h"
class JoystickListener {
public:
	JoystickListener(Joystick*);
	~JoystickListener();
	void FinalUpdate();
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

#endif /* JOYSTICKLISTENER_H_ */
