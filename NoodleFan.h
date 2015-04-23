/** \file
 * Definitions of class to control anti-noodle fan on the pallet jack.
 *
 * This classes is derived from the standard Component base class and includes
 * definitions for the device used to control the pallet jack's noodle fan.
 */

#ifndef NOODLEFAN_H
#define NOODLEFAN_H

//Robot
#include "WPILib.h"
#include "ComponentBase.h"			//For ComponentBase class

class NoodleFan : public ComponentBase
{
public:
	NoodleFan();
	virtual ~NoodleFan();
	static void *StartTask(void *pThis)
	{
		((NoodleFan *)pThis)->DoWork();
		return(NULL);
	}

private:

	CANTalon *fanMotor;
	const float fPower = 1.0;

	bool bBlowing = false;

	void OnStateChange();
	void Run();
};

#endif /* NOODLEFAN_H */
