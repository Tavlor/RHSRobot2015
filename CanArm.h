#ifndef CANARM_H
#define CANARM_H

//Robot
#include "WPILib.h"
#include "ComponentBase.h"			//For ComponentBase class

class CanArm : public ComponentBase
{
public:
	CanArm();
	virtual ~CanArm();
	static void *StartTask(void *pThis)
	{
		((CanArm *)pThis)->DoWork();
		return(NULL);
	}

private:

	CANTalon *armMotor;

	const float fOpen = -0.65;
	const float fClose = 0.65;
	const float fArmMotorCurrentMax = 19;//WILL NEED ADJUSTING

	bool bOpening = false;
	bool bClosing = false;

	void OnStateChange();
	void Run();
	bool CheckArmCurrentOK();
};

#endif /* CANARM_H */
