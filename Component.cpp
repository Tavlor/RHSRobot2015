/*
 * The Component class is a template for component
 * classes (simple, yes?). Be sure to replace each
 * instance of "Component" with your desired class
 * name. Leave "ComponentBase" alone.
 */

#include "WPILib.h"

//Robot
#include "ComponentBase.h"
#include "RobotParams.h"

//Local
#include "Component.h"

Component::Component()
: ComponentBase(COMPONENT_TASKNAME, COMPONENT_QUEUE, COMPONENT_PRIORITY)
{
	//TODO: add member objects
	pTask = new Task(COMPONENT_TASKNAME, (FUNCPTR) &Component::StartTask,
			COMPONENT_PRIORITY, COMPONENT_STACKSIZE);
	wpi_assert(pTask);
	pTask->Start((int)this);
};

Component::~Component()
{
	delete(pTask);
	//TODO delete member objects
};

void Component::OnStateChange()	
{
};

void Component::Run()
{
	switch(localMessage.command)			//Reads the message command
	{
	//TODO add command cases for Component
		case COMMAND_COMPONENT_TEST:
			break;

		default:
			break;
		}
};
