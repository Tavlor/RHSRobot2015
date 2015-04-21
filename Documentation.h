/** \file
 * A place to hold all the documentation which really has no anchor in the code.
 * It currently holds: \n
 * \ref index, \n
 * \ref style.
 */

/** \mainpage
 * 	Welcome to the documentation for team 1296's 2015 robot code! Here are some pages of interest:\n
 * 	\ref style \n
 * 	\ref joysticks \n
 * 	\ref motorID \n
 * 	\ref Cube \n
 * 	\ref RobotMessage.h \n
 * 	\ref AutoParser.h \n
 * 	\ref Component
 */

/** \page style Style Guide
 *	Some points on the matter of code formatting to keep things consistent and working.
 *
 * <b> CODING </b> \n
 *
 *	-- NEVER use "magic numbers". Instead, create a constant value with a descriptive name in the
 *		class's header file. This makes it easier to understand the number's purpose and adjust it.
 *
 *
 *	-- Name components in RhsRobot with all lower case.
 *
 *		\verbatim
 		Drivetrain *drivetrain;
 		\endverbatim
 *
 *
 *	-- ALWAYS use brackets with control structures, even if there is only one line.
 *
 *		\verbatim
 		if(bVariable)
 		{
 			DoStuff();
 		}
 		\endverbatim

 	-- When adding components to RhsRobot: use pointers; set to NULL in RhsRobot(), then construct them in Init().
 		When writing Run, check if the pointer points to something before using it.

 		\verbatim
 		//In RhsRobot.h
		Drivetrain *drivetrain;

 		//In RhsRobot.cpp
 		RhsRobot::RhsRobot()
 		{
 			drivetrain = NULL;
 		}

 		RhsRobot::Init()
 		{
 			drivetrain = new Drivetrain();
 		}

 		RhsRobot:: Run()
 		{
 			if(drivetrain)
 			{
 				//CODE
 			}
 		}
 		\endverbatim

 		This allows us to comment out the construction and effectively disable the execution of any code relating
 		to a particular component in the case of a physical detachment of the represented component.
 *
 *
 *	-- ALWAYS put breaks in your switches and repeat code if necessary. You may leave the condition
 *		empty except for the break.
 *
 *		\verbatim
 		switch(iVariable)
 		{
 		case 1:
 			DoThing();
 			break;
 		case 2:
 			DoThing();
 			break;
 		case 3:
 			break;
 		case 4:
 			DoOtherThing();
 			break;
 		default:
 			break;
 		}
 		\endverbatim
 *
 *
 *	-- Macros are your friend. Use them.
 *
 *
 *	-- A class should represent as few motor controllers as possible. Only allow multiple controllers
 *		if they are unavoidably intertwined.
 *
 *
 *		<b> AUTONOMOUS </b> \n
 *
 *		<b> DOCUMENTATION </b> \n
 */
