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
 *	-- ALWAYS use brackets with control structures, even if there is only one line.
 *
 *		\verbatim
 		if(bVariable)
 		{
 			DoStuff();
 		}
 		\endverbatim
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
 				DoOtherThing();
 				break;
 			default:
 		}
 		\endverbatim
 *
 *
 *	-- Macros are your friend. Use them.
 *
 *	-- A class should represent as few motor controllers as possible. Only allow multiple controllers
 *		if they are unavoidably intertwined.
 *
 *	-- When chec
 *		<b> AUTONOMOUS </b> \n
 *
 *		<b> DOCUMENTATION </b> \n
 */
