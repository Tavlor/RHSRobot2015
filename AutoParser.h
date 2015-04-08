/** \file
 * Tokens used in our scripting language
 */

#ifndef AUTOPARSER_H
#define AUTOPARSER_H

// any line in the parser file that begins with a space or a # is skipped

const char sComment = '#';
const char szDelimiters[] = " ,[]()";

typedef enum AUTO_COMMAND_TOKENS
{
	//NR - no response; R - response; _ - contained within auto thread
	AUTO_TOKEN_MODE,			//		mode block number, number(integer)
	AUTO_TOKEN_BEGIN,			//		mark beginning of mode block
	AUTO_TOKEN_END,				//		mark end of mode block
	AUTO_TOKEN_DELAY,			//		delay (seconds - float)
	AUTO_TOKEN_MOVE,			//NR	move (left & right PWM - float)
	AUTO_TOKEN_MMOVE,			//R		mmove (speed) (inches - float)
	AUTO_TOKEN_TURN,			//R		turn (degrees - float) (timeout)
	AUTO_TOKEN_STRAIGHT,		//R		straight drive (speed) (duration)
	AUTO_TOKEN_RAISE_TOTES,		//R		can lift raises tote stack
	AUTO_TOKEN_LOWER_TOTES,		//NR	can lift lowers tote stack
	AUTO_TOKEN_CLAW_OPEN,		//NR	open the can lifter claw
	AUTO_TOKEN_CLAW_CLOSE,		//NR	close the can lifter claw
	//CHECKPOINT
	AUTO_TOKEN_RAISE_CAN,		//R		lift the can beyond the stack's height
	AUTO_TOKEN_LOWER_CAN,		//NR	put the can down
	AUTO_TOKEN_FRONT_LOAD_TOTE,	//R		pull tote in the front, stop at back sensor
	AUTO_TOKEN_BACK_LOAD_TOTE,	//R		pull tote in the back, stop at front sensor
	AUTO_TOKEN_DEPOSITTOTES_BCK,//R		deposit totestack out the back
	AUTO_TOKEN_SHIFT_TOTES_FWD,	//NR	shift totestack to the front sensor
	AUTO_TOKEN_SHIFT_TOTES_BCK,	//NR	shift totestack to the back sensor
	//AUTO_TOKEN_DRIVE_TO_CAN,	//R		drive to the can
	AUTO_TOKEN_CAN_ARM_OPEN,	//NR	open the can arm
	AUTO_TOKEN_CAN_ARM_CLOSE,	//NR	close the can arm
	//Old stuff
	AUTO_TOKEN_SEEK_TOTE,		//R		seektote (timein) (timeout)
	AUTO_TOKEN_START_RAISE_TOTE,//NR	can lift begins to raise tote stack - won't stop auto
	AUTO_TOKEN_EXTEND_TOTE,		//NR	tote lift extends tote stack
	AUTO_TOKEN_RETRACT_TOTE,	//NR	tote lift retracts tote stack
	AUTO_TOKEN_CUBE_AUTO,		//NR	start the automated behaviour of the clicker
	AUTO_TOKEN_CLICKER_UP,		//NR	raise the tote clicker in the Cube
	AUTO_TOKEN_CLICKER_DOWN,	//NR	lower the tote clicker in the Cube
	AUTO_TOKEN_LAST
} AUTO_COMMAND_TOKENS;

#endif  // AUTOPARSER_H

