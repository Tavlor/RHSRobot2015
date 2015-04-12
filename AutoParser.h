/** \file
 * Tokens used in our scripting language
 */

#ifndef AUTOPARSER_H
#define AUTOPARSER_H

// any line in the parser file that begins with a space or a # is skipped

const char sComment = '#';
const char szDelimiters[] = " ,[]()";

///N - doesn't need a response; R - needs a response; _ - contained within auto thread
typedef enum AUTO_COMMAND_TOKENS
{
	AUTO_TOKEN_MODE,			//!<	mode block number, number(integer)
	AUTO_TOKEN_BEGIN,			//!<	mark beginning of mode block
	AUTO_TOKEN_END,				//!<	mark end of mode block
	AUTO_TOKEN_DELAY,			//!<	delay (seconds - float)
	AUTO_TOKEN_MOVE,			//!<N	move (left & right PWM - float)
	AUTO_TOKEN_MMOVE,			//!<R	mmove (speed) (inches - float)
	AUTO_TOKEN_TURN,			//!<R	turn (degrees - float) (timeout)
	AUTO_TOKEN_STRAIGHT,		//!<R	straight drive (speed) (duration)
	AUTO_TOKEN_CLAW_OPEN,		//!<N	open the can lifter claw
	AUTO_TOKEN_CLAW_CLOSE,		//!<N	close the can lifter claw
	//CHECKPOINT
	AUTO_TOKEN_RAISE_CLAW,		//!<R	lift the claw beyond the stack's height - to the top?
	AUTO_TOKEN_LOWER_CLAW,		//!<N	put the claw down to very bottom
	AUTO_TOKEN_RAISE_CAN,		//!<N	lift the claw (with can) to the lower hall effect
	AUTO_TOKEN_LOWER_CAN,		//!<N	lower the claw (with can) to the upper hall effect
	AUTO_TOKEN_RAISE_TOTES,		//!<R	can lift raises tote stack to accept a new tote
	AUTO_TOKEN_LOWER_TOTES,		//!<N	can lift lowers tote stack to add new tote
	AUTO_TOKEN_FRONT_LOAD_TOTE,	//!<R	pull tote in the front, stop at back sensor
	AUTO_TOKEN_BACK_LOAD_TOTE,	//!<R	pull tote in the back, stop at front sensor
	AUTO_TOKEN_DEPOSITTOTES_BCK,//!<R	deposit totestack out the back
	AUTO_TOKEN_SHIFT_TOTES_FWD,	//!<N	shift totestack to the front sensor
	AUTO_TOKEN_SHIFT_TOTES_BCK,	//!<N	shift totestack to the back sensor
	AUTO_TOKEN_CAN_ARM_OPEN,	//!<R	open the can arm to push a can away
	AUTO_TOKEN_CAN_ARM_CLOSE,	//!<N	close the can arm
	//Old stuff
	AUTO_TOKEN_SEEK_TOTE,		//!<R	seektote (timein) (timeout)
	AUTO_TOKEN_START_RAISE_TOTE,//!<N	can lift begins to raise tote stack - won't stop auto
	AUTO_TOKEN_EXTEND_TOTE,		//!<N	tote lift extends tote stack
	AUTO_TOKEN_RETRACT_TOTE,	//!<N	tote lift retracts tote stack
	AUTO_TOKEN_CUBE_AUTO,		//!<N	start the automated behaviour of the clicker
	AUTO_TOKEN_CLICKER_UP,		//!<N	raise the tote clicker in the Cube
	AUTO_TOKEN_CLICKER_DOWN,	//!<N	lower the tote clicker in the Cube
	AUTO_TOKEN_LAST
} AUTO_COMMAND_TOKENS;

#endif  // AUTOPARSER_H

