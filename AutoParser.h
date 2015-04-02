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
	AUTO_TOKEN_MODE,			// mode block number, number(integer)
	AUTO_TOKEN_BEGIN,			// mark beginning of mode block
	AUTO_TOKEN_END,				// mark end of mode block
	AUTO_TOKEN_DELAY,			// delay (seconds - float)
	AUTO_TOKEN_MOVE,			// move (left & right PWM - float)
	AUTO_TOKEN_MMOVE,			// mmove (speed) (inches - float)
	AUTO_TOKEN_TURN,			// turn (degrees - float) (timeout)
	AUTO_TOKEN_SEEK_TOTE,		// seektote (timein) (timeout)
	AUTO_TOKEN_STRAIGHT,		// straight (speed)
	AUTO_TOKEN_EXTEND_TOTE,		// tote lift extends tote stack
	AUTO_TOKEN_RETRACT_TOTE,	// tote lift retracts tote stack
	AUTO_TOKEN_RAISE_TOTE,		// can lift raises tote stack
	AUTO_TOKEN_LOWER_TOTE,		// can lift lowers tote stack
	AUTO_TOKEN_CLAW_OPEN,		// open the can lifter claw
	AUTO_TOKEN_CLAW_CLOSE,		// close the can lifter claw
	AUTO_TOKEN_CLICKER_UP,		// raise the tote clicker in the Cube
	AUTO_TOKEN_CLICKER_DOWN,	// lower the tote clicker in the Cube
	AUTO_TOKEN_CUBE_AUTO,		// start the automated behaviour of the clicker
	AUTO_TOKEN_LAST
} AUTO_COMMAND_TOKENS;

#endif  // AUTOPARSER_H

