/* *****************************************************************************
 * Copyright (C) 2009, Neil Johnson
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms,
 * with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of Neil Johnson nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ************************************************************************** */

/*****************************************************************************
   System Includes
 *****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>

/*****************************************************************************
   Project Includes
 *****************************************************************************/

#include "hostif.h"

/*****************************************************************************
  Private typedefs, macros and constants.
 *****************************************************************************/

/* None */

/*****************************************************************************
  Private data.  Declare as static.
 *****************************************************************************/

/* None */

/*****************************************************************************
   Private Function Declarations.  Declare as static.
 *****************************************************************************/

static char push_msg( unsigned int );

/*****************************************************************************
   Private Functions.  Declare as static.
 *****************************************************************************/

/*****************************************************************************/
/**
	Transmit the message MSB first.

	@param msg		16-bit message to transmit.

	@return 0 if failed, 1 if successful.
**/
static char push_msg( unsigned int msg )
{
	/* SS low */
	PORTB &= ~( 1 << PORTB2 );

	/* Transmit high byte followed by low byte */
	SPDR = msg >> 8;
	while ( !( SPSR & ( 1 << SPIF ) ) ) /* spin */ ;
	SPDR = msg & 0xFF;
	while ( !( SPSR & ( 1 << SPIF ) ) ) /* spin */ ;

	/* SS high */
	PORTB |= 1 << PORTB2;

	return 1;
}

/*****************************************************************************
   Public Functions.  Declare in header file.
 *****************************************************************************/

/*****************************************************************************/
/**
   Initialise the host interface.
**/
void hostif_init( void )
{
	/* Set SCK, MOSI and SS as outputs, with SS high */
	PORTB |= 1 << PORTB2;
	DDRB  |= ( 1 << DDB5 ) | ( 1 << DDB3 ) | ( 1 << DDB2 );

	SPCR = ( 1 << SPE  ) |  /* Enable SPI peripheral 				*/
		   ( 0 << DORD ) |  /* Send MSB first 						*/
		   ( 1 << MSTR ) |  /* Set MASTER mode 						*/
		   ( 0 << CPOL ) |  /* SCLK low when idle 					*/
		   ( 1 << CPHA ) |  /* Setup rising, sample falling 		*/
		   ( 0 << SPR1 ) |  /* Set divisor to 4                  	*/
		   ( 1 << SPR0 );

	/* Double speed up to 1Mbps */
	SPSR = 1 << SPI2X;
}

/*****************************************************************************

	SPI Message format:

	Each message is 16 bits long.

	bit[15] : 0 = key, 1 = control

	key:	bit[14]   : 0 = on/down, 1 = off/up
	        bit[13:8] : key number (3-63)  -- 0..2 not used
			bit[ 7:0] : velocity (0-255)

	control:bit[14:13]: control number (0-3)
	        bit[   12]: sustain pedal (0 = up, 1 = down/pressed)
	        bit[11: 0]: reading (12-bit, left-justified)

 *****************************************************************************/

#define MSG_KEY				( 0 << 15 )
#define MSG_CTRL    		( 1 << 15 )

#define KEY_ON  			( 0 << 14 )
#define KEY_OFF 			( 1 << 14 )
#define KEY_NUM_SHIFT 		( 8 )
#define KEY_NUM_MASK  		( 0x3F )
#define KEY_VEL_SHIFT		( 0 )
#define KEY_VEL_MASK		( 0xFF )

#define CTRL_SHIFT			( 13 )
#define CTRL_MASK			( 0x03 )
#define CTRL_SUS_SHIFT		( 12 )
#define CTRL_SUS_MASK		( 0x01 )
#define CTRL_READ_SHIFT		( 0 )
#define CTRL_READ_MASK		( 0xFFF )

/*****************************************************************************/
/**
	Build and send Key On message.

	@param key			Number of key that has been pressed.
	@param velocity		Velocity reading from pressed note.
**/
void hostif_msg_key_on ( unsigned char key, unsigned char velocity )
{
	push_msg(   MSG_KEY 
				| KEY_ON 
				| ( (      key & KEY_NUM_MASK ) << KEY_NUM_SHIFT ) 
				| ( ( velocity & KEY_VEL_MASK ) << KEY_VEL_SHIFT ) );
}

/*****************************************************************************/
/**
	Build and send Key Off message.

	@param key			Number of key that has been released.
	@param velocity		Velocity reading from released note.
**/
void hostif_msg_key_off ( unsigned char key, unsigned char velocity )
{
	push_msg(   MSG_KEY 
	            | KEY_OFF 
				| ( (      key & KEY_NUM_MASK ) << KEY_NUM_SHIFT ) 
				| ( ( velocity & KEY_VEL_MASK ) << KEY_VEL_SHIFT ) );
}

/*****************************************************************************/
/**
	Build and send Control message.

	@param ctrl			Control number (0-3).
	@param sus          Sustain pedal (0/1).
	@param reading		Current reading from control (0 - 0xFFF).
**/
void hostif_msg_ctrl ( unsigned char ctrl, unsigned char sus, unsigned int reading )
{
	push_msg(   MSG_CTRL 
	            | ( (    ctrl &      CTRL_MASK ) <<      CTRL_SHIFT )
				| ( (     sus &  CTRL_SUS_MASK ) <<  CTRL_SUS_SHIFT ) 
				| ( ( reading & CTRL_READ_MASK ) << CTRL_READ_SHIFT ) );
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
