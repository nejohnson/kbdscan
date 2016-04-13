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
#include <util/delay_basic.h>
#include <avr/interrupt.h>

/*****************************************************************************
   Project Includes
 *****************************************************************************/

#include "scanner.h"
#include "hostif.h"

/*****************************************************************************
  Private typedefs, macros and constants.
 *****************************************************************************/

#define TIME_INIT		( 255 )

/*****************************************************************************
  Private data.  Declare as static.
 *****************************************************************************/

static unsigned char state[8];
static unsigned char t[64];

/*****************************************************************************
   Public Functions.  Declare in header file.
 *****************************************************************************/

/*****************************************************************************/
/**
   Initialise the keyboard scanner.
**/
void scanner_init( void )
{
	DDRD = 0;   /* Port D: key sense   (input ) */

	/* Pins 0 and 1 of Ports B and C are the demux controls (output) */
	DDRB |= ( 1 << DDB0 ) | ( 1 << DDB1 );
	DDRC |= ( 1 << DDC0 ) | ( 1 << DDC1 );
}

/* Set up the various output pins to control the demux */
/* Note: because PortB is also modified by the SPI interrupt handler we
 *        need to modify PortB in a critical section.
 */
#define SET_DMUX(n)		do { unsigned char t = (n);							\
                        PORTC = t & 0x03;									\
						cli();												\
						PORTB = ( PORTB & ~0x03 ) | ( ( t >> 2 ) & 0x03 );	\
						sei();												\
						} while(0);

/*****************************************************************************/
/**
	Scan the keyboard and emit any key up/down messages.

	The keyboard is scanned in eight banks of 8 keys, except the first bank
	which only has 5 keys.
	For each bank we sample the upper and lower bars, from which we can
	determine whether a key is up, down, or mid-flight.  For speed of execution
	we run the key state machines as single-bit operations so we can compute
	eight of them in parallel using bitwise operations, so-called SWAR
	(SIMD Within A Register).
	Each time round, the time-of-flight counters are decremented.  At state
	change times we sample the TOF counters when generating key messages.
	Debounce is included in the state machine design so we don't generate
	spurious messages.
	The TOF counters are reloaded with the initial value when we detect the
	start of a key transition (up or down).  While the key is in flight the
	counter decrements, until we reach the destination.  The value of the
	TOF counter then gives an indication of the speed at which a key is
	played or released.
**/
void scanner_update( void )
{
	unsigned char i;
	unsigned char nn = 0;
	unsigned char muxaddr = 0;

	/* Process each bank of 8 keys */
	for ( i = 0; i < 8; i++ )
	{
		unsigned char up, dn, current_state;
		unsigned char a, b, r;		

		/* Read UP and DN status */
		SET_DMUX( muxaddr++ );
		_delay_loop_1(5);	/* allow lines to settle before reading */
		dn = PIND;

		SET_DMUX( muxaddr++ );
		_delay_loop_1(5);	/* allow lines to settle before reading */
		up = PIND;

		/* Cache current state */
		current_state = state[i];
		
		/* Compute state actions a-d */
		a =   ~current_state &  up & ~dn;
		b =    current_state & ~up &  dn;
		r = ( ~current_state & ~up &  dn ) | 
		    (  current_state &  up & ~dn );

		/* Check if there's anything to do for this bank */
		if ( a | b | r )
		{
			unsigned char bit;

			for ( bit = 1; bit != 0; bit <<= 1)
			{
				if ( r & bit )
					t[nn] = TIME_INIT;

				if ( a & bit )
					hostif_msg_key_on( nn, t[nn] );

				if ( b & bit )
					hostif_msg_key_off( nn, t[nn] );

				nn++;
			}
		}
		else /* Nothing to do, so bump to next bank */
		{
			nn += 8;
		}

		/* Update state */
		state[i] ^= a | b;
	}

	/* Update flight timers */
	for ( i = 3; i < 64; i++ )
		if ( t[i] )
			t[i]--;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
