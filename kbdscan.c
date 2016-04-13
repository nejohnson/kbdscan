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

#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>

/*****************************************************************************
   Project Includes
 *****************************************************************************/

#include "hostif.h"
#include "ctrl.h"
#include "scanner.h"

/*****************************************************************************
  Private typedefs, macros and constants.
 *****************************************************************************/

/* Timer0 counts up to 255 and then overflows, triggering an interrupt.  So
   we set the reload value to give the correct number of timer ticks for
   our required time interval, in units of 8us.
*/
#define TIMER_PERIOD_US		( 1000 )
#define TIMER0_RELOAD		( 256 - ( TIMER_PERIOD_US / 8 ) )

/*****************************************************************************
  Private data.  Declare as static.
 *****************************************************************************/

static volatile unsigned char sync = 0;

/*****************************************************************************
   Private Function Declarations.  Declare as static.
 *****************************************************************************/

static void timer_init( void );

/*****************************************************************************
   Private Functions.  Declare as static.
 *****************************************************************************/

/*****************************************************************************/
/**
	Initialise Timer 0 to trigger an interrupt on overflow
**/
static void timer_init( void )
{
	/* Set input to PCK0/64, i.e. base time unit is 8us */
	TCCR0 = ( 0 << CS02 ) | ( 1 << CS01 ) | ( 1 << CS00 );

	/* Initialise with reload value */
	TCNT0 = TIMER0_RELOAD;

	/* Enable Timer0 compare-match interrupt */
	TIMSK |= ( 1 << TOIE0 );
}

/*****************************************************************************/
/**
	Timer 0 Interrupt handler - set the sync flag and reload counter.
**/
ISR( TIMER0_OVF_vect )
{
	sync = 1;
	TCNT0 = TIMER0_RELOAD;
}

/*****************************************************************************
   Public Functions.  Declare in header file.
 *****************************************************************************/

/*****************************************************************************/
/**
   Keyboard scanner main function
   
   @return     IGNORED
**/
int main(void)
{
	/* Initialise the system blocks */
	hostif_init();
	ctrl_init();
	scanner_init();
	timer_init();

	/* Turn on interrupts, all systems go */
	sei();

	/* Main loop - never exits */
	while ( 1 )
	{
		/* Sit here waiting for the sync flag to be raised by the timer */
		while ( !sync ) /* spin */ ;

		/* Reset the sync flag */
		sync = 0;

		/* Now run the cyclic tasks */
		scanner_update();
		ctrl_update();
	}

	return 0;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
