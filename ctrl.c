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

/*****************************************************************************
   Project Includes
 *****************************************************************************/

#include "ctrl.h"
#include "hostif.h"

/*****************************************************************************
  Private typedefs, macros and constants.
 *****************************************************************************/

/* Set the number of controller channels */
#define MAX_CTRL			( 3 )

/* Allow channel offset to ease PCB layout */
#define ADC_OFFSET			( 2 )

/* Macro to set the ADC channel independent of any offsets */
#define ADC_CHAN(n)			( ADC_OFFSET + (n) )

/* External Vref, data right-shifted */
#define ADMUX_CONFIG		( ( 0 << REFS1 ) | ( 0 << REFS0 ) | ( 0 << ADLAR ) )

/*****************************************************************************
   Public Functions.  Declare in header file.
 *****************************************************************************/

/*****************************************************************************/
/**
   Initialise the control reading task.
**/
void ctrl_init( void )
{
	/* Clear all but the bottom two bits of DDRC */
	DDRC &= ~0x03;

	/* Enable the ADC with a conversion clock of 125kHz. */
	ADCSRA = ( 1 << ADEN ) | ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 0 << ADPS0 );

	/* Select initial channel */
	ADMUX = ADMUX_CONFIG | ADC_CHAN( 0 );

	/* Kick-off the first conversion */
	ADCSR |= ( 1 << ADSC );
}

/*****************************************************************************/
/**
   Update the continuous controllers.
   
   Each time this function is called we process one of the ADC inputs and send
   a message to the host.
**/
void ctrl_update( void )
{
	unsigned char low, high;
	unsigned char curr_ctrl, next_ctrl;
	unsigned char sus;

	/* Wait for conversion to complete (in theory should already be done) */
	while ( !( ADCSRA & ( 1 << ADIF ) ) )  /* spin */ ;

	/* Read 10-bit ADC and channel number */
	low       = ADCL;
	high      = ADCH;
	curr_ctrl = ( ADMUX & 0x0F ) - ADC_OFFSET;

	/* Also read the sustain pedal input */
	sus = !( PINC & ( 1 << PINC5 ) );

	/* Switch to next channel */
	next_ctrl = curr_ctrl + 1;
	if ( next_ctrl >= MAX_CTRL )
		next_ctrl = 0;
	ADMUX = ADMUX_CONFIG | ADC_CHAN( next_ctrl );

	/* Kick off the next conversion */
    ADCSR |= ( 1 << ADSC );

	/* Send the value to the master processor */
	hostif_msg_ctrl( curr_ctrl, sus, ( ( high << 8 ) | low ) << 2 );
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
