/*
 * File Name: mcu.c
 *
 * Description: Contains general microcontroller level functions
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include "mcu.h"
#include "trf79xxa.h"

//*****************************************************************************
//
//! \addtogroup mcu_api General MSP430FR4133 API's
//! @{
//!
//! This section contains descriptions for general MSP430FR4133 functions
//! including but not limited to clock configuration and timers for the
//! MSP430FR4133.
//
//*****************************************************************************

static volatile uint8_t g_ui8Heartbeat = 1;

//===========================================================================
//
//! MCU_setCounter - Set up the counter for Timer A0
//!
//! Set up the counter for Timer A0 and enable the interrupt
//!
//! \return None.
//
//===========================================================================

void MCU_setCounter(uint16_t ui16mSecTimeout)
{
	// Max delay = 3.2 seconds (to cover maximum VLO speed of 20kHz)
	if (ui16mSecTimeout > 16000)
	{
		ui16mSecTimeout = 16000;
	}

	TA0CTL |= TACLR;					// Clear the TImer
	TA0CTL |= TASSEL_1 + ID_3 + TAIE;	// ACLK, Div 8, Interrupt Enable, Timer Stop

	TA0CCR0 = ui16mSecTimeout << 2;		// Clock runs at 32768/8 = 4,096 Hz. Shift left 2 = multiply by 4 in order to set timer for approx. millisecond delays

	TA0R = 0x0000;
	TA0CCTL0 |= CCIE;		// Compare interrupt enable

	TA0CTL |= MC_1;			//start counter in up mode
}

//===========================================================================
//
//! MCU_delayMillisecond - Delay for inputted number of millisecond
//!
//! \param n_ms is the number of milliseconds to delay by
//!
//! Blocking function to delay is approximately one millisecond, looping
//! until the inputted number of milliseconds have gone by. DELAY_1ms must
//! be calibrated based on the clock speed of the MCU.
//!
//! \return None.
//
//===========================================================================

void MCU_delayMillisecond(uint32_t n_ms)
{
    while (n_ms--)
    {
    	__delay_cycles(DELAY_1ms);		// clock speed in Hz divined by 1000
    }
}

//===========================================================================
//
//! MCU_initClock - Calibrate the Oscillator
//!
//! \param ui32Freq is the clock frequency (in kHz) to set in the MSP430FR4133
//!
//! Function to calibrate the DCO of the MSP430FR4133.
//!
//! \return None.
//
//===========================================================================

void MCU_initClock(uint32_t ui32Freq)
{	// freq = Frequency in kHz
	uint32_t ui32Ratio;

	ui32Ratio = ((ui32Freq*1000)/32768) + 1;
	CS_initFLLSettle(ui32Freq,ui32Ratio);
}

//===========================================================================
//
//! MCU_setHeartbeat - Handle the timer for the LED Heartbeat
//!
//! Function starts the 500 millisecond timer which is interrupt driven in
//! order to blink the heartbeat on the MSP-EXP430FR4133 LaunchPad LCD.
//!
//! \return None.
//
//===========================================================================

void MCU_setHeartbeat(void)
{
	TA1CTL |= TACLR;							// Clear the Timer
	TA1CTL |= TASSEL_1 + ID_3 + TAIE;			// ACLK, Div 8, Interrupt Enable, Timer Stop

	TA1R = 0x0000;
	TA1CCTL0 |= CCIE;							// Compare interrupt enable

	TA1CCR0 = 4*500;							// 500 ms Timer
	TA1CTL |= MC_1;								// Start counter in up mode
}


#pragma vector=TIMER1_A0_VECTOR
__interrupt void
TimerA1Handler(void)
{
	TA1CTL &= ~(MC0 + MC1);	//stops the counter

	if (g_ui8Heartbeat == 1)
	{
		LCDMEM[12] |= 0x04;
		g_ui8Heartbeat = 0;
	}
	else
	{
		LCDMEM[12] = (LCDMEM[12] & 0xFB);
		g_ui8Heartbeat = 1;
	}

	MCU_setHeartbeat();	// Restart the counter
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
