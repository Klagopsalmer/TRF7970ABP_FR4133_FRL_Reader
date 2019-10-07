/*******************************************************************************
 *
 *  HAL_Buttons.c - Driver for the buttons
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
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
 ******************************************************************************/

/***************************************************************************//**
 * @file       HAL_Buttons.c
 * @addtogroup HAL_Buttons
 * @{
 ******************************************************************************/
#include "hal_Buttons.h"

volatile uint8_t buttonDebounce = 0;
volatile uint8_t buttonsPressed = 0;

/***************************************************************************//**
 * @brief  Sets up the WDT as a button debouncer, only activated once a
 *         button interrupt has occurred.
 * @param  none
 * @return none
 ******************************************************************************/

void Buttons_startWDT(void)
{
    // WDT as 250ms interval counter
    SFRIFG1 &= ~WDTIFG;
    WDTCTL = WDTPW + WDTSSEL_0 + WDTTMSEL + WDTCNTCL + WDTIS_4;
    SFRIE1 |= WDTIE;
}

/***************************************************************************//**
 * @brief  Handles Watchdog Timer interrupts.
 *
 *         Global variables are used to determine the module triggering the
 *         interrupt, and therefore, how to handle it.
 * @param  none
 * @return none
 ******************************************************************************/

#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
    if (buttonDebounce == 1)
    {
        buttonDebounce = 2;

        SFRIFG1 &= ~WDTIFG;
        SFRIE1 &= ~WDTIE;
        WDTCTL = WDTPW + WDTHOLD;
    }
}

/***************************************************************************//**
 * @brief  Handles Port 2 interrupts - performs button debouncing and registers
 *         button presses.
 * @param  none
 * @return none
 ******************************************************************************/

#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void)
{

    switch (__even_in_range(P2IV, P2IV_P2IFG7))
    {
        case  P2IV_NONE: break;		        // Vector  P2IV_NONE:  No Interrupt pending
        case  P2IV_P2IFG0: break;
        case  P2IV_P2IFG1: break;
        case  P2IV_P2IFG2: break;
        case  P2IV_P2IFG3: break;
        case  P2IV_P2IFG4: break;
        case  P2IV_P2IFG5: break;
        case  P2IV_P2IFG6:
			if (buttonDebounce == 2)
			{
				buttonsPressed &= ~0x02;
				buttonDebounce = 0;
				__bic_SR_register_on_exit(LPM3_bits);
			}
			else if (buttonDebounce == 0)
			{
				buttonsPressed |= 0x02;
				buttonDebounce = 1;
				Buttons_startWDT();
				__bic_SR_register_on_exit(LPM3_bits);
			}

			break;
        case  P2IV_P2IFG7: break;
        default: break;
    }
}

/***************************************************************************//**
 * @}
 ******************************************************************************/
