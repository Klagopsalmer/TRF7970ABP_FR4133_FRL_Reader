/*
 * File Name: mcu.h
 *
 * Description: Header file for all functions for mcu.h
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

#ifndef MCU_H_
#define MCU_H_

//===============================================================

#include "MSP430.h" 		// Processor specific header
#include "types.h"
#include "cs.h"

//=====MCU constants=============================================

#define TRF_ENABLE_SET	P8DIR |= BIT3		// P8.3 is switched in output direction
#define	TRF_ENABLE		P8OUT |= BIT3		// EN pin on the TRF7970A
#define TRF_DISABLE		P8OUT &= ~BIT3

// IRQ on P1.6
#define IRQ_PIN_SET		P1DIR &= ~BIT6;
#define IRQ_PIN			BIT6
#define IRQ_PORT		P1IN
#define IRQ_ON			P1IE |= IRQ_PIN
#define IRQ_OFF			P1IE &= ~IRQ_PIN
#define IRQ_EDGE_SET	P1IES &= ~IRQ_PIN		// Rising edge interrupt
#define IRQ_CLR			P1IFG = 0x00
#define IRQ_REQ_REG		P1IFG

#define LED_PORT_SET	P1DIR |= 0x38;
#define LED_ALL_OFF		P1OUT &= ~0x38;

#define LED_14443B_ON	P1OUT |= BIT5;
#define LED_14443B_OFF	P1OUT &= ~BIT5;
#define LED_14443A_ON	P1OUT |= BIT4;
#define LED_14443A_OFF	P1OUT &= ~BIT4;
#define LED_15693_ON	P1OUT |= BIT3;
#define LED_15693_OFF	P1OUT &= ~BIT3;

#define SLAVE_SELECT_PORT_SET	P8DIR |= BIT2;
#define SLAVE_SELECT_HIGH		P8OUT |= BIT2;
#define SLAVE_SELECT_LOW		P8OUT &= ~BIT2;

//-----Counter-timer constants-----------------------------------
#define STOP_COUNTER	TA0CTL &= ~(MC0 + MC1)	//stops the counter
#define RESET_COUNTER   TA0CTL |= TACLR	    	//Resets and stops counter.

//===============================================================

#define DELAY_1ms		8000	// Used for McuDelayMillisecond

//===============================================================

void MCU_setCounter(uint16_t ui16mSecTimeout);
void MCU_delayMillisecond(uint32_t n_ms);
void MCU_initClock(uint32_t ui32Freq);
void MCU_setHeartbeat(void);

//===============================================================

#endif
