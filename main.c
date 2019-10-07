/*
 * File Name: main.c
 *
 * Description: The TRF797x is an integrated analog front end and
 * data framing system for a 13.56 MHz RFID reader system.
 * Built-in programming options make it suitable for a wide range
 * of applications both in proximity and vicinity RFID systems.
 * The reader is configured by selecting the desired protocol in
 * the control registers. Direct access to all control registers
 * allows fine tuning of various reader parameters as needed.
 *
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
*
* DESCRIPTION:
* This Example detects ISO15693 NFC/RFID tags and then checks if an
* RF430FRL15xH transponder has been presented.
*
* If an RF430FRL15xH transponder has been detected, then it is configured
* for sensor measurements using the Write Single Block NFC command. The
* example firmware sends configurations specifically for thermistors, but
* these configurations can be modified to support other custom
* applications
*
* Once the raw sensor data is received by the TRF7970A, then the
* MSP430FR4133 uses the IQmathLib in order to calculate the temperature
* result and display it on the on-board LCD of the MSP-EXP430FR4133
* LaunchPad.
*
* This example also can optionally output that data to UART along with
* information such as the tag UID.
*
* The TRF7970A is an integrated analog front end and data framing system
* for a 13.56 MHz RFID reader system. Built-in programming options make
* it suitable for a wide range of applications both in proximity and
* vicinity RFID systems. The reader is configured by selecting the
* desired protocol in the control registers. Direct access to all
* control registers allows fine tuning of various reader parameters
* as needed.
*
* The TRF7970A is interfaced to a MSP430FR4133 through a SPI (serial)
* interface using a hardware USCI. The MCU is the master device and
* initiates all communication with the reader.
*
* The anti-collision procedures (as described in the ISO15693 standard)
* are implemented in the MCU firmware to help the reader detect and
* communicate with one VICC among several VICCs.
*
* AUTHORS:   	Ralph Jacobi
*
* BUILT WITH:
* Code Composer Studio Core Edition Version: 6.0.1.00040
* (c) Copyright Texas Instruments, 2014. All rights reserved.
*****************************************************************/

//===============================================================
// Program with hardware USART and SPI communication            ;
// interface with TRF7970A reader chip.                         ;
//                                                              ;
// PORT5.1 - SPI DATACLK                                        ;
// PORT5.2 - SPI MISO                                           ;
// PORT5.3 - SPI MOSI                                           ;
//                                                              ;
// PORT1.6 - IRQ (INTERUPT REQUEST from TRF7970A)               ;
// PORT8.2 - SLAVE SELECT                                       ;
// PORT8.3 - TRF7970A ENABLE                                    ;
// PORT1.5 - ISO14443B LED                                      ;
// PORT1.4 - ISO14443A LED                                      ;
// PORT1.3 - ISO15693  LED                                      ;
//===============================================================

//======================================
// ********** HEADER FILES ********** //
//======================================

#include "nfc_app.h"
#include "lcd_app.h"

//===============================================================

void main(void)
{
	// Stop the Watchdog timer
	WDT_A_hold(__MSP430_BASEADDRESS_WDT_A__);
	
	// Select DCO to be 8 MHz
	MCU_initClock(8000);

	// Set the SPI SS high
	SLAVE_SELECT_PORT_SET;
	SLAVE_SELECT_HIGH;

	// Switch TRF Enable Pin from low to high
	TRF_DISABLE;
	TRF_ENABLE_SET;
	MCU_delayMillisecond(10);
	TRF_ENABLE;

	// Wait until TRF system clock started
	MCU_delayMillisecond(10);

	// Initialize SPI settings for communication with TRF
	TRF79xxA_initialSettings();

	// Initialize LED on FR4133
	P4DIR |= BIT0;
	P4OUT &= ~0x01;

#ifdef ENABLE_HOST
	UART_setup();
#endif

	// Initialize Push Button S1 on FR4133
    GPIO_interruptEdgeSelect(GPIO_PORT_P1, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);

	// Initialize Push Button S2 on FR4133
    GPIO_interruptEdgeSelect(GPIO_PORT_P2, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN6);
    GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN6);
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);

	// Enable IRQ Interrupts
	IRQ_ON;

	// Enable global interrupts
	__bis_SR_register(GIE);

	// Initial ISO15693 Layer
	ISO15693_init();

	// Initialize the LCD screen
	Init_LCD();

	// Initialize the Heartbeat LCD Symbol
	MCU_setHeartbeat();

	// Initialize RSSI Display Brackets
	LCDMEM[12] = 0x00 | (LCDMEM[12] & 0x04);
	LCDMEM[13] = 0x01;

#ifdef ENABLE_HOST
	UART_putIntroReaderMsg();		// Send a simple intro message over UART which can be read from a UART terminal
#endif

	while(1)
	{
		LCD_displayTINFCMsg();		// Display basic intro message on the LCD Screen ("TI NFC")

		NFC_runAppReadFRLTag();		// NFC Application to search for RF430FRL15xH tags and read thermistor data

		MCU_delayMillisecond(500);	// Delay to allow user to read temperature on LCD screen
	}
}

