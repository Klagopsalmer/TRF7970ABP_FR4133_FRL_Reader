/*
 * File Name: lcd_app.c
 *
 * Description: Functions to handle LCD Application
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

#include "lcd_app.h"

//*****************************************************************************
//
//! \addtogroup lcd_app_api LCD Application Specific API's
//! @{
//!
//! Application specific API's to display messages to the LCD of a TI
//! MSP-EXP430FR4133 LaunchPad Module.
//!
//
//*****************************************************************************

//*****************************************************************************
//
//! LCD_displayTINFCMsg - Display "TI NFC" message to LCD
//!
//! Function sends commands to display a simple message ("TI NFC") on the LCD.
//!
//! \return None
//
//*****************************************************************************

void LCD_displayTINFCMsg(void)
{
	showChar('T',pos1);
	showChar('I',pos2);
	showChar(' ',pos3);
	showChar('N',pos4);
	showChar('F',pos5);
	showChar('C',pos6);

	P4OUT &= ~0x01;
#ifndef ENABLE_HOST
	P1OUT &= ~0x01;
#endif

	MCU_delayMillisecond(200);
}

//*****************************************************************************
//
//! LCD_printRSSI - Function to read RSSI Level and display it on an LCD.
//!
//! Function reads the RSSI level from the TRF7970A, masks the relevant bits,
//! and then display a number of bars to indicate the signal strength on the
//! LCD of the MSP-EXP430FR4133 LaunchPad.
//!
//! \return None
//
//*****************************************************************************

void LCD_printRSSI(void)
{
	uint8_t ui8RSSILevel;

	ui8RSSILevel = TRF79xxA_readRegister(TRF79XXA_RSSI_LEVELS);

	ui8RSSILevel = ui8RSSILevel & 0x07;

	if (ui8RSSILevel < 0x02)
	{
	    LCDMEM[12] = 0x00 | (LCDMEM[12] & 0x04);
		LCDMEM[13] = 0x01;
	}
	else
	{
    	LCDMEM[pos3+1] |= 0x04;			// Place Wireless Connectivity symbol on LCD

    	// Light up the bars on the top right of the LCD based on the strength of the RSSI signal received.
		if (ui8RSSILevel == 0x02)
		{
			LCDMEM[12] = 0x00 | (LCDMEM[12] & 0x04);
			LCDMEM[13] = 0x03;
		}
		else if (ui8RSSILevel == 0x03)
		{
			LCDMEM[12] = 0x20 | (LCDMEM[12] & 0x04);
			LCDMEM[13] = 0x03;
		}
		else if (ui8RSSILevel == 0x04)
		{
			LCDMEM[12] = 0x20 | (LCDMEM[12] & 0x04);
			LCDMEM[13] = 0x07;
		}
		else if (ui8RSSILevel == 0x05)
		{
			LCDMEM[12] = 0x60 | (LCDMEM[12] & 0x04);
			LCDMEM[13] = 0x07;
		}
		else if (ui8RSSILevel == 0x06)
		{
			LCDMEM[12] = 0x60 | (LCDMEM[12] & 0x04);
			LCDMEM[13] = 0x0F;
		}
		else if (ui8RSSILevel == 0x07)
		{
			LCDMEM[12] = 0xE0 | (LCDMEM[12] & 0x04);
			LCDMEM[13] = 0x0F;
		}
	}
}

//*****************************************************************************
//
//! LCD_displayTemperature - Function to display a temperature value to an LCD.
//!
//! \param fTemperature is a floating point temperature to display.
//!
//! Function displays the provided floating point temperature from an
//! RF430FRL152H Sensor Patch to the LCD of the MSP-EXP430FR4133 LaunchPad.
//!
//! \return None
//
//*****************************************************************************

void LCD_displayTemperature(float fTemperature)
{
	uint8_t pui8TemperatureResult[4];

	P4OUT |= 0x01;							// Light Green LED when Temperature is displayed

	LCD_convertFloatToAscii(fTemperature, &pui8TemperatureResult[0]);

	// Display values from Temperature Result buffer on LCD screen
    showChar(pui8TemperatureResult[0], pos1);
    showChar(pui8TemperatureResult[1], pos2);
    showChar(pui8TemperatureResult[2], pos3);
    showChar(pui8TemperatureResult[3], pos4);

    // Check if display should be for Fahrenheit or Celcius and append correct symbol
	if ((buttonsPressed & 0x02) == 0x00)
	{
		showChar(' ', pos5);
		showChar('C', pos6);
	}
	else if ((buttonsPressed & 0x02) == 0x02)
	{
		showChar(' ', pos5);
		showChar('F', pos6);
	}

	// Check if Temperature is Negative
	if (fTemperature < 0)
	{
		LCDMEM[pos1+1] |= 0x04;				// Display negative symbol
	}

    LCDMEM[pos3+1] |= 0x01;					// Display decimal point
    LCDMEM[pos5+1] |= 0x04;					// Display degree symbol
	LCDMEM[pos3+1] |= 0x04;					// Display the Wireless Connectivity symbol
	LCD_printRSSI();
}


//*****************************************************************************
//
//! LCD_convertFloatToAscii - Function to convert a floating point input value
//! into ASCII characters which are then stored in an unsigned 8 bit integar
//! array.
//!
//! \param fFloatInput is the floating point value to be converted.
//! \param pui8OutputBuffer is a user provided buffer to store the converted
//! value into.
//!
//! Function converts a provided floating point input to ASCII characters and
//! stores each converted digit into a user provided buffer.
//!
//! Function is limited to maximum 4 digit conversion with one decimal point
//! precision as it is designed to handle temperatures in the range of
//! -40 to 125 degree Celcius.
//!
//! \return None
//
//*****************************************************************************

void LCD_convertFloatToAscii(float fFloatInput, uint8_t * pui8OutputBuffer)
{
	float fTempFloatVar;
	uint8_t ui8TempIntVar;

	// Check if Temperature is Negative
	if (fFloatInput < 0)
	{
		// If so, turn it into a Positive value for the conversion
		fFloatInput = fabs(fFloatInput);
	}

	if (fFloatInput < 100)
	{
		pui8OutputBuffer[0] = ' ';

		fTempFloatVar = fFloatInput;

		ui8TempIntVar = fTempFloatVar/10;					// Divide by 10 and truncate off the decimal points to acquire 10's digit of the value
		pui8OutputBuffer[1] = 0x30+ui8TempIntVar;	// Turn the result into an Ascii character

		fTempFloatVar = fFloatInput-(ui8TempIntVar*10);	// Subtract 10's digit from original value to acquire 1's digit of the value
		ui8TempIntVar = fTempFloatVar;						// Truncate off the decimal points
		pui8OutputBuffer[2] = 0x30+ui8TempIntVar;	// Turn the result into an Ascii character

		fTempFloatVar = (fTempFloatVar-ui8TempIntVar)*10;	// Subtract the 1's digit from the remaining value to have only decimals left
		// then multiply by 10 to shift the 1st decimal into the 1's digit
		ui8TempIntVar = fTempFloatVar;						// Truncate off the decimal points
		pui8OutputBuffer[3] = 0x30+ui8TempIntVar;	// Turn the result into an Ascii character
	}
	else
	{
		fTempFloatVar = fFloatInput;

		ui8TempIntVar = fTempFloatVar/100;					// Divide by 100 and truncate off the decimal points to acquire 100's digit of the value
		pui8OutputBuffer[0] = 0x30+ui8TempIntVar;	// Turn the result into an Ascii character

		fTempFloatVar = fFloatInput-(ui8TempIntVar*100);	// Subtract 100's digit from original value to acquire 10's digit of the value
		ui8TempIntVar = fTempFloatVar/10;					// Divide by 10 and truncate off the decimal points to acquire 10's digit of the value
		pui8OutputBuffer[1] = 0x30+ui8TempIntVar;	// Turn the result into an Ascii character

		fTempFloatVar = fTempFloatVar-(ui8TempIntVar*10);	// Subtract 10's digit from original value to acquire 1's digit of the value
		ui8TempIntVar = fTempFloatVar;						// Truncate off the decimal points
		pui8OutputBuffer[2] = 0x30+ui8TempIntVar;	// Turn the result into an Ascii character

		fTempFloatVar = (fTempFloatVar-ui8TempIntVar)*10;	// Subtract the 1's digit from the remaining value to have only decimals left
		// then multiply by 10 to shift the 1st decimal into the 1's digit
		ui8TempIntVar = fTempFloatVar;						// Truncate off the decimal points
		pui8OutputBuffer[3] = 0x30+ui8TempIntVar;	// Turn the result into an Ascii character
	}
}

//*****************************************************************************
//
//! LCD_displayUID - Function to read RSSI Level and display it on an LCD.
//!
//! \param ui8SelectedTag is a pointer to the array which contains the ISO15693
//! tag UID to be displayed.
//!
//! Function displays the UID to the LCD of the FR4133 LaunchPad.
//!
//! \return None
//
//*****************************************************************************

void LCD_displayUID(uint8_t * pui8ISO15693UID)
{
    uint8_t ui8LoopCount1 = 0;
    uint8_t ui8LoopCount2 = 0;
    int8_t i8DisplayOffset = 5;
    int8_t pi8DisplayBuffer[6] = "      ";	// Initialize display buffer with all spaces
    int8_t pi8UIDBuffer[20];					// Buffer required to handle displaying the whole UID including "UID "
    uint8_t ui8TempVar;

    MCU_delayMillisecond(250);			// Delay to allow user to read the screen
    // Preclear UID display
    showChar(pi8DisplayBuffer[0], pos1);
    showChar(pi8DisplayBuffer[1], pos2);
    showChar(pi8DisplayBuffer[2], pos3);
    showChar(pi8DisplayBuffer[3], pos4);
    showChar(pi8DisplayBuffer[4], pos5);
    showChar(pi8DisplayBuffer[5], pos6);

    LCD_printRSSI();					// Update Signal Strength indicator bar on LCD screen (Top right corner)
    MCU_delayMillisecond(250);			// Delay to allow screen to remain clear for a few moments

    // Initialize the first 4 characters of the buffer as "UID "
    pi8UIDBuffer[0] = 'U';
    pi8UIDBuffer[1] = 'I';
    pi8UIDBuffer[2] = 'D';
    pi8UIDBuffer[3] = ' ';

   	// Fill the display buffer with the UID of a tag
    for (ui8LoopCount1 = 0; ui8LoopCount1 < 16; ui8LoopCount1++)
    {
    	// Break the byte into nibbles
    	ui8TempVar = pui8ISO15693UID[7-(ui8LoopCount1/2)] >> 4;

    	// Handle placing Hexadecimal characters A-F in the buffer and converting 0-9 to Ascii
    	switch (ui8TempVar)
    	{
    	case 0x0A:
    		pi8UIDBuffer[ui8LoopCount1+4] = 'A';
    		break;
    	case 0x0B:
    		pi8UIDBuffer[ui8LoopCount1+4] = 'B';
    		break;
    	case 0x0C:
    		pi8UIDBuffer[ui8LoopCount1+4] = 'C';
    		break;
    	case 0x0D:
    		pi8UIDBuffer[ui8LoopCount1+4] = 'D';
    		break;
    	case 0x0E:
    		pi8UIDBuffer[ui8LoopCount1+4] = 'E';
    		break;
    	case 0x0F:
    		pi8UIDBuffer[ui8LoopCount1+4] = 'F';
    		break;
    	default:
        	pi8UIDBuffer[ui8LoopCount1+4] = ui8TempVar + 0x30;
    		break;
    	}

    	ui8LoopCount1++;

    	// Break the byte into nibbles
    	ui8TempVar = pui8ISO15693UID[7-(ui8LoopCount1/2)] & 0x0F;

    	// Handle placing Hexadecimal characters A-F in the buffer and converting 0-9 to Ascii
    	switch (ui8TempVar)
    	{
    	case 0x0A:
    		pi8UIDBuffer[ui8LoopCount1+4] = 'A';
    		break;
    	case 0x0B:
    		pi8UIDBuffer[ui8LoopCount1+4] = 'B';
    		break;
    	case 0x0C:
    		pi8UIDBuffer[ui8LoopCount1+4] = 'C';
    		break;
    	case 0x0D:
    		pi8UIDBuffer[ui8LoopCount1+4] = 'D';
    		break;
    	case 0x0E:
    		pi8UIDBuffer[ui8LoopCount1+4] = 'E';
    		break;
    	case 0x0F:
    		pi8UIDBuffer[ui8LoopCount1+4] = 'F';
    		break;
    	default:
        	pi8UIDBuffer[ui8LoopCount1+4] = ui8TempVar + 0x30;
    		break;
    	}
    }

    i8DisplayOffset = 5; 				// Initialize display offset

    // Loop to display full UID
    for (ui8LoopCount1 = 0; ui8LoopCount1 < 27; ui8LoopCount1++)	// 4 ('UID ') + 16 (Digits for UID) + 6 (Display spaces on LCD) + 1 (End Loop) = 27
    {
    	// Start by setting current display buffer values to ' '
    	// This is needed for the back half of the loop in order to ensure that digits are cleared while not needing to use an extra 6 bytes of RAM memory
        for (ui8LoopCount2 = 0; ui8LoopCount2 < 6; ui8LoopCount2++)
        {
            pi8DisplayBuffer[ui8LoopCount2] = ' ';
        }

        // Then cycle through the pi8UIDBuffer to get the next 6 characters stored into pi8DisplayBuffer
        for (ui8LoopCount2 = 0; ui8LoopCount2 < 20; ui8LoopCount2++)
        {
            if (((i8DisplayOffset+ui8LoopCount2) >= 0) && ((i8DisplayOffset+ui8LoopCount2) < 6))
                pi8DisplayBuffer[i8DisplayOffset+ui8LoopCount2] = pi8UIDBuffer[ui8LoopCount2];
        }
        i8DisplayOffset--;		// Adjust the display offset

        // Display the 6 characters for this cycle to the LCD screen
        showChar(pi8DisplayBuffer[0], pos1);
        showChar(pi8DisplayBuffer[1], pos2);
        showChar(pi8DisplayBuffer[2], pos3);
        showChar(pi8DisplayBuffer[3], pos4);
        showChar(pi8DisplayBuffer[4], pos5);
        showChar(pi8DisplayBuffer[5], pos6);

        MCU_delayMillisecond(150);		// Delay to give user time to read the digits, can be increased or decreased if needed.
    }
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
