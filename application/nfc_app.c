/*
 * File Name: nfc.c
 *
 * Description: Functions to handle the Application Layer processing of
 * the NFC/RFID stack.
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

#include "nfc_app.h"

//*****************************************************************************
//
//! \addtogroup nfc_app_api NFC Application Specific API's
//! @{
//!
//! Application specific API's to handle NFC communication between the TRF7970A
//! and the RF430FRL152H and calculate temperature measurements from received
//! sensor data.
//
//*****************************************************************************

//*****************************************************************************
//
//! NFC_runAppReadFRLTag - Application designed to detect
//! ISO15693 tags and read data from them.
//!
//! This function detects ISO15693 tags, determines how many tags
//! (if any) were detected, and then for each of these tags, run
//! the application state machine that attempts to find and read
//! data from an RF430FRL15xH tag.
//!
//! \return None.
//
//*****************************************************************************

void NFC_runAppReadFRLTag(void)
{
	uint8_t ui8TagSelected;
	uint8_t ui8NumberTagsFound;
	uint8_t ui8Status;

	if (NFC_findISO15693Tag() == STATUS_SUCCESS)		// Scans for NFC Type 5 / ISO15693 tags, reads all data blocks indicated.
	{
		ui8NumberTagsFound = ISO15693_getTagCount();	// Determine how many ISO15693 have been detected in the RF field
		LCD_printRSSI();	// Update RSSI on the LCD

		// Loop through each presented ISO15693 in order to find any which are RF430FRL15xH tags.
		// This also allows for multiple RF430FRL152H Sensor Patches to be read in a single function call
		for (ui8TagSelected = 0; ui8TagSelected < ui8NumberTagsFound; ui8TagSelected++)
		{
			ui8Status = NFC_runRF430FRLStateMachine(ui8TagSelected);

			if (ui8Status == STATUS_SUCCESS)
			{
				// Delay within the function to give the user ample time to read the temperature data from the LCD screen.
				// This is used for Out of Box demo purposes only, and is not functionally required at all
				MCU_delayMillisecond(100);
				NFC_updateRSSI();
				MCU_delayMillisecond(100);
				NFC_updateRSSI();
				MCU_delayMillisecond(100);
				NFC_updateRSSI();
			}
		}
	}
	else
	{

#ifdef ENABLE_STANDALONE		// No tag detected
		LED_14443A_OFF;
		LED_14443B_OFF;
		LED_15693_OFF;
#endif
	}
}

//*****************************************************************************
//
//! NFC_runRF430FRLStateMachine - Simple state machine to pull data from an
//! RF430FRL15xH tag.
//!
//! \param ui8TagIndex is the tag index marker for the UID buffer.
//!
//! This function runs through the process of searching for RF430FRL15xH tag,
//! configuring the sensors, and pulling the sensor data from the tag.
//!
//! It also has states that call the API's needed to convert and display the
//! data from a thermistor onto the LCD of the MSP-EXP430FR4133 LaunchPad
//!
//! The ui8TagIndex variable allows for the commands to be issued with
//! addressed commands directed at a single ISO15693 tag.
//!
//! \return ui8Status to indicate whether or not an RF430FRL15xH tag was found
//! and had it's sensor data read.
//
//*****************************************************************************

uint8_t NFC_runRF430FRLStateMachine(uint8_t ui8TagIndex)
{
	tRF430FRLStates eRF430FRLState = RF430FRL_SEARCH_FOR_FRL_TAG;
	uint8_t ui8Status;
	uint16_t ui16ThermReading; 		// Store Thermistor result
	uint16_t ui16RefResReading; 	// Store Reference Resistor result
	uint8_t * pui8TagData;
	float fTemperatureResult;
	bool bRunStateMachine = true;

	while (bRunStateMachine)
	{
		switch (eRF430FRLState)
		{
		case RF430FRL_SEARCH_FOR_FRL_TAG:
			ui8Status = NFC_searchRF430FRL(ui8TagIndex);

			if (ui8Status == STATUS_SUCCESS)
			{
				eRF430FRLState = RF430FRL_WRITE_SENSOR_CONFIG;
			}
			else
			{
				bRunStateMachine = false;
			}
			break;
		case RF430FRL_WRITE_SENSOR_CONFIG:
			ui8Status = NFC_writeRF430FRLSensorConfig();	// Call function which will write the registers to enable Sensor Tag temp sensor measurements

			if (ui8Status == STATUS_SUCCESS)
			{
				eRF430FRLState = RF430FRL_POLL_DATA_READY;
			}
			else
			{
				bRunStateMachine = false;
			}
			break;
		case RF430FRL_POLL_DATA_READY:
			ui8Status = NFC_pollRF430FRLDataReady();

			if (ui8Status == STATUS_SUCCESS)
			{
				eRF430FRLState = RF430FRL_READ_SENSOR_DATA;
			}
			else
			{
				bRunStateMachine = false;
			}
			break;
		case RF430FRL_READ_SENSOR_DATA:
			ui8Status = ISO15693_sendReadSingleBlock(0x22,0x09); // Read Block 9, which is where the sensor data is stored

			if (ui8Status == STATUS_SUCCESS)
			{
				LCD_printRSSI();		// Update RSSI on the LCD
				pui8TagData = TRF79xxA_getTrfBuffer();		// Fetch the TRF7970A Data Buffer pointer

				eRF430FRLState = RF430FRL_CONVERT_SENSOR_DATA;
			}
			else
			{
				bRunStateMachine = false;
			}
			break;
		case RF430FRL_CONVERT_SENSOR_DATA:
			ui16ThermReading = ((pui8TagData[4] << 8) | pui8TagData[3]);  // Store the raw ADC value into a 16 bit integar for temperature conversion
			ui16RefResReading = ((pui8TagData[2] << 8) | pui8TagData[1]);	// Store the raw ADC value into a 16 bit integar for temperature conversion

			fTemperatureResult = NFC_calculateTemperature(ui16ThermReading,ui16RefResReading);	// Acquire temperature result based on the raw ADC values

			eRF430FRLState = RF430FRL_DISPLAY_TEMPERATURE_RESULT;

			break;
		case RF430FRL_DISPLAY_TEMPERATURE_RESULT:
			LCD_displayTemperature(fTemperatureResult);		// Display the resulting temperature value on the LCD

			ui8Status = STATUS_SUCCESS;
			bRunStateMachine = false;

			break;
		}
	}
	return ui8Status;
}

//*****************************************************************************
//
//! NFC_findISO15693Tag - Application to search for ISO15693
//! compliant tags.
//!
//! This function configures TRF79xxA for ISO1593, waits for the
//! guard time to allow VICC to power up, and then searches for
//! ISO15693 tags.
//!
//! \return ui8Status returns either STATUS_SUCCESS or STATUS_FAIL
//! to indicate if an ISO15693 tag was found or not.
//
//*****************************************************************************

uint8_t NFC_findISO15693Tag(void)
{
	uint8_t ui8TagFound = STATUS_FAIL;

	if (TRF79xxA_checkExternalRfField() == true)
	{
		return STATUS_FAIL;
	}

	TRF79xxA_setupInitiator(0x02);		// Configure the TRF797x for ISO15693 @ High Bit Rate, One Subcarrier, 1 out of 4

	// The VCD should wait at least 1 ms after it activated the
	// powering field before sending the first request, to
	// ensure that the VICCs are ready to receive it. (ISO15693-3)

	// Since certain RF430FRL15xH tag designs such as RF430-TMPSNS-EVM are RF field powered
	// a longer guard time is used to allow these IC's to energize before sending out RFID commands.
	MCU_delayMillisecond(20);

	ISO15693_resetTagCount();				// Reset the counter for tags detected
	ISO15693_setSelectedTagIndex(0);		// Reset the Selected Tag Index

	ui8TagFound = ISO15693_sendSingleSlotInventory();							// Send a single slot inventory request to try and detect a single ISO15693 Tag

	// Inventory failed - search with full anticollision routine
	if (ui8TagFound == STATUS_FAIL)
	{
		ISO15693_resetRecursionCount();			// Clear the recursion counter
		MCU_delayMillisecond(5);				// Delay before issuing the anticollision commmand
		ui8TagFound = ISO15693_runAnticollision(0x06, 0x00, 0x00);		// Send 16 Slot Inventory request with no mask length and no AFI
	}

	if (ui8TagFound == STATUS_SUCCESS)
	{
		LCD_printRSSI();

		if (ISO15693_getTagCount() > 1)
		{
#ifdef ENABLE_HOST
			UART_putNewLine();
			UART_sendCString("Multiple ISO15693 Tags Found");
			UART_putNewLine();
			UART_sendCString("# of Tags Detected: ");
			UART_putByteDecimalValue(ISO15693_getTagCount());
			UART_putNewLine();
#endif
		}
	}
	else
	{
#ifdef ENABLE_STANDALONE		// No card detected
		LED_15693_OFF;
#endif
	}

	return STATUS_SUCCESS;
}


//*****************************************************************************
//
//! NFC_searchRF430FRL - Issues ISO15693 Commands which can identify if an
//! ISO15693 tag is an RF430FRL15xH tag.
//!
//! This function first checks the tag UID to determine if the ISO15693 is made
//! by Texas Instruments. If so, then the Get System Information command is
//! issued to get information about the memory size of the tag.
//!
//! The only Texas Instruments ISO15693 transponders that have a memory size of
//! of 0xF2 are from the RF430FRL15xH family.
//!
//! \return ui8Status returns either STATUS_SUCCESS or STATUS_FAIL
//! to indicate if the sequence of commands were successful or not.
//
//*****************************************************************************

uint8_t NFC_searchRF430FRL(uint8_t ui8TagIndex)
{
	uint8_t ui8Status = STATUS_FAIL;
	uint16_t ui16ReadBlocks;
	uint8_t * pui8UIDPtr;

	// Check if the TI Vendor ID is given (0x07)
	pui8UIDPtr = ISO15693_getUid(ui8TagIndex);

	if (pui8UIDPtr[6] == 0x07)
	{
		// Check if the Memory size reported matches the RF430FRL15xH tags (0xF2) via Get System Information command
		ISO15693_setSelectedTagIndex(ui8TagIndex);			// Set index used to pull the UID for any Addressed command to the correct tag
		ui16ReadBlocks = ISO15693_sendGetSystemInfo(0x22); 	// Get Tag Information with Request Flag = 0x22 for Addressed sending

		// Check the Get System Information response
		if (ui16ReadBlocks == 0xF2)
		{
			ui8Status = STATUS_SUCCESS;
		}
		else
		{
			ui8Status = STATUS_FAIL;
		}

		LCD_printRSSI();									// Update Signal Strength indicator bar on LCD screen (Top right corner)
	}
	return ui8Status;
}

//*****************************************************************************
//
//! NFC_pollRF430FRLDataReady - Checks for sensor data to be ready on the
//! RF430FRL15xH.
//!
//! This function reads Block 0 of the RF430FRL15xH repeatedly and checks the
//! result to determine when the ADC conversion is completed and sensor data
//! is available to be read.
//!
//! \return ui8Status returns either STATUS_SUCCESS or STATUS_FAIL
//! to indicate if the sensor data is ready or if an error occurred.
//
//*****************************************************************************

uint8_t NFC_pollRF430FRLDataReady(void)
{
	uint8_t ui8Status;
	uint8_t * pui8TagData;
	bool bRetry = true;

	// Once the tag has been configured, then continuously read Block 0 to wait for tag data to be available.
	// If the tag is removed from the RF field, or a communication error occurs, the Iso15693_ReadSingleBlock function
	// will return a STATUS_FAIL and the function will be exited.
	while ((ISO15693_sendReadSingleBlock(0x22,0x00) == STATUS_SUCCESS) && (bRetry == true))
	{
		pui8TagData = TRF79xxA_getTrfBuffer();
		if ((pui8TagData[2] & 0x03) == 0x02)	// If result of second byte in Block 0 == 0x02, then data is available
		{
			// Data is ready, set status to Success
			ui8Status = STATUS_SUCCESS;
			bRetry = false;
		}
		else if ((pui8TagData[2] & 0x03) == 0x03)	// If result of second byte in Block 0 == 0x03, an error occured
		{
			// Error occurred, set status to Fail
			ui8Status = STATUS_FAIL;
			bRetry = false;
		}
		else
		{
			// Otherwise, Do Nothing, Keep Reading
			bRetry = true;
		}
	}

	return ui8Status;
}

//*****************************************************************************
//
//! NFC_writeRF430FRLSensorConfig - Configures the RF430FRL152H to sample a
//! thermistor.
//!
//! This function is used to write the data blocks of the RF430FRL152H with
//! the necessary configuration parameters to begin sampling of the thermistor
//! and reference resistor for temperature measurements.
//!
//! \return ui8Status returns either STATUS_SUCCESS or STATUS_FAIL
//! to indicate if the sequence of commands were successful or not.
//
//*****************************************************************************

uint8_t NFC_writeRF430FRLSensorConfig(void)
{
	uint8_t ui8LoopCount = 0;
	uint8_t ui8Status = STATUS_FAIL;
	uint8_t pui8BClearBlockContents[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t pui8Block0Contents[8] = {0x01, 0x00, 0x03, 0x03, 0x01, 0x01, 0x00, 0x40};
	uint8_t pui8Block20Contents[8] = {0x19, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	if (ISO15693_sendWriteSingleBlock(0x22,0x01,0x08,pui8BClearBlockContents) == STATUS_SUCCESS)
	{
		if (ISO15693_sendWriteSingleBlock(0x22,0x02,0x08,pui8Block20Contents) == STATUS_SUCCESS)
		{
			for (ui8LoopCount = 0; ui8LoopCount < 5; ui8LoopCount++)
			{
				ui8Status = ISO15693_sendWriteSingleBlock(0x22,ui8LoopCount+3,0x08,pui8BClearBlockContents);
				if (ui8Status == STATUS_FAIL)
				{
					break;
				}
			}
			if (ui8Status == STATUS_SUCCESS)
			{
				ui8Status = ISO15693_sendWriteSingleBlock(0x22,0x00,0x08,pui8Block0Contents);
			}
		}
	}
	return ui8Status;
}

//*****************************************************************************
//
//! NFC_calculateTemperature - Function to calculate a temperature based on
//! thermistor and reference resistor inputs.
//!
//! \param ui16ThermValue is the raw ADC value after reading the
//!	FRL152H Sensor Patch thermistor
//!	\param ui16RefResValue is the raw ADC value after reading the
//!	FRL152H Sensor Patch reference resistor
//!
//! This function calculates the temperature result from the sensors on the
//! RF430FRL152H Sensor Patch by using the MSP430 IQmathLib Fixed Point
//! mathematics library.
//!
//! The IQmathLib offers various grades of decimal precision versus maximum
//! possible value stored. In order to support the temperature conversion while
//! maintaining a high level of precision, the IQ10 setting is used.
//!
//! The process of calculating the temperature from the thermistor value is
//! specific to the thermistor used on the RF430-TMPSNS-EVM board
//! (ERT-J1VS104FA).
//!
//! \return fTemperatureResult is the floating point result of the temperature
//! conversion.
//
//*****************************************************************************

float NFC_calculateTemperature(uint16_t ui16ThermValue, uint16_t ui16RefResValue)
{
	float fTemperatureResult;
	const float Conversion_Factor = 99999.96185;	// This represents a pre-calculated number specifically for the RF430FRL152H Sensor Patch.
													// This stems from the formula below:
													// ((((((ui16ThermValue * 0.9) / 16384.0) / 2.0) / 0.0000024) * 8738.13) / ui16RefResValue)
													// The result is for: (((((0.9) / 16384.0) / 2.0) / 0.0000024) * 8738.13)
													// This is done to simplify the Fixed Point Library math calculations and provide a better IQ Resolution without overflowing

	const float B_Value = 4330.0;					// Thermistor Constant (in degrees K)
	const float K0_Temp = 273.15;					// Conversion factor to change between Celcius and Kelvin
	const float Log_Const = 2.718;					//
	const float Calibration_Const = 0.049293;		// This result is equal to R0_Value * Exp(-B_Value/T0_Value)
													// where R0_Value = the resistance of the reference resistor (100kohm for the FRL Sensor Patch)
													// and T0_Value is the typical average temperature for the tag to experience in degrees Kelvin (K) (Room temp = 25 degrees C for this example)

	float iq10_TempCalculation1 = 0;
	float iq10_TempCalculation2 = 0;
	float iq10_TempCalculation3 = 0;
	float iq10_TempResult = 0;
	float iq10_Conversion_Factor = 0;
	float iq10_B_Value = 0;
	float iq10_K0_Temp = 0;
	float iq10_Log_Const = 0;
	float iq10_Calibration_Const = 0;

	iq10_Conversion_Factor = _IQ10(Conversion_Factor);

	//	Original Formula for reference: tempConv = ((((((ui16ThermValue * 0.9) / 16384.0) / 2.0) / 0.0000024) * 8738.13) / ui16RefResValue);
	iq10_TempResult = _IQ10mpy(_IQ10div(iq10_Conversion_Factor,ui16RefResValue),ui16ThermValue); // Dividing and then multiplying in order to not overflow the IQ10 value

	// Conversions for fixed point library
	iq10_B_Value = _IQ10(B_Value);
	iq10_K0_Temp = _IQ10(K0_Temp);
	iq10_Log_Const = _IQ10(Log_Const);
	iq10_Calibration_Const = _IQ10(Calibration_Const);

	// Series of calculations in order to get the temperature based on FRL Sensor Patch component values
	// Based on: tempConv = (B_Value / (Math.Log10(tempConv / (R0_Value * Math.Exp((-B_Value) / T0_Value))) / Math.Log10(2.718))) - K0_Temp
	// Note: The tempConv used inside of Math.Log10 is the one calculated above for iq10_TempResult
	iq10_TempCalculation2 = _IQ10div(iq10_TempResult,iq10_Calibration_Const);
	iq10_TempCalculation1 = _IQ10log(iq10_TempCalculation2);
	iq10_TempCalculation3 = _IQ10log(iq10_Log_Const);
	iq10_TempCalculation2 = _IQ10div(iq10_TempCalculation1,iq10_TempCalculation3);
	iq10_TempCalculation1 = _IQ10div(iq10_B_Value,iq10_TempCalculation2);
	iq10_TempResult = iq10_TempCalculation1 - iq10_K0_Temp;

	// Temperature result is in degrees Celcius after calculations are finished
	if ((buttonsPressed & 0x02) == 0x00)
	{
		fTemperatureResult = _IQ10toF(iq10_TempResult); // Convert the result back to Floating Point
	}
	else if ((buttonsPressed & 0x02) == 0x02)
	{
		// If Button S2 (P2.6) has been toggled, display the temperature in Fahrenheit
		// Fahrenheit = ((Celcius * 9) / 5 ) + 32;

		iq10_TempCalculation1 = _IQ10mpy(iq10_TempResult,(_IQ10(9)));
		iq10_TempResult = _IQ10div(iq10_TempCalculation1,(_IQ10(5)));

		fTemperatureResult = _IQ10toF(iq10_TempResult); 		// Convert the result back to Floating Point

		fTemperatureResult = fTemperatureResult + 32;
	}

	return fTemperatureResult;
}

//*****************************************************************************
//
//! NFC_updateRSSI - Updates the RSSI reading.
//!
//! This function sends an ISO15693 Read Single Block command to get a new
//! RSSI reading in and then updates the LCD display of the MSP-EXP430FR4133
//! LaunchPad module.
//!
//! This is an application specific function designed for ISO15693, but can
//! be modified to support other tag technologies by changing the RF command
//! issued to a command which will trigger a reply from the desired tag
//! technology.
//!
//! \return None.
//
//*****************************************************************************

void NFC_updateRSSI(void)
{
	ISO15693_sendReadSingleBlock(0x22,0x00);
	LCD_printRSSI();	// Update RSSI value on the LCD screen
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
