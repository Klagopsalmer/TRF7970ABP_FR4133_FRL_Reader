/*
 * File Name: nfc.h
 *
 * Description: Headers and Defines for the Application Layer processing
 * of the NFC/RFID stack.
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

#ifndef NFC_H_
#define NFC_H_

//================================================================

#include "iso15693.h"
#include "lcd_app.h"
#include "IQmathLib.h"

//================================================================

typedef enum
{
    RF430FRL_SEARCH_FOR_FRL_TAG = 0x00,

    RF430FRL_WRITE_SENSOR_CONFIG,

    RF430FRL_POLL_DATA_READY,

    RF430FRL_READ_SENSOR_DATA,

    RF430FRL_CONVERT_SENSOR_DATA,

    RF430FRL_DISPLAY_TEMPERATURE_RESULT,

    RF430FRL_STATE_MACHINE_ERROR

}tRF430FRLStates;

//================================================================

void NFC_runAppReadFRLTag(void);

uint8_t NFC_runRF430FRLStateMachine(uint8_t ui8TagIndex);

uint8_t NFC_findISO15693Tag(void);
uint8_t NFC_searchRF430FRL(uint8_t ui8TagIndex);

uint8_t NFC_pollRF430FRLDataReady(void);
uint8_t NFC_writeRF430FRLSensorConfig(void);
float NFC_calculateTemperature(uint16_t ui16ThermValue, uint16_t ui18RefResValue);

void NFC_updateRSSI(void);

#endif /* NFC_H_ */
