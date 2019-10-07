/*
 * File Name: trf797x.h
 *
 * Description: Headers and Defines for TRF797x Driver Functions
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

#ifndef _TRF79xxa_H_
#define _TRF79xxa_H_

//===============================================================
//==== Revision History =========================================
//
// REV.   DATE        		WHO    			DETAIL
// 1.0	  Sept 1, 2015		RJ				First working version of FR4133 FRL Reader Project
// 1.1	  Sept 29, 2015		RJ				Updated with latest ISO15693/TRF7970A files from sloc297 codebase
// 1.2	  Jan 15, 2016		RJ				Added ISO15693 and TRF7970A driver improvements, added license headers
// 1.3	  Feb 23, 2016		RJ				Improved tag detection process, implemented anticollision improvements
// 1.4	  Aug 1, 2016		RJ				Added TRF7970A Driver and SPI driver bug fixes, added initialization functions
// 1.5	  Sept 14, 2016		RJ				Function renaming, added Doxygen comments
// 2.00	  Oct 5, 2016		RJ				Ported updated TRF79xxA driver file, designed a proper application layer, added Doxygen comments
// 2.01	  Oct 6, 2016		RJ				Updated function names, Finalized Doxygen comments
//
//===============================================================

//===============================================================

#include "spi.h"
#include "uart.h"
#include "hal_Buttons.h"

//===============================================================

#define ENABLE_STANDALONE			// Disables all LED features to allow for easier porting

//
// This defines the version of the TRF79xxA chip being used
// Current Support:
//  - TRF7970A ==> 70
//  - TRF7960A ==> 60
//

#define	TRF79xxA_VERSION			70

//
// This defines the largest FIFO size of the TRF79xxA chip being used
// TRF7970A and TRF7964A - 127 bytes
// TRF7960A, TRF7961A, TRF7962A, and TRF7963A - 12 bytes
//
#if (TRF79xxA_VERSION == 70)
#define TRF79xxA_MAX_FIFO_SIZE 			127
#elif (TRF79xxA_VERSION == 60)
#define TRF79xxA_MAX_FIFO_SIZE 			12
#endif

//
// This defines the size of the FIFO buffer generated. This size comes out of RAM memory.
// For example, setting this to 100 takes up 100 bytes of RAM.
// Recommendation is to reduce this if the full buffer would not be used in certain applications.
//
#define NFC_FIFO_SIZE 					120

//
// Power Settings for TRF79xxA.
// Each definition is based of the setting which would required in the Chip Status Control Register
// These settings do not differ across the TRF79xxA family.
//
typedef enum
{
	TRF79xxA_3V_FULL_POWER = 0x00,			// Full Power, 3V
	TRF79xxA_5V_FULL_POWER = 0x01,			// Full Power, 5V
	TRF79xxA_3V_HALF_POWER = 0x10,			// Half Power, 3V
	TRF79xxA_5V_HALF_POWER = 0x11,			// Half Power, 5V
}tTrfPowerOptions;

//
// Structure for various TRF79xxA Settings
//
typedef struct
{
	uint8_t ui8IsoControl;
	bool bRfFieldOn;					// RF Field Status
	tTrfPowerOptions eTrfPowerSetting;
}tTrfSettings;

//
// TRF79xxA Status
//
typedef enum
{
	TRF_IDLE,
	TX_COMPLETE,
	RX_COMPLETE,
	TX_ERROR,
	RX_WAIT,
	RX_WAIT_EXTENSION,
	TX_WAIT,
	PROTOCOL_ERROR,
	COLLISION_ERROR,
	NO_RESPONSE_RECEIVED,
	NO_RESPONSE_RECEIVED_15693
}tTrfStatus;

//==== TRF797x definitions ======================================

//---- Direct commands ------------------------------------------

#define TRF79XXA_IDLE_CMD                    			0x00
#define TRF79XXA_SOFT_INIT_CMD               			0x03
#if (TRF79xxA_VERSION == 70)
#define TRF79XXA_INITIAL_RF_COLLISION_AVOID_CMD			0x04
#define TRF79XXA_PERFORM_RES_RF_COLLISION_AVOID_CMD		0x05
#define TRF79XXA_PERFORM_RES_RF_COLLISION_AVOID_N0_CMD	0x06
#endif
#define TRF79XXA_RESET_FIFO_CMD              			0x0F
#define TRF79XXA_TRANSMIT_NO_CRC_CMD         			0x10
#define TRF79XXA_TRANSMIT_CRC_CMD            			0x11
#define TRF79XXA_DELAY_TRANSMIT_NO_CRC_CMD   			0x12
#define TRF79XXA_DELAY_TRANSMIT_CRC_CMD      			0x13
#define TRF79XXA_TRANSMIT_NEXT_SLOT_CMD      			0x14
#if (TRF79xxA_VERSION == 70)
#define TRF79XXA_CLOSE_SLOT_SEQUENCE_CMD     			0x15
#endif
#define TRF79XXA_STOP_DECODERS_CMD           			0x16
#define TRF79XXA_RUN_DECODERS_CMD            			0x17
#define TRF79XXA_TEST_INTERNAL_RF_CMD        			0x18
#define TRF79XXA_TEST_EXTERNAL_RF_CMD        			0x19

//---- Registers ------------------------------------------------

#define TRF79XXA_CHIP_STATUS_CONTROL			0x00
#define TRF79XXA_ISO_CONTROL					0x01
#define TRF79XXA_ISO14443B_TX_OPTIONS			0x02
#define TRF79XXA_ISO14443A_BITRATE_OPTIONS		0x03
#define TRF79XXA_TX_TIMER_EPC_HIGH				0x04
#define TRF79XXA_TX_TIMER_EPC_LOW				0x05
#define TRF79XXA_TX_PULSE_LENGTH_CONTROL		0x06
#define TRF79XXA_RX_NO_RESPONSE_WAIT_TIME		0x07
#define TRF79XXA_RX_WAIT_TIME					0x08
#define TRF79XXA_MODULATOR_CONTROL				0x09
#define TRF79XXA_RX_SPECIAL_SETTINGS			0x0A
#define TRF79XXA_REGULATOR_CONTROL				0x0B
#define TRF79XXA_IRQ_STATUS						0x0C
#define TRF79XXA_IRQ_MASK						0x0D
#define	TRF79XXA_COLLISION_POSITION				0x0E
#define TRF79XXA_RSSI_LEVELS					0x0F
#define TRF79XXA_SPECIAL_FUNCTION_1				0x10	// Register not documented on TRF7960A Datasheet, See sloa155 Section 7.8
#if (TRF79xxA_VERSION == 70)
#define TRF79XXA_SPECIAL_FUNCTION_2				0x11
#define TRF79XXA_FIFO_IRQ_LEVELS				0x14
#define TRF79XXA_NFC_LOW_DETECTION_LEVEL		0x16
#define TRF79XXA_NFC_ID_REG                 	0x17
#define TRF79XXA_NFC_TARGET_LEVEL				0x18
#define TRF79XXA_NFC_TARGET_PROTOCOL			0x19
#endif
#define TRF79XXA_TEST_SETTINGS_1				0x1A
#define TRF79XXA_TEST_SETTINGS_2				0x1B
#define TRF79XXA_FIFO_STATUS					0x1C
#define TRF79XXA_TX_LENGTH_BYTE_1				0x1D
#define TRF79XXA_TX_LENGTH_BYTE_2				0x1E
#define TRF79XXA_FIFO							0x1F

//---- IRQ STATUS -----------------------------------------------

#define TRF79XXA_IRQ_STATUS_IDLE					0x00
#define TRF79XXA_IRQ_STATUS_NO_RESPONSE				0x01
#define TRF79XXA_IRQ_STATUS_COLLISION_ERROR			0x02
#define TRF79XXA_IRQ_STATUS_FRAMING_ERROR 			0x04
#define TRF79XXA_IRQ_STATUS_PARITY_ERROR 			0x08
#define TRF79XXA_IRQ_STATUS_CRC_ERROR	 			0x10
#define TRF79XXA_IRQ_STATUS_FIFO_HIGH_OR_LOW 		0x20
#define TRF79XXA_IRQ_STATUS_RX_COMPLETE 			0x40
#define TRF79XXA_IRQ_STATUS_TX_COMPLETE 			0x80

//==== End of TRF797x definitions ===============================

//===============================================================

extern void TRF79xxA_disableSlotCounter(void);
extern void TRF79xxA_enableSlotCounter(void);
extern void TRF79xxA_initialSettings(void);
extern void TRF79xxA_writeRaw(uint8_t * pui8Payload, uint8_t ui8Length);
extern void TRF79xxA_readContinuous(uint8_t * pui8Payload, uint8_t ui8Length);
extern uint8_t TRF79xxA_readRegister(uint8_t ui8TrfRegister);
extern void TRF79xxA_reset(void);
extern void TRF79xxA_resetFIFO(void);
extern void TRF79xxA_resetIrqStatus(void);
extern void TRF79xxA_sendDirectCommand(uint8_t ui8Value);
extern void TRF79xxA_setupInitiator(uint8_t ui8IsoControl);
extern void TRF79xxA_turnRfOff(void);
extern void TRF79xxA_turnRfOn(void);
extern void TRF79xxA_writeContinuous(uint8_t * pui8Payload, uint8_t ui8Length);
extern void TRF79xxA_writeRegister(uint8_t ui8TrfRegister, uint8_t ui8Value);

extern void TRF79xxA_waitIRQ(uint8_t ui8TxTimeout, uint8_t ui8RxTimeout);
extern void TRF79xxA_waitIRQ_txOnly(uint8_t ui8TxTimeout);
extern void TRF79xxA_waitIRQ_rxOnly(uint8_t ui8RxTimeout);

extern tTrfStatus TRF79xxA_getTrfStatus(void);
extern uint8_t TRF79xxA_getCollisionPosition(void);
extern void TRF79xxA_setCollisionPosition(uint8_t ui8ColPos);

extern void TRF79xxA_setTrfStatus(tTrfStatus sTrfStatus);
extern void TRF79xxA_setTrfPowerSetting(tTrfPowerOptions sTrfPowerSetting);
extern uint8_t TRF79xxA_getRxBytesReceived(void);
extern uint8_t * TRF79xxA_getTrfBuffer(void);
extern uint8_t TRF79xxA_getIsoControlValue(void);
extern bool TRF79xxA_checkExternalRfField(void);

//===============================================================

#endif
