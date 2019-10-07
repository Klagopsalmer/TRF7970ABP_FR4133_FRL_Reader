

# TRF7970ABP_FR4133_FRL_Reader
## Description
Texas Instrument [SLOA233](http://www.ti.com/lit/an/sloa233/sloa233.pdf) application. Designed to read temperature data from the [RF430-TMPSNS-EVM](http://www.ti.com/tool/RF430-TMPSNS-EVM) using the [MSP-EXP430FR4133](http://www.ti.com/tool/MSP-EXP430FR4133) launchpad and the [DLP-7970ABP](http://www.ti.com/tool/DLP-7970ABP) booster pack.
## Original source code
Source code can be downloaded [here](http://www.ti.com/lit/an/sloa233/sloa233.zip). A few changes have been made to make it work.

## Software changes
[Fix compilation error caused by include casing.](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/commit/b752b348c85d3f6210ea718dc047b29057960b5a)   
[Fix watchdog disable not working.](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/commit/c6101a07b829fa8c737466e8e43a63f3e39bf3a0)   
[Enable UART host logging](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/commit/ed62871c9fffa22a1655af4e9c1f3e885a53b0ff)  
[Allow sending only RSSI by UART](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/commit/7b4f4850043c7cdd0ad0fc7577c4a263f5d3766c)
## Hardware setup
Ensure that the IRQ jumper on the DLP-7970ABP booster pack is set to position 1 (default) as described in [DLP-7970ABP Hardware Update Overview](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/blob/master/doc/sloa226.pdf)

## RSSI reading over UART
The software has been modified to allow sending the RSSI to the host via UART. The [datasheet](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/blob/master/doc/trf7970a.p) of the transceiver says (page 69) the following:

> RSSI measurement block is measuring the demodulated envelope signal (except in case of direct  
command for RF amplitude measurement described later in direct commands section). The measuring  
system is latching the peak value, so the RSSI level can be read after the end of receive packet. The  
RSSI value is reset during next transmit action of the reader, so the new tag response level can be  
measured. The RSSI levels calculated to the RF_IN1 and RF_IN2 are presented in Section 6.5.1.1 and  
Section 6.5.1.2. The RSSI has 7 steps (3 bits) with 4-dB increment. The input level is the peak-to-peak  
modulation level of RF signal measured on one side envelope (positive or negative).

The UART message looks like this:

    RSSI: 05
    RSSI: 05
    RSSI: 05
    RSSI: 05
    RSSI: 05
    RSSI: 05
    RSSI: 05
    RSSI: 06
    RSSI: 07
    RSSI: 07
    RSSI: 07
    RSSI: 07
    RSSI: 07
    RSSI: 05
    RSSI: 05
    RSSI: 05
    RSSI: 05
    RSSI: 04

For simplicity's sake, the RSSI value is sent at the same time the bar chart is refreshed on the LCD.

## Doc
[MSP430FR413x mixed-signal microcontrollers datasheet](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/blob/master/doc/msp430fr4133.pdf)  
[MSP430FR4xx and MSP430FR2xx family user's guide](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/blob/master/doc/slau445i.pdf)  
[DLP-7970ABP Hardware Update Overview](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/blob/master/doc/sloa226.pdf)  
[F430FRL152H NFC Sensor Tag Application Example With MSP430™ Microcontrollers](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/blob/master/doc/sloa233.pdf)  
[TRF7970A Multiprotocol Fully Integrated 13.56-MHz RFID and Near Field Communication (NFC) Transceiver IC](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/blob/master/doc/trf7970a.p)  
