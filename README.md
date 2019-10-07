# TRF7970ABP_FR4133_FRL_Reader
## Description
Texas Instrument [SLOA233](http://www.ti.com/lit/an/sloa233/sloa233.pdf) application. Designed to read temperature data from the [RF430-TMPSNS-EVM](http://www.ti.com/tool/RF430-TMPSNS-EVM) using the [MSP-EXP430FR4133](http://www.ti.com/tool/MSP-EXP430FR4133) launchpad and the [DLP-7970ABP](http://www.ti.com/tool/DLP-7970ABP) booster pack.
## Original source code
Source code can be downloaded [here](http://www.ti.com/lit/an/sloa233/sloa233.zip). A few changes have been made to make it work.

## Software changes
[b752b348c85d3f6210ea718dc047b29057960b5a] Fix compilation error caused by include casing.
[c6101a07b829fa8c737466e8e43a63f3e39bf3a0] Fix watchdog disable not working.
[ed62871c9fffa22a1655af4e9c1f3e885a53b0ff] Enable UART host logging.
## Hardware setup
Ensure that the IRQ jumper on the DLP-7970ABP booster pack is set to position 1 (default) as described in [DLP-7970ABP Hardware Update Overview](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/blob/master/doc/sloa226.pdf)

## Doc
[MSP430FR413x mixed-signal microcontrollers datasheet](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/blob/master/doc/msp430fr4133.pdf)  
[MSP430FR4xx and MSP430FR2xx family user's guide](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/blob/master/doc/slau445i.pdf)  
[DLP-7970ABP Hardware Update Overview](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/blob/master/doc/sloa226.pdf)  
[F430FRL152H NFC Sensor Tag Application Example With MSP430™ Microcontrollers](https://github.com/Klagopsalmer/TRF7970ABP_FR4133_FRL_Reader/blob/master/doc/sloa233.pdf)  
