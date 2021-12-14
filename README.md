# CYBT-343026-EVAL

### Overview

The Infineon CYBT-343026-EVAL Kit enables you to evaluate and develop single-chip Bluetooth&#174; applications using the CYW20706, an ultra-low-power dual-mode Bluetooth&#174; 5.0 wireless MCU device. The CYW20706 is a stand-alone baseband processor with an integrated 2.4 GHz transceiver supporting LE/BR/EDR. The CYW20706 employs high integration to reduce external components, thereby minimizing the device's footprint and cost. This kit helps evaluate device functionality and develop applications quickly for faster time-to-market.

### Kit Features

* CYW20706 is a monolithic, single chip, Bluetooth&#174; dual-mode System-on-a-Chip (SoC) that includes a baseband processor, an Arm&#174; Cortex&#174;-M3 processor, and an integrated transceiver.
* The CYBT-343026-EVAL enables you to evaluate and develop applications on the AIROC&#8482; CYBT-343026-01 Bluetooth&#174; and Bluetooth&#174; LE module. CYBT-343026-EVAL can be used as a standalone evaluation kit or can be combined with Arduino compatible shields.

### Standalone Usage

To use the CYBT-343026-EVAL:

1) Configure the evaluation board headers/switches to the desired settings

2) Connect the evaluation board to a PC via a USB cable

3) Refer to KBA226703 for platform files, Makefile target generation, and HCI UART switch position setting for programming

4) Use ModusToolbox&#8482; 2.0 and BTSDK 2.0 (or higher) to develop your application, program, and test

Note: Recover the CYBT-343026-EVAL before programming. The Arduino-compatible headers(J3/J4/J6/J7) are optional connections, which provide additional I/O connections to the module and allow for other Arduino shields to be used during development.

Optional Usage with Arduino Shield:

Arduino compatible shields can be connected through the Arduino compatible headers (J3/J4/J6/J7) to provide additional I/O connections and functionality.

### Additional Information

Max UART baud rate is 1M. Use baud rate of 115200 for Client Control.

For more information, see [CYBT-343026-EVAL](http://www.cypress.com/CYBT-343026-EVAL)
