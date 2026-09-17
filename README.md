# MSP432 Seizure Detection and Emergency Response System

A two-module embedded systems project built with MSP432 microcontrollers that simulates seizure detection and emergency response using accelerometer data, UART communication, LCD output, and a seven-segment timer.

## Project Overview

The system is divided into two independent MSP432 modules:

### Detection Module

- Reads simulated accelerometer X/Y movement using ADC14
- Calculates movement magnitude from sensor readings
- Classifies seizure activity through multiple levels
- Controls local LED and buzzer feedback
- Sends level, magnitude, event, heartbeat, and ACK packets to the response module using UART

### Response Module

- Receives UART packets from the detection module
- Displays seizure level, magnitude, and timer information on a 16x2 LCD
- Tracks seizure duration using a three-digit seven-segment display
- Sends arm/disarm, test, mute, reset, and threshold commands back to the detection module
- Detects UART communication failure using heartbeat packets

## Detection Levels

| Level | State | System Response |
|---|---|---|
| 0 | Normal | Normal movement, timer reset |
| 1 | Suspicious | Abnormal movement spike detected |
| 2 | Tonic | Seizure timer starts |
| 3 | Clonic | Timer continues |
| 4 | Confirmed seizure | Alert state active |
| 5 | Post-seizure | Timer freezes until reset |

## Embedded Systems Features

- MSP432 microcontrollers
- C programming
- 14-bit ADC14
- UART communication
- UART receive interrupts
- Packet-based communication with checksum
- Heartbeat communication checking
- SysTick timer interrupts
- Timer_A0 PWM buzzer control
- GPIO buttons and LEDs
- LCD output
- Seven-segment display multiplexing
- Low-power mode
- Hardware/software integration

## Hardware

### Detection Module

- MSP432 LaunchPad
- Accelerometer sensor
- Buzzer on P2.4
- Onboard LEDs for local feedback
- UART RX on P3.2
- UART TX on P3.3

### Response Module

- MSP432 LaunchPad
- 16x2 character LCD
- Three-digit seven-segment display
- Potentiometer on P4.7/A6
- Arm/disarm button on P1.1
- Test/mute/reset button on P1.4
- Buzzer on P2.4
- UART RX on P3.2
- UART TX on P3.3

## Communication

The detection module sends seizure level and movement magnitude data to the response module using UART A2.

```text
Detection P3.3 (TX)  --->  Response P3.2 (RX)
Detection P3.2 (RX)  <---  Response P3.3 (TX)
Detection GND        --->  Response GND
```

The response module receives packets using a UART receive interrupt and uses the received level and magnitude to update the LCD, seven-segment timer, and communication fault logic.

The response module also sends command packets and threshold updates back to the detection module. These commands are used for arm/disarm, test, mute, reset, and potentiometer-based sensitivity adjustment.

## UART Packet Format

```text
+------------+-------------+-------------+------------+----------+
| Start Byte | Packet Type |   Payload   |  Checksum  | End Byte |
+------------+-------------+-------------+------------+----------+
|    0xAA    |    1 byte   |  0-8 bytes  |   1 byte   |   0x55   |
+------------+-------------+-------------+------------+----------+
```

## Timing

The detection module uses SysTick for accelerometer sampling and heartbeat timing.

```text
50 samples per second = normal sampling mode
100 samples per second = faster detection mode
```

ADC conversions are triggered by SysTick, and the detection module sends sensor packets regularly or whenever the seizure level changes.

The response module uses SysTick with a 1 ms interrupt period.

```text
1000 timer interrupts x 1 ms = 1 second
```

The response module uses this timing to refresh the seven-segment display, count seizure duration, flash the UART error LED, send heartbeat packets, and check for communication timeout.

The buzzer uses Timer_A0 PWM on P2.4.

## Repository Structure

```text
msp432-seizure-detection-system/
├── module-a-detection/
│   └── detection_main.c
├── module-b-response/
│   ├── response_main.c
│   ├── lcdLib_432.c
│   └── lcdLib_432.h
└── README.md
```

## Skills Demonstrated

- Embedded C programming
- Register-level MSP432 programming
- ADC configuration
- UART communication
- Interrupt handling
- PWM generation
- GPIO control
- State machine design
- Hardware/software integration
- Embedded systems debugging

## Notes

This project is an embedded systems prototype created for educational purposes. It is not intended for real medical use.
