# STM32 Bluetooth-Controlled 4WD Robot

A 4-wheel-drive robotic vehicle developed as an embedded systems project using an STM32F4 Discovery board and Bluetooth-based remote control from an Android mobile device.

## Project Overview

This project demonstrates a practical real-time embedded control system in which an STM32 microcontroller receives movement commands over Bluetooth and controls four DC geared motors through L298N motor drivers.

The vehicle supports:

- Forward and backward motion
- Left and right turning
- PWM-based speed control
- Bluetooth remote control
- UART interrupt-based communication
- Speed adjustment commands
- Feedback of the current speed to the mobile device
- Fail-safe stopping when an invalid command is received or communication is no longer producing valid commands

## System Architecture

```text
Android Mobile Device
        │
        │ Bluetooth
        ▼
     HC-05
        │
        │ USART2 / UART
        ▼
  STM32F4 Discovery
        │
        │ GPIO + PWM
        ▼
   L298N Motor Drivers
        │
        ▼
  4 × TT DC Gear Motors
        │
        ▼
     4WD Robot
```

## Hardware

| Component | Role |
|---|---|
| STM32F4 Discovery | Main microcontroller and real-time control |
| HC-05 | Bluetooth communication |
| 2 × L298N | DC motor direction and drive control |
| 4 × TT DC geared motors | Vehicle actuation |
| 4 × wheels | 4WD mechanical system |
| 12 V 3S 5P 18650 Li-ion battery | Main power source |
| 10 A BMS | Battery protection |
| LM2596 | Voltage regulation for low-voltage electronics |
| Chassis | Mechanical platform |

## Firmware Architecture

The firmware is implemented using the STM32 HAL framework in C.

### Communication

The HC-05 Bluetooth module is connected through USART2 at **9600 baud**. Reception is interrupt-driven using `HAL_UART_Receive_IT()`, allowing the main control loop to continue operating without blocking on serial input.

### Motor Control

The control firmware maps single-character Bluetooth commands to dedicated motor-control functions:

| Command | Function |
|---|---|
| `F` | Forward |
| `B` | Backward |
| `L` | Turn left |
| `R` | Turn right |
| `S` | Stop |
| `U` | Increase speed |
| `D` | Decrease speed |

### PWM Speed Control

Motor speed is controlled through TIM1 PWM. The documented configuration uses:

- Timer clock: 168 MHz
- Prescaler: 84 − 1
- Auto-reload: 1000 − 1
- PWM frequency: approximately 2 kHz
- Initial PWM value: 500
- PWM limits: 200–950
- Speed step: 50

The speed value is adjusted in software and applied to the motor drive through the PWM compare register.

### Feedback

When speed-up or speed-down commands are received, the firmware calculates the current speed percentage and transmits a formatted status message back through UART to the mobile device.

## Control Logic

The project uses a command-driven control structure based on a `switch-case` statement. Forward and backward commands drive all motors in the corresponding direction. Left and right commands stop the motors on one side while driving the opposite side forward, producing an in-place turning behavior.

The stop function sets the PWM output to zero and resets the motor-direction GPIO signals.

## Safety Behavior

The command receiver processes one-character commands and is re-armed after each received byte. The firmware also uses a default case that calls the stop function for unrecognized input, providing a simple fail-safe behavior at the application level.

## Development Workflow

The project follows the typical embedded-system development flow:

1. Define the robot movement and communication requirements.
2. Configure STM32 peripherals using the STM32 development environment.
3. Configure USART2 for Bluetooth communication.
4. Configure TIM1 for PWM generation.
5. Configure GPIO pins for motor-driver direction control.
6. Implement interrupt-driven Bluetooth reception.
7. Implement motor movement and speed-control functions.
8. Test the complete hardware/software integration.

## Repository Structure

```text
STM32-Bluetooth-Controlled-4WD-Robot/
├── README.md
├── documentation/
│   ├── README.md
│   └── Project_Report_Group25.pdf
├── firmware/
│   └── STM32/
│       └── README.md
├── hardware/
│   └── README.md
├── mobile/
│   └── README.md
└── figures/
    └── README.md
```

## Academic Context

**Course:** Embedded Systems / STM32 Project

**University:** Ege University, Faculty of Engineering, Department of Electrical and Electronics Engineering

**Academic Year:** 2025–2026

**Project:** STM32 ve Bluetooth Kontrollü Robotik Araç Tasarımı

**Group:** 25

**Authors:**

- Sam Zoghi
- Amirkia Mahd Gharehbagh
- Macit Cem Kızıl

## Technologies & Tools

- STM32F4 Discovery
- Embedded C
- STM32 HAL
- UART / USART
- PWM / Timer
- GPIO
- HC-05 Bluetooth
- L298N H-bridge motor driver

## Project Status

This repository contains the academic project documentation and is intended to preserve the project architecture, firmware concepts, and implementation details in a reproducible and professional format.

## Documentation

The original project report is planned to be stored under `documentation/Project_Report_Group25.pdf`.

---

Academic embedded-systems project developed at Ege University.
