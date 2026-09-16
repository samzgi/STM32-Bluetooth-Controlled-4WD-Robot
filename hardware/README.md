# Hardware

## Main Hardware Blocks

- STM32F4 Discovery development board
- HC-05 Bluetooth module
- Two L298N motor-driver boards
- Four TT DC geared motors
- Four wheels and mechanical chassis
- 12 V 3S 5P 18650 Li-ion battery pack with 10 A BMS
- LM2596 voltage regulator

## Motor Drive

The L298N drivers provide the interface between STM32 GPIO control signals and the DC motors. Direction is controlled through the motor-driver input pins, while PWM is used for speed control.

## Power

The documented power architecture uses a 12 V battery pack as the main source and an LM2596 regulator to provide a lower regulated voltage for the STM32 and Bluetooth electronics.

> Pin assignments and wiring diagrams should be added here when the original project hardware files are available.