# Mobile Control

The robot is designed to receive movement commands from an Android-based mobile device through Bluetooth.

## Command Interface

The embedded firmware interprets single-character commands received from the HC-05 module:

- `F` — Forward
- `B` — Backward
- `L` — Left
- `R` — Right
- `S` — Stop
- `U` — Increase speed
- `D` — Decrease speed

The STM32 can also transmit the current speed information back to the mobile side after speed adjustment commands.