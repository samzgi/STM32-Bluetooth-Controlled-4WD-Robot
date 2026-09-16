# STM32 Firmware

The firmware is written in Embedded C using the STM32 HAL framework.

## Main Functions

- Bluetooth command reception through USART2
- Interrupt-driven UART reception
- PWM generation with TIM1
- GPIO-based motor direction control
- Forward / backward / left / right / stop commands
- PWM speed increase and decrease
- Speed feedback transmission to the mobile device

## Bluetooth Commands

| Command | Action |
|---|---|
| `F` | Forward |
| `B` | Backward |
| `L` | Left |
| `R` | Right |
| `S` | Stop |
| `U` | Speed up |
| `D` | Speed down |

The original report identifies `HAL_UART_RxCpltCallback()` as the interrupt callback used to re-enable UART reception after a byte is received.