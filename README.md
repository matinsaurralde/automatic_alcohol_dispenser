# automatic_alcohol_dispenser

Touchless alcohol dispenser built on an STM32F411 with FreeRTOS.

A hand near the dispenser triggers an ultrasonic sensor, the system reads the user's temperature, beeps, and a servo dispenses alcohol. The current temperature reading is shown on a 16x2 LCD.

## Hardware

- STM32F411VE (CMSIS-RTOS v2 / FreeRTOS)
- HC-SR04 ultrasonic distance sensor (hand detection, ≤15 cm)
- DHT22 temperature & humidity sensor
- SG90 servo motor (dispenser arm)
- 16x2 I2C LCD (status display)
- Passive buzzer (dispense confirmation)

## Tasks

| Task | Priority | Role |
|------|----------|------|
| `DHT_Task` | Above normal | Polls HC-SR04, triggers DHT22 read on proximity, pushes temp to queue |
| `Display` | Above normal | Reads queue, renders temperature on the LCD |
| `Buzzer` | Normal | Beeps on notify, then notifies Servo |
| `Servo` | Normal | Drives PWM (TIM2 CCR1) to open/close the dispenser |

Synchronization uses task notifications and a FreeRTOS queue for the temperature value.

## Build

Generated from STM32CubeMX (`final_embebidos_v2.ioc`). Open in STM32CubeIDE or build with the included CMake configuration and the ARM GCC toolchain.

## Context

Final project for the Embedded Systems course at UCC — 2020.
