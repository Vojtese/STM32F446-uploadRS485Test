# STM32F446 Sensor Test and Hardware Validation

This repository contains firmware for validating analog and digital interfaces of the rainwater signal acquisition unit. It is used during development to test ADC channels, signal conditioning, and galvanic isolation.

## 🚀 Features

- ADC testing for pressure, flow, and voltage sensors
- UART CLI interface for debugging
- RS485 and 1-Wire communication validation
- PWM and timer-based signal generation
- CMSIS LL drivers for direct peripheral access

## 📁 Project Structure

- `Core/`: Sensor test routines
- `Drivers/`: STM32 LL drivers
- `.ioc`: STM32CubeMX configuration

## 🔗 Related Projects

- [STM32F446-Bootloader](https://github.com/Vojtese/STM32F446-Bootloader)

## 📜 License

This project is licensed under the GNU General Public License v3.0.
