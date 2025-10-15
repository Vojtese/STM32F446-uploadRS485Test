# STM32F446 RS485 Upload Test

This repository contains firmware for testing firmware upload over RS485. It extends the bootloader’s UART-based IAP to RS485, enabling robust field updates in electrically noisy environments.

## 🚀 Features

- RS485 communication using MAX1480 bus driver
- DMA-based UART transmission for efficient data handling
- CRC verification and packet parsing
- Compatible with GUI uploader and bootloader protocol
- Designed for integration into underground tank unit

## 🧠 CMSIS LL Driver Usage

- `LL_USART_TransmitData8()`, `LL_USART_IsActiveFlag_TXE()` – RS485 transmission
- `LL_DMA_EnableChannel()` – DMA for UART
- `LL_GPIO_SetOutputPin()` – RS485 direction control
- `LL_CRC_FeedData32()` – CRC validation

## 📁 Project Structure

- `Core/`: RS485 upload logic
- `Drivers/`: STM32 LL drivers
- `.ioc`: STM32CubeMX configuration

## 🔗 Related Projects

- [STM32F446-Bootloader](https://github.com/Vojtese/STM32F446-Bootloader)
- [serial_BIN_file_transfer](https://github.com/Vojtese/serial_BIN_file_transfer)

## 📜 License

This project is licensed under the GNU General Public License v3.0.
