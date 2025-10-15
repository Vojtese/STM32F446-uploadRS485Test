# STM32F446 RS485 Upload Test

This repository contains a firmware test application for validating RS485-based packet transfer and protocol parsing. It simulates the front-end of an in-application programming (IAP) system but does not implement actual Flash writing or firmware update logic.

---

## 🚀 Features

- RS485 communication using MAX1480 bus driver
- DMA-based UART reception
- Custom packet protocol (not XMODEM)
- CRC validation of incoming packets
- Packet parsing and debug output via UART
- Compatible with [serial_BIN_file_transfer](https://github.com/Vojtese/serial_BIN_file_transfer)

---

## ❌ Limitations

- No Flash erase/write operations
- No jump to new application
- No IAPstruct or persistent update tracking
- No bootloader integration

---

## 🧠 CMSIS LL Driver Usage

- `LL_USART_ReceiveData8()`, `LL_USART_IsActiveFlag_RXNE()` – RS485 reception
- `LL_DMA_EnableChannel()` – DMA for UART
- `LL_GPIO_SetOutputPin()` – RS485 direction control
- `LL_CRC_FeedData32()` – CRC validation

---

## 📁 Project Structure

- `Core/` – RS485 and packet parsing logic
- `Drivers/` – STM32 LL drivers
- `packet_parser.c` – Custom protocol handler
- `.ioc` – STM32CubeMX configuration
- `STM32F446RETX_FLASH.ld` – Linker script
- `docs/` – Optional diagrams or flowcharts

---

## 🔗 Related Projects

- [STM32F446-Bootloader](https://github.com/Vojtese/STM32F446-Bootloader)
- [STM32F446-APP1](https://github.com/Vojtese/STM32F446-APP1)
- [STM32F446-APP2](https://github.com/Vojtese/STM32F446-APP2)
- [serial_BIN_file_transfer](https://github.com/Vojtese/serial_BIN_file_transfer)

---

## 📜 License

This project is licensed under the GNU General Public License v3.0.
