# About
aaa

## Component
- ESP32 Dev Board
    - https://www.amazon.co.jp/gp/product/B0718T232Z
- IMU (LSM9DS1)
    - https://akizukidenshi.com/catalog/g/gM-15381/
    - 9DoF
    - i2c interface
- PC + WiFi

## Connection
### Overview
- IMU - (i2c) -> ESP32
- ESP32 - (WiFi(UDP))-> PC

# How to Build
## How to Build Firmware for ESP32 + IMU
- Install espressif.esp-idf-extension extension in VSCode
- Open `imu_esp32` folder
- `idf.py menuconfig` in `ESP-IDF: Open ESP-IDF Terminal` in VSCode
    - set `ESP_WIFI_SSID` for your environment
    - set `ESP_WIFI_PASSWORD` for your environment
    - set `CONFIG_COMPILER_OPTIMIZATION_***` as you want like O2
- Build
    - `ESP-IDF: Build your project` in VSCode
- Flash
    - `ESP-IDF: Flash (UART) your project` in VSCode

## How to Build Visualization Tool
aaa