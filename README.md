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
- Open `sensor_esp32` folder
- `idf.py menuconfig`
    - set `ESP_WIFI_SSID` for your environment
    - set `ESP_WIFI_PASSWORD` for your environment
    - if you use VSCode, run the command in `ESP-IDF: Open ESP-IDF Terminal`
- Build
    - `ESP-IDF: Build your project` in VSCode
- Flash
    - `ESP-IDF: Flash (UART) your project` in VSCode

## How to Build Visualization Tool
aaa