/* Copyright 2021 iwatake2222

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
/*** Include ***/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "asio.hpp"
#include "driver/i2c.h"

#include "wifi_helper.h"
#include "imu_lsm9ds1.h"

/*** Type ***/
/* this must be the same as in PC side */
typedef struct {
    uint32_t timestamp; // [ms]
    int16_t acc[3];
    int16_t gyro[3];
    int16_t mag[3];
} PayloadIMU;

/*** Macro ***/
static const char *TAG = "main";
static const char *ADDRESS_DST = "192.168.1.2";
static uint16_t PORT_DST = 1234;

/*** Global variable ***/

/*** Function ***/
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Initialize NVS");
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "Initialize WiFi as Station mode");
    wifi_init_sta();

    ESP_LOGI(TAG, "Create UDP socket to send");
    asio::io_context io_context;
    asio::ip::udp::socket socket(io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(), 1234));
    asio::ip::udp::endpoint destination(asio::ip::address_v4::from_string(ADDRESS_DST), PORT_DST);

    ESP_LOGI(TAG, "Initialize IMU");
    ESP_ERROR_CHECK(imu_initialize());

    while(1) {
        uint32_t current_ms_time = pdTICKS_TO_MS(xTaskGetTickCount());
        ESP_LOGV(TAG, "%d\n", current_ms_time);

        /*** Get data from IMU sensor ***/
        // (void)imu_read_temperature();

        int16_t acc_x = 0;
        int16_t acc_y = 0;
        int16_t acc_z = 0;
        imu_read_acc(acc_x, acc_y, acc_z);

        int16_t gyro_x = 0;
        int16_t gyro_y = 0;
        int16_t gyro_z = 0;
        imu_read_gyro(gyro_x, gyro_y, gyro_z);

        int16_t mag_x = 0;
        int16_t mag_y = 0;
        int16_t mag_z = 0;
        imu_read_mag(mag_x, mag_y, mag_z);

        /*** Send the data to PC via WiFi (UDP) ***/
        PayloadIMU payload;
        payload.timestamp = current_ms_time;
        payload.acc[0] = acc_x;
        payload.acc[1] = acc_y;
        payload.acc[2] = acc_z;
        payload.gyro[0] = gyro_x;
        payload.gyro[1] = gyro_y;
        payload.gyro[2] = gyro_z;
        payload.mag[0] = mag_x;
        payload.mag[1] = mag_y;
        payload.mag[2] = mag_z;
        socket.send_to(asio::buffer(&payload, sizeof(payload)), destination);

        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
