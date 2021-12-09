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
#include <cmath>
#include "esp_system.h"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
// #define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "driver/i2c.h"

#include "imu_lsm9ds1.h"

 
/*** Macro ***/
static const char *TAG = "imu";

#define I2C_NUM I2C_NUM_0
#define I2C_SCL_IO 19
#define I2C_SDA_IO 18
#define I2C_FREQ_HZ 100000
#define I2C_TX_BUF_DISABLE 0
#define I2C_RX_BUF_DISABLE 0
#define ACK_CHECK_EN 1
#define ACK_CHECK_DIS 0

#define ACC_GYRO_ADDR 0x6A
#define ACC_GYRO_CMD_WHOAMI 0x0F
#define ACC_GYRO_CMD_CTRL_REG1_G 0x10
#define ACC_GYRO_CMD_CTRL_REG2_G 0x11
#define ACC_GYRO_CMD_CTRL_REG3_G 0x12
#define ACC_GYRO_CMD_OUT_TEMP_L 0x15
#define ACC_GYRO_CMD_OUT_X_G 0x18
#define ACC_GYRO_CMD_CTRL_REG4 0x1E
#define ACC_GYRO_CMD_CTRL_REG5_XL 0x1F
#define ACC_GYRO_CMD_CTRL_REG6_XL 0x20
#define ACC_GYRO_CMD_CTRL_REG7_XL 0x21
#define ACC_GYRO_CMD_CTRL_REG8 0x22
#define ACC_GYRO_CMD_CTRL_REG9 0x23
#define ACC_GYRO_CMD_OUT_X_XL 0x28
#define ACC_GYRO_CMD_FIFO_CTRL 0x2E

#define MAG_ADDR 0x1C
#define MAG_CMD_WHOAMI 0x0F
#define MAG_CMD_CTRL_REG1_M 0x20
#define MAG_CMD_CTRL_REG2_M 0x21
#define MAG_CMD_CTRL_REG3_M 0x22
#define MAG_CMD_CTRL_REG4_M 0x23
#define MAG_CMD_OUT_X_M 0x28

/*** Global variable ***/

/*** Function ***/
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    // conf.clk_flags = 0;          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    esp_err_t err = i2c_param_config(I2C_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2C_NUM, conf.mode, I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);
}

static esp_err_t i2c_write(uint8_t slave_addr, uint8_t sub_addr, uint8_t* data, uint32_t len = 1)
{
    int ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, sub_addr, ACK_CHECK_EN);
    for (int i = 0; i < len; i++) {
        i2c_master_write_byte(cmd, data[i], ACK_CHECK_EN);
    }
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_write(uint8_t slave_addr, uint8_t sub_addr, uint8_t data)
{
    int ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, sub_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read(uint8_t slave_addr, uint8_t sub_addr, uint8_t* data, uint32_t len = 1)
{
    int ret = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, sub_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

union ValShort {
    struct {
        uint8_t low;
        uint8_t high;
    } raw;
    int16_t val;
};

#define SHORT_FS 32768
#define GYRO_FS 500 // [dps]
#define ACC_FS 2 // [G]
#define MAG_FS 4 // [Gauss]

int16_t imu_read_temperature()
{
    ValShort val;
    i2c_read(MAG_ADDR, ACC_GYRO_CMD_OUT_TEMP_L, (uint8_t*)&val, sizeof(val));
    ESP_LOGV(TAG, "[TEMP] val = %d, val_l = 0x%02X, val_h = 0x%02X", val.val, val.raw.low, val.raw.high);
    val.raw.high &= 0x0F;
    if (val.raw.high >= 0x08) {
        val.raw.high |= 0xF0;
    }
    ESP_LOGD(TAG, "[TEMP] val = %d", val.val);
    return val.val;
}


void imu_read_gyro(int16_t& x, int16_t& y, int16_t& z)
{
    ValShort val[3];
    i2c_read(ACC_GYRO_ADDR, ACC_GYRO_CMD_OUT_X_G, (uint8_t*)&val, sizeof(val));
    for (int32_t i = 0; i < 3; i++) {
        ESP_LOGV(TAG, "[GYRO] %d: val = %d, val_l = 0x%02X, val_h = 0x%02X", i, val[i].val, val[i].raw.low, val[i].raw.high);
    }
    ESP_LOGD(TAG, "[GYRO] x = %.03f, y = %.03f, z = %.03f", (float)val[0].val / SHORT_FS * GYRO_FS, (float)val[1].val / SHORT_FS * GYRO_FS, (float)val[2].val / SHORT_FS * GYRO_FS);

    x = val[0].val;
    y = val[1].val;
    z = val[2].val;
}

void imu_read_acc(int16_t& x, int16_t& y, int16_t& z)
{
    ValShort val[3];
    i2c_read(ACC_GYRO_ADDR, ACC_GYRO_CMD_OUT_X_XL, (uint8_t*)&val, sizeof(val));
    for (int32_t i = 0; i < 3; i++) {
        ESP_LOGV(TAG, "[ACC]  %d: val = %d, val_l = 0x%02X, val_h = 0x%02X", i, val[i].val, val[i].raw.low, val[i].raw.high);
    }
    ESP_LOGD(TAG, "[ACC] x = %.03f, y = %.03f, z = %.03f", (float)val[0].val / SHORT_FS * ACC_FS, (float)val[1].val / SHORT_FS * ACC_FS, (float)val[2].val / SHORT_FS * ACC_FS);
    x = val[0].val;
    y = val[1].val;
    z = val[2].val;
}

void imu_read_mag(int16_t& x, int16_t& y, int16_t& z)
{
    ValShort val[3];
    i2c_read(MAG_ADDR, MAG_CMD_OUT_X_M, (uint8_t*)&val, sizeof(val));
    for (int32_t i = 0; i < 3; i++) {
        ESP_LOGV(TAG, "[MAG]  %d: val = %d, val_l = 0x%02X, val_h = 0x%02X", i, val[i].val, val[i].raw.low, val[i].raw.high);
    }
    ESP_LOGD(TAG, "[MAG] x = %.03f, y = %.03f, z = %.03f", (float)val[0].val / SHORT_FS * MAG_FS, (float)val[1].val / SHORT_FS * MAG_FS, (float)val[2].val / SHORT_FS * MAG_FS);
    float rad = atan2((float)val[1].val / SHORT_FS * MAG_FS, (float)val[2].val / SHORT_FS * MAG_FS); 
    ESP_LOGD(TAG, "[MAG] deg = %.01f", rad * 180 / 3.14159);

    x = val[0].val;
    y = val[1].val;
    z = val[2].val;
}

esp_err_t imu_initialize()
{
    int ret = ESP_OK;

    ESP_LOGI(TAG, "imu_initialize");
    ESP_ERROR_CHECK(i2c_master_init());

    ESP_LOGI(TAG, "device test");
    uint8_t whoami = 0;
    i2c_read(ACC_GYRO_ADDR, ACC_GYRO_CMD_WHOAMI, &whoami);
    ESP_LOGI(TAG, "[ACC_GYRO] whoami = 0x%02X", whoami);
    ESP_ERROR_CHECK_WITHOUT_ABORT(whoami != 0x68);
    i2c_read(MAG_ADDR, MAG_CMD_WHOAMI, &whoami);
    ESP_LOGI(TAG, "[MAG] whoami = 0x%02X", whoami);
    ESP_ERROR_CHECK_WITHOUT_ABORT(whoami != 0x3D);

    ESP_LOGI(TAG, "Setting Accelerometer and gyroscope");
    uint8_t odr_g = 6;  // ODR = 952 Hz
    uint8_t fs_g = 1;   // 500 dps (degree per second)
    uint8_t bw_g = 0;
    i2c_write(ACC_GYRO_ADDR, ACC_GYRO_CMD_CTRL_REG1_G, odr_g << 5 | fs_g << 3 | bw_g);

    uint8_t odr_acc = odr_g;
    uint8_t fs_acc = 0; // +-2G
    uint8_t bw_acc = bw_g;
    i2c_write(ACC_GYRO_ADDR, ACC_GYRO_CMD_CTRL_REG6_XL, odr_acc << 5 | fs_acc << 3 | bw_acc);

    uint8_t fmode = 6;  // Continuous mode. If the FIFO is full, the new sample overwrites the older sample.
    i2c_write(ACC_GYRO_ADDR, ACC_GYRO_CMD_FIFO_CTRL, fmode << 5);

    ESP_LOGI(TAG, "Setting Magnetometer");
    uint8_t temp_comp = 0;  // temperature compensation disabled
    uint8_t om = 2; // high-performance mode
    uint8_t data_rate = 6; //ODR = 40Hz
    i2c_write(MAG_ADDR, MAG_CMD_CTRL_REG1_M, temp_comp << 7 | om << 5 | data_rate << 2);

    uint8_t fs_mag = 0; // +- 4 gauss
    i2c_write(MAG_ADDR, MAG_CMD_CTRL_REG2_M, fs_mag << 5);

    uint8_t md_mag = 0; // continuous-conversion mode
    i2c_write(MAG_ADDR, MAG_CMD_CTRL_REG3_M, md_mag);

    uint8_t omz_mag = 2; // high-performance mode
    i2c_write(MAG_ADDR, MAG_CMD_CTRL_REG4_M, omz_mag << 2);


    return ret;
}
