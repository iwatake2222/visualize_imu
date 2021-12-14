/* Copyright 2020 iwatake2222

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
#ifndef TYPE_
#define TYPE_

/*** Include ***/
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

/* this must be the same as in ESP32 side */
#define SHORT_FS 32768
#define ACC_FS 2 // [G]
#define GYRO_FS 500 // [dps]
#define MAG_FS 4 // [Gauss]

/* this must be the same as in ESP32 side */
typedef struct {
    uint32_t timestamp; // [ms]
    int16_t acc[3];
    int16_t gyro[3];
    int16_t mag[3];
} PayloadImu;

/* this is for internal use */
typedef struct {
    uint32_t timestamp; // [ms]
    float acc[3];
    float gyro[3];
    float mag[3];
} ValImu;

/* buffer from UDP receiver to viewer */
class SharedList {
public:
    void push(const ValImu& data) {
        std::lock_guard<std::mutex> lock(mtx_);
        val_list_.push_back(data);
    }

    bool pop_latest(ValImu& val_imu) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (val_list_.empty()) {
            return false;
        } else {
            val_imu = val_list_.back();
            val_list_.clear();
            return true;
        }
    }

private:
    std::mutex mtx_;
    std::vector<ValImu> val_list_;
};

#endif
