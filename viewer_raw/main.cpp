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
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <vector>
#include <string>
#include <chrono>
#include <thread>

#include "my_udp.h"

/*** Type ***/
/* this must be the same as in ESP32 side */
typedef struct {
    uint32_t timestamp; // [ms]
    int16_t acc[3];
    int16_t gyro[3];
    int16_t mag[3];
} PayloadImu;

typedef struct {
    uint32_t timestamp; // [ms]
    float acc[3];
    float gyro[3];
    float mag[3];
} ValImu;

class ValImuList {
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

/*** Macro ***/
/* this must be the same as in ESP32 side */
#define SHORT_FS 32768
#define GYRO_FS 500 // [dps]
#define ACC_FS 2 // [G]
#define MAG_FS 4 // [Gauss]

/*** Global variable ***/
bool do_exit = false;
ValImuList val_imu_list;

/*** Function ***/
static std::string GetDateTimeText()
{
    char text[64];
    time_t time_now = time(NULL);
    struct tm* local_time = localtime(&time_now);
    strftime(text, sizeof(text), "%Y/%m/%d_%H:%M:%S", local_time);
    return std::string(text);
}

static ValImu NormalizeIMU(const PayloadImu& payload)
{
    ValImu val_imu;
    val_imu.timestamp = payload.timestamp;
    for (int32_t i = 0; i < 3; i++) {
        val_imu.acc[i] = (float)payload.acc[i] / SHORT_FS * ACC_FS,
        val_imu.gyro[i] = (float)payload.gyro[i] / SHORT_FS * GYRO_FS;
        val_imu.mag[i] = (float)payload.mag[i] / SHORT_FS * MAG_FS;
    }
    return val_imu;
}

static void PrintIMU(const ValImu& val_imu)
{
    printf("[%s] timestamp = %d\n", GetDateTimeText().c_str(), val_imu.timestamp);
    printf("    ACC : %.03f, %.03f, %.03f\n", val_imu.acc[0], val_imu.acc[1], val_imu.acc[2]);
    printf("    GYRO: %.03f, %.03f, %.03f\n", val_imu.gyro[0], val_imu.gyro[1], val_imu.gyro[2]);
    printf("    MAG : %.03f, %.03f, %.03f\n", val_imu.mag[0], val_imu.mag[1], val_imu.mag[2]);
}

static void ThreadEchoBacker()
{
    MyUdp udp_recv("0.0.0.0", 1234, MyUdp::Mode::kModeRecvBlocking);
    
    while (true) {
        if (do_exit) break;
        PayloadImu payload = { 0 };
        udp_recv.Recv((char*)&payload, sizeof(payload));
        ValImu val_imu = NormalizeIMU(payload);
        val_imu_list.push(val_imu);
        //PrintIMU(val_imu);
    }
}

int main(int argc, char *argv[])
{
    std::thread thread_echo(ThreadEchoBacker);
    
    while (true) {
        ValImu val_imu;
        if (val_imu_list.pop_latest(val_imu)) {
            PrintIMU(val_imu);
        }
    }

    thread_echo.join();

    return 0;
}
