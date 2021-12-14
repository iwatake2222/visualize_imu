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
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <deque>
#include <string>
#include <chrono>
#include <thread>

#include <opencv2/opencv.hpp>

#include "my_udp.h"
#include "graph_scatter.h"
#include "graph_object.h"


/*** Macro ***/
/* this must be the same as in ESP32 side */
#define SHORT_FS 32768
#define ACC_FS 2 // [G]
#define GYRO_FS 500 // [dps]
#define MAG_FS 4 // [Gauss]

/*** Type ***/
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

/*** Global variable ***/
bool do_exit = false;
SharedList shared_list;
cv::Point3f mag_calib;

/*** Function ***/
static inline float Deg2Rad(float deg) { return static_cast<float>(deg * M_PI / 180.0); }

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
    val_imu.mag[0] -= mag_calib.x;
    val_imu.mag[1] -= mag_calib.y;
    val_imu.mag[2] -= mag_calib.z;
    return val_imu;
}

static void PrintIMU(const ValImu& val_imu)
{
    printf("[%s] timestamp = %d\n", GetDateTimeText().c_str(), val_imu.timestamp);
    printf("    ACC : %.03f, %.03f, %.03f\n", val_imu.acc[0], val_imu.acc[1], val_imu.acc[2]);
    printf("    GYRO: %.03f, %.03f, %.03f\n", val_imu.gyro[0], val_imu.gyro[1], val_imu.gyro[2]);
    printf("    MAG : %.03f, %.03f, %.03f\n", val_imu.mag[0], val_imu.mag[1], val_imu.mag[2]);
}

static void ThreadReceiver()
{
    MyUdp udp_recv("0.0.0.0", 1234, MyUdp::Mode::kModeRecvBlocking);

    while (true) {
        if (do_exit) break;
        PayloadImu payload = { 0 };
        udp_recv.Recv((char*)&payload, sizeof(payload));
        ValImu val_imu = NormalizeIMU(payload);
        shared_list.push(val_imu);
        //PrintIMU(val_imu);
    }
}



int main(int argc, char* argv[])
{
    std::thread thread_receiver(ThreadReceiver);

    GraphScatterImu graph_scatter(ACC_FS, GYRO_FS, MAG_FS / 2.0);
    GraphObject graph_object("Object", 5);

    std::deque<ValImu> val_imu_list;    /* Store history for scatter graph */

    while (true) {
        /*** Get the latest IMU data from sensor via UDP ***/
        ValImu val_imu_latest;
        if (shared_list.pop_latest(val_imu_latest)) {
            //PrintIMU(val_imu);
            val_imu_list.push_back(val_imu_latest);
            if (val_imu_list.size() > 10000) {
                val_imu_list.pop_front();
            }
        }

        /*** Key input ***/
        int32_t key = cv::waitKey(1);
        if (key == 27) break;   /* ESC to quit */
        if (key == 'r') {
            /* Clear list */
            val_imu_list.clear();
        }
        if (key == 'c') {
            /* Calibration */
            /* todo: better to use the method of least squares for sphere*/
            cv::Point3f calib = 0;
            for (const auto& val_imu : val_imu_list) {
                calib.x += val_imu.mag[0];
                calib.y += val_imu.mag[1];
                calib.z += val_imu.mag[2];
            }
            mag_calib.x = calib.x / val_imu_list.size();
            mag_calib.y = calib.y / val_imu_list.size();
            mag_calib.z = calib.z / val_imu_list.size();
            val_imu_list.clear();
        }
        if (key == 'C') {
            /* Clear Calibration */
            mag_calib.x = 0;
            mag_calib.y = 0;
            mag_calib.z = 0;
        }

        /*** Scatter Graph ***/
        /* Make list for scatter graph */
        std::vector<cv::Point3f> acc_list;
        std::vector<cv::Point3f> gyro_list;
        std::vector<cv::Point3f> mag_list;
        for (const auto& val_imu : val_imu_list) {
            acc_list.push_back(cv::Point3f(val_imu.acc[0], val_imu.acc[1], val_imu.acc[2]));
            gyro_list.push_back(cv::Point3f(val_imu.gyro[0], val_imu.gyro[1], val_imu.gyro[2]));
            mag_list.push_back(cv::Point3f(val_imu.mag[0], val_imu.mag[1], val_imu.mag[2]));
        }

        /* Draw scatter graph (will be displayed at the next waitKey) */
        graph_scatter.Update(key, acc_list, gyro_list, mag_list);

        /*** Draw Object ***/
        static float deg = 0;
        if (++deg >= 360) deg = 0;
        graph_object.Update(key, deg, deg, deg, 0.5, 0.5, -0.5);
    }

    do_exit = true;
    thread_receiver.join();

    return 0;
}
