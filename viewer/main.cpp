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

#include "type.h"
#include "my_udp.h"
#include "graph_scatter.h"
#include "graph_object.h"
#include "lsm_sphere.h"

/*** Macro ***/

/*** Global variable ***/
bool do_exit = false;
SharedList shared_list;
LsmSphere mag_calibrator;

/*** Function ***/
static inline float Deg2Rad(float deg) { return static_cast<float>(deg * M_PI / 180.0); }
static inline float Rad2Deg(float rad) { return static_cast<float>(rad * 180.0 / M_PI); }

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
    val_imu.mag[0] -= mag_calibrator.Get().x;
    val_imu.mag[1] -= mag_calibrator.Get().y;
    val_imu.mag[2] -= mag_calibrator.Get().z;
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

class ObjectImu
{
public:
    ObjectImu() {};
    void Calculate(ValImu val_imu, float& pitch, float& yaw, float& roll, float& x, float& y, float& z)
    {
        float acc_x = val_imu.acc[0];
        float acc_y = val_imu.acc[2];
        float acc_z = val_imu.acc[1];
        float gyro_x = val_imu.gyro[0];
        float gyro_y = val_imu.gyro[2];
        float gyro_z = val_imu.gyro[1];
        float mag_x = val_imu.mag[0];
        float mag_y = val_imu.mag[2];
        float mag_z = val_imu.mag[1];
        pitch = 0;
        yaw = 0;
        roll = 0;
        x = 0;
        y = 0;
        z = 0;

        pitch = atan2(-acc_y, -acc_z);
        roll = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z));

        //float Dx = cos(pitch) * mag_x + sin(pitch) * sin(roll) * mag_y + sin(pitch) * cos(roll) * mag_z;
        //float Dy = cos(roll)* mag_y - sin(roll) * mag_z;
        //float Dz = -sin(pitch) * mag_x + cos(pitch) * sin(roll) * mag_y + cos(pitch) * cos(roll) * mag_z;
        //yaw = atan2(-Dy, Dx);

        pitch = Rad2Deg(pitch);
        yaw = Rad2Deg(yaw);
        roll = Rad2Deg(roll);

    }
};

int main(int argc, char* argv[])
{
    std::thread thread_receiver(ThreadReceiver);

    GraphScatterImu graph_scatter(ACC_FS, GYRO_FS, MAG_FS / 2.0);
    GraphObject graph_object("Object", 5);
    ObjectImu object_imu;

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
            mag_calibrator.Calculate(val_imu_list);
            val_imu_list.clear();
        }
        if (key == 'C') {
            mag_calibrator.Clear();
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
        float pitch;
        float yaw;
        float roll;
        float x;
        float y;
        float z;
        object_imu.Calculate(val_imu_latest, pitch, yaw, roll, x, y, z);
        graph_object.Update(key, pitch, yaw, roll, x, y, z);
    }

    do_exit = true;
    thread_receiver.join();

    return 0;
}
