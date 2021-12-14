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
/*** Include ***/
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <numeric>

#include <opencv2/opencv.hpp>

#include "lsm_sphere.h"

/*** Macro ***/

/*** Global variable ***/

/*** Function ***/
void LsmSphere::Calculate(std::deque<ValImu>& val_imu_list)
{
    /* Get raw value */
    for (auto& val : val_imu_list) {
        val.mag[0] += origin_.x;
        val.mag[1] += origin_.y;
        val.mag[2] += origin_.z;
    }
#if 0
    cv::Point3f val_sum = 0;
    for (const auto& val_imu : val_imu_list) {
        val_sum.x += val_imu.mag[0];
        val_sum.y += val_imu.mag[1];
        val_sum.z += val_imu.mag[2];
    }
    origin_.x = val_sum.x / val_imu_list.size();
    origin_.y = val_sum.y / val_imu_list.size();
    origin_.z = val_sum.z / val_imu_list.size();
#else
    /*
    (R     (n      Sum(x)    Sum(y)   Sum(z)               (Sum(x^2 + y^2 + z^2)
    X0  =   Sum(x) Sum(x^2)  Sum(xy)  Sum(xz)           *   Sum(x(x^2 + y^2 + z^2))
    Y0      Sum(y) Sum(xy)   Sum(y^2) Sum(yz)               Sum(y(x^2 + y^2 + z^2))
    Z0)     Sum(z) Sum(xz)   Sum(yz)  Sum(z^2) ) ^ INV      Sum(z(x^2 + y^2 + z^2)))

    x0 = X0 / 2, y0 = Y0 / 2, z0 = Z0 / 2, r = sqrt(R + x0^2 + y0^2 + z0^2)
    */
    float n = static_cast<float>(val_imu_list.size());
    float sum_x = std::accumulate(val_imu_list.begin(), val_imu_list.end(), 0.0f, [](float sum, auto& val) { return sum + val.mag[0]; });
    float sum_y = std::accumulate(val_imu_list.begin(), val_imu_list.end(), 0.0f, [](float sum, auto& val) { return sum + val.mag[1]; });
    float sum_z = std::accumulate(val_imu_list.begin(), val_imu_list.end(), 0.0f, [](float sum, auto& val) { return sum + val.mag[2]; });
    float sum_x2 = std::accumulate(val_imu_list.begin(), val_imu_list.end(), 0.0f, [](float sum, auto& val) { return sum + val.mag[0] * val.mag[0]; });
    float sum_y2 = std::accumulate(val_imu_list.begin(), val_imu_list.end(), 0.0f, [](float sum, auto& val) { return sum + val.mag[1] * val.mag[1]; });
    float sum_z2 = std::accumulate(val_imu_list.begin(), val_imu_list.end(), 0.0f, [](float sum, auto& val) { return sum + val.mag[2] * val.mag[2]; });
    float sum_xy = std::accumulate(val_imu_list.begin(), val_imu_list.end(), 0.0f, [](float sum, auto& val) { return sum + val.mag[0] * val.mag[1]; });
    float sum_xz = std::accumulate(val_imu_list.begin(), val_imu_list.end(), 0.0f, [](float sum, auto& val) { return sum + val.mag[0] * val.mag[2]; });
    float sum_yz = std::accumulate(val_imu_list.begin(), val_imu_list.end(), 0.0f, [](float sum, auto& val) { return sum + val.mag[1] * val.mag[2]; });
    float sum_xyz = std::accumulate(val_imu_list.begin(), val_imu_list.end(), 0.0f, [](float sum, auto& val) { return sum + val.mag[0] * val.mag[0] + val.mag[1] * val.mag[1] + val.mag[2] * val.mag[2]; });
    float sum_x_xyz = std::accumulate(val_imu_list.begin(), val_imu_list.end(), 0.0f, [](float sum, auto& val) { return sum + val.mag[0] * (val.mag[0] * val.mag[0] + val.mag[1] * val.mag[1] + val.mag[2] * val.mag[2]); });
    float sum_y_xyz = std::accumulate(val_imu_list.begin(), val_imu_list.end(), 0.0f, [](float sum, auto& val) { return sum + val.mag[1] * (val.mag[0] * val.mag[0] + val.mag[1] * val.mag[1] + val.mag[2] * val.mag[2]); });
    float sum_z_xyz = std::accumulate(val_imu_list.begin(), val_imu_list.end(), 0.0f, [](float sum, auto& val) { return sum + val.mag[2] * (val.mag[0] * val.mag[0] + val.mag[1] * val.mag[1] + val.mag[2] * val.mag[2]); });

    cv::Mat LEFT = (cv::Mat_<float>(4, 4) <<
        n, sum_x, sum_y, sum_z,
        sum_x, sum_x2, sum_xy, sum_xz,
        sum_y, sum_xy, sum_y2, sum_yz,
        sum_z, sum_xz, sum_yz, sum_z2
        );
    cv::Mat RIGHT = (cv::Mat_<float>(4, 1) << sum_xyz, sum_x_xyz, sum_y_xyz, sum_z_xyz);
    cv::Mat ANS = LEFT.inv() * RIGHT;
    origin_.x = ANS.at<float>(1) / 2;
    origin_.y = ANS.at<float>(2) / 2;
    origin_.z = ANS.at<float>(3) / 2;
#endif
}

void LsmSphere::Clear()
{
    origin_.x = 0;
    origin_.y = 0;
    origin_.z = 0;
};

const cv::Point3f& LsmSphere::Get() const
{
    return origin_;
}
