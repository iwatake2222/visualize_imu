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

#include <opencv2/opencv.hpp>

#include "graph_scatter.h"

/*** Macro ***/

/*** Global variable ***/


/*** Function ***/


GraphScatter::GraphScatter(std::string name, float axis_size, int32_t width, int32_t height, int32_t focal_length)
    :Graph(name, axis_size, width, height, focal_length)
{
    //cv::setMouseCallback(name_, CallbackMouseHandler, this);
}
 

GraphScatter::~GraphScatter()
{
    //
}

static uint8_t normalize_255(float val, float axis_size)
{
    val += axis_size;   /* [-1,1] -> [0, 2] */
    val /= axis_size;   /* [0, 2] -> [0, 1] */
    val *= 255.0;   /* [0, 1] -> [0, 255] */
    return static_cast<uint8_t>((std::min)(255.0f, (std::max)(0.0f, val)));

}

void GraphScatter::Update(int32_t key, const std::vector<cv::Point3f>& object_point_list)
{
    if (Graph::name_focused_ == this->name_) {
        TreatKeyInputMain(key);
    }

    cv::Mat mat = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(70, 70, 70));
    DrawAxes(mat);

    std::vector<cv::Point2f> image_point_list;
    if (object_point_list.size() > 0) {
        cv::projectPoints(object_point_list, rvec_, tvec_, K_, cv::Mat(), image_point_list);
    }

    for (int32_t i = 0; i < image_point_list.size(); i++) {
        cv::circle(mat, image_point_list[i], 1, cv::Scalar(normalize_255(object_point_list[i].x, axis_size_), normalize_255(object_point_list[i].y, axis_size_), normalize_255(object_point_list[i].z, axis_size_)), -1);
    }

    cv::imshow(name_, mat);
    cv::setMouseCallback(name_, CallbackMouseHandler, this);
}



GraphScatterImu::GraphScatterImu(float axis_size_acc, float axis_size_gyro, float axis_size_mag)
{
    graph_scatter_acc = GraphScatter("Acc", axis_size_acc);
    graph_scatter_acc_x = GraphScatter("Acc_x", axis_size_acc);
    graph_scatter_acc_y = GraphScatter("Acc_y", axis_size_acc);
    graph_scatter_acc_z = GraphScatter("Acc_z", axis_size_acc);
    graph_scatter_acc_x.SetViewOnX();
    graph_scatter_acc_y.SetViewOnY();
    graph_scatter_acc_z.SetViewOnZ();

    graph_scatter_gyro = GraphScatter("Gyro", axis_size_gyro);
    graph_scatter_gyro_x = GraphScatter("Gyro_x", axis_size_gyro);
    graph_scatter_gyro_y = GraphScatter("Gyro_y", axis_size_gyro);
    graph_scatter_gyro_z = GraphScatter("Gyro_z", axis_size_gyro);
    graph_scatter_gyro_x.SetViewOnX();
    graph_scatter_gyro_y.SetViewOnY();
    graph_scatter_gyro_z.SetViewOnZ();

    graph_scatter_mag = GraphScatter("Mag", axis_size_mag);
    graph_scatter_mag_x = GraphScatter("Mag_x", axis_size_mag);
    graph_scatter_mag_y = GraphScatter("Mag_y", axis_size_mag);
    graph_scatter_mag_z = GraphScatter("Mag_z", axis_size_mag);
    graph_scatter_mag_x.SetViewOnX();
    graph_scatter_mag_y.SetViewOnY();
    graph_scatter_mag_z.SetViewOnZ();
}


void GraphScatterImu::Update(int32_t key, const std::vector<cv::Point3f>& acc_list, const std::vector<cv::Point3f>& gyro_list, const std::vector<cv::Point3f>& mag_list)
{
    static constexpr int32_t kHeightTitle = 30;
    auto rect_acc = cv::getWindowImageRect("Acc");
    cv::moveWindow("Acc_x", rect_acc.x + rect_acc.width, rect_acc.y - kHeightTitle);
    cv::moveWindow("Acc_y", rect_acc.x, rect_acc.y + rect_acc.height);
    cv::moveWindow("Acc_z", rect_acc.x + rect_acc.width, rect_acc.y + rect_acc.height);

    auto rect_gyro = cv::getWindowImageRect("Gyro");
    cv::moveWindow("Gyro_x", rect_gyro.x + rect_gyro.width, rect_gyro.y - kHeightTitle);
    cv::moveWindow("Gyro_y", rect_gyro.x, rect_gyro.y + rect_gyro.height);
    cv::moveWindow("Gyro_z", rect_gyro.x + rect_gyro.width, rect_gyro.y + rect_gyro.height);

    auto rect_mag = cv::getWindowImageRect("Mag");
    cv::moveWindow("Mag_x", rect_mag.x + rect_mag.width, rect_mag.y - kHeightTitle);
    cv::moveWindow("Mag_y", rect_mag.x, rect_mag.y + rect_mag.height);
    cv::moveWindow("Mag_z", rect_mag.x + rect_mag.width, rect_mag.y + rect_mag.height);

    graph_scatter_acc.Update(key, acc_list);
    graph_scatter_acc_x.Update(key, acc_list);
    graph_scatter_acc_y.Update(key, acc_list);
    graph_scatter_acc_z.Update(key, acc_list);

    graph_scatter_gyro.Update(key, gyro_list);
    graph_scatter_gyro_x.Update(key, gyro_list);
    graph_scatter_gyro_y.Update(key, gyro_list);
    graph_scatter_gyro_z.Update(key, gyro_list);

    graph_scatter_mag.Update(key, mag_list);
    graph_scatter_mag_x.Update(key, mag_list);
    graph_scatter_mag_y.Update(key, mag_list);
    graph_scatter_mag_z.Update(key, mag_list);
}
