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

#include "graph_object.h"

/*** Macro ***/

/*** Global variable ***/


/*** Function ***/
static inline float Deg2Rad(float deg) { return static_cast<float>(deg * M_PI / 180.0); }

GraphObject::GraphObject(std::string name, float axis_size, int32_t width, int32_t height, int32_t focal_length)
    :Graph(name, axis_size, width, height, focal_length)
{
    //cv::setMouseCallback(name_, CallbackMouseHandler, this);
}
 

GraphObject::~GraphObject()
{
    //
}


void GraphObject::Update(int32_t key, float pitch, float yaw, float roll, float x, float y, float z)
{
    if (Graph::name_focused_ == this->name_) {
        TreatKeyInputMain(key);
    }

    cv::Mat mat = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(70, 70, 70));
    DrawAxes(mat);

    std::vector<cv::Point3f> object_axes_list = {
        cv::Point3f(-1, 0, 0),
        cv::Point3f(1, 0, 0),
        cv::Point3f(0, -1, 0),
        cv::Point3f(0, 1, 0),
        cv::Point3f(0, 0, -1),
        cv::Point3f(0, 0, 1),
    };

    cv::Mat rvec_obj = (cv::Mat_<float>(3, 1) << Deg2Rad(pitch), Deg2Rad(yaw), Deg2Rad(roll));
    cv::Mat R_obj;
    cv::Rodrigues(rvec_obj, R_obj);
    for (auto& object_point : object_axes_list) {
        cv::Mat p = (cv::Mat_<float>(3, 1) << object_point.x, object_point.y, object_point.z);
        p = R_obj * p;
        object_point.x = p.at<float>(0);
        object_point.y = p.at<float>(1);
        object_point.z = p.at<float>(2);
        object_point.x += x;
        object_point.y += y;
        object_point.z += z;
    }

    std::vector<cv::Point2f> image_axes_list;
    cv::projectPoints(object_axes_list, rvec_, tvec_, K_, cv::Mat(), image_axes_list);
    cv::arrowedLine(mat, image_axes_list[0], image_axes_list[1], cv::Scalar(128, 0, 0), 3);
    cv::arrowedLine(mat, image_axes_list[2], image_axes_list[3], cv::Scalar(0, 128, 0), 3);
    cv::arrowedLine(mat, image_axes_list[4], image_axes_list[5], cv::Scalar(0, 0, 128), 3);

    char text[64];
    snprintf(text, sizeof(text), "Pitch = %.1f", pitch);
    cv::putText(mat, text, cv::Point(10, 20), 0, 0.6, cv::Scalar(220, 220, 0));
    snprintf(text, sizeof(text), "Yaw = %.1f", yaw);
    cv::putText(mat, text, cv::Point(10, 40), 0, 0.6, cv::Scalar(220, 220, 0));
    snprintf(text, sizeof(text), "Roll = %.1f", roll);
    cv::putText(mat, text, cv::Point(10, 60), 0, 0.6, cv::Scalar(220, 220, 0));
    snprintf(text, sizeof(text), "X = %.3f", x);
    cv::putText(mat, text, cv::Point(10, 80), 0, 0.6, cv::Scalar(220, 220, 0));
    snprintf(text, sizeof(text), "Y = %.3f", y);
    cv::putText(mat, text, cv::Point(10, 100), 0, 0.6, cv::Scalar(220, 220, 0));
    snprintf(text, sizeof(text), "Z = %.3f", z);
    cv::putText(mat, text, cv::Point(10, 120), 0, 0.6, cv::Scalar(220, 220, 0));

    cv::imshow(name_, mat);
    cv::setMouseCallback(name_, CallbackMouseHandler, this);
}

