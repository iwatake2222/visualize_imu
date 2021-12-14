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
    static const std::string image_path = RESOURCE_DIR"/baboon.jpg";
    image_ = cv::imread(image_path);
}
 

GraphObject::~GraphObject()
{
    //
}

static void RotateMoveObject(std::vector<cv::Point3f>& object_point_list, float pitch, float yaw, float roll, float x, float y, float z)
{
    cv::Mat rvec_obj = (cv::Mat_<float>(3, 1) << Deg2Rad(pitch), Deg2Rad(yaw), Deg2Rad(roll));
    cv::Mat R_obj;
    cv::Rodrigues(rvec_obj, R_obj);

    for (auto& object_point : object_point_list) {
        cv::Mat p = (cv::Mat_<float>(3, 1) << object_point.x, object_point.y, object_point.z);
        p = R_obj * p;
        object_point.x = p.at<float>(0);
        object_point.y = p.at<float>(1);
        object_point.z = p.at<float>(2);
        object_point.x += x;
        object_point.y += y;
        object_point.z += z;
    }
}

void GraphObject::Update(int32_t key, float pitch, float yaw, float roll, float x, float y, float z)
{
    if (Graph::name_focused_ == this->name_) {
        TreatKeyInputMain(key);
    }

    cv::Mat mat = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(70, 70, 70));
    DrawAxes(mat);

    /*** Draw Image ***/
    std::vector<cv::Point3f> object_point_list;
    float aspect = static_cast<float>(image_.cols) / image_.rows;
    object_point_list.push_back(cv::Point3f(-1 * aspect, -1, 0));
    object_point_list.push_back(cv::Point3f(1 * aspect, -1, 0));
    object_point_list.push_back(cv::Point3f(1 * aspect, 1, 0));
    object_point_list.push_back(cv::Point3f(-1 * aspect, 1, 0));
    RotateMoveObject(object_point_list, pitch, yaw, roll, x, y, z);

    std::vector<cv::Point2f> image_point_list;
    cv::projectPoints(object_point_list, rvec_, tvec_, K_, cv::Mat(), image_point_list);
    /* Affine transform */
    cv::Point2f pts1[] = { cv::Point2f(0, 0), cv::Point2f(image_.cols - 1.0f, 0) , cv::Point2f(image_.cols - 1.0f, image_.rows - 1.0f) , cv::Point2f(0, image_.rows - 1.0f) };
    cv::Mat mat_affine = cv::getPerspectiveTransform(pts1, &image_point_list[0]);
    cv::warpPerspective(image_, mat, mat_affine, mat.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

    /*** Draw Axes ***/
    std::vector<cv::Point3f> object_axes_list = {
        cv::Point3f(-1, 0, 0),
        cv::Point3f(1, 0, 0),
        cv::Point3f(0, -1, 0),
        cv::Point3f(0, 1, 0),
        cv::Point3f(0, 0, -1),
        cv::Point3f(0, 0, 1),
    };
    RotateMoveObject(object_axes_list, pitch, yaw, roll, x, y, z);

    std::vector<cv::Point2f> image_axes_list;
    cv::projectPoints(object_axes_list, rvec_, tvec_, K_, cv::Mat(), image_axes_list);
    cv::arrowedLine(mat, image_axes_list[0], image_axes_list[1], cv::Scalar(128, 0, 0), 3);
    cv::arrowedLine(mat, image_axes_list[2], image_axes_list[3], cv::Scalar(0, 128, 0), 3);
    cv::arrowedLine(mat, image_axes_list[4], image_axes_list[5], cv::Scalar(0, 0, 128), 3);

    /*** Draw info ***/
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

