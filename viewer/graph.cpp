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

#include "graph.h"

/*** Macro ***/
static constexpr int32_t kInvalidValue = -99999;

/*** Global variable ***/
std::string Graph::name_focused_;

/*** Function ***/
static inline float Deg2Rad(float deg) { return static_cast<float>(deg * M_PI / 180.0); }
static inline float Rad2Deg(float rad) { return static_cast<float>(rad * 180.0 / M_PI); }

Graph::Graph(std::string name, float axis_size, int32_t width, int32_t height, int32_t focal_length)
{
    axis_size_ = axis_size;
    kIncPosPerFrame = 0.1f * axis_size;
    drag_previous_point_ = { kInvalidValue, kInvalidValue };
    name_ = name;
    width_ = width;
    height_ = height;
    K_ = (cv::Mat_<float>(3, 3) <<
        focal_length, 0, width / 2.f,
        0, focal_length, height / 2.f,
        0, 0, 1);
    rvec_ = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    cv::Mat T = (cv::Mat_<float>(3, 1) << 2 * axis_size, -2 * axis_size, -2 * axis_size);

    cv::Mat R;
    cv::Rodrigues(rvec_, R);
    tvec_ = -R * T;

    object_axes_list_ = {
        cv::Point3f(-1, 0, 0) * axis_size,
        cv::Point3f(1, 0, 0) * axis_size,
        cv::Point3f(0, -1, 0) * axis_size,
        cv::Point3f(0, 1, 0) * axis_size,
        cv::Point3f(0, 0, -1) * axis_size,
        cv::Point3f(0, 0, 1) * axis_size,
    };

    RotateCameraAngle(40, 40, 0);

    cv::namedWindow(name_);
    cv::setMouseCallback(name_, CallbackMouseHandler, this);
}

Graph::~Graph()
{
    //
}

void Graph::DrawAxes(cv::Mat& mat)
{
    std::vector<cv::Point2f> image_axes_list;
    cv::projectPoints(object_axes_list_, rvec_, tvec_, K_, cv::Mat(), image_axes_list);
    cv::arrowedLine(mat, image_axes_list[0], image_axes_list[1], cv::Scalar(255, 0, 0));
    cv::arrowedLine(mat, image_axes_list[2], image_axes_list[3], cv::Scalar(0, 255, 0));
    cv::arrowedLine(mat, image_axes_list[4], image_axes_list[5], cv::Scalar(0, 0, 255));
    cv::putText(mat, "X", image_axes_list[1], 0, 1.0, cv::Scalar(255, 0, 0));
    cv::putText(mat, "Y", image_axes_list[3], 0, 1.0, cv::Scalar(0, 255, 0));
    cv::putText(mat, "Z", image_axes_list[5], 0, 1.0, cv::Scalar(0, 0, 255));

}

void Graph::Update(int32_t key)
{
    if (name_focused_ == this->name_) {
        TreatKeyInputMain(key);
    }

    cv::Mat mat = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(70, 70, 70));
    DrawAxes(mat);
    cv::imshow(name_, mat);
}


void Graph::RotateCameraAngle(float dpitch_deg, float dyaw_deg, float droll_deg)
{
    cv::Mat R_old;
    cv::Rodrigues(rvec_, R_old);
    cv::Mat T = -R_old.inv() * tvec_;

    cv::Mat rvec_delta = (cv::Mat_<float>(3, 1) << Deg2Rad(dpitch_deg), Deg2Rad(dyaw_deg), Deg2Rad(droll_deg));
    cv::Mat R_delta;
    cv::Rodrigues(rvec_delta, R_delta);

    cv::Mat R_new = R_delta * R_old;
    cv::Rodrigues(R_new, rvec_);

    tvec_ = -R_new * T;
}

void Graph::MoveCameraPos(float dtx, float dty, float dtz, bool is_on_world)    /* Oc - Ow */
{
    cv::Mat tvec_delta = (cv::Mat_<float>(3, 1) << dtx, dty, dtz);
    if (is_on_world) {
        cv::Mat R;
        cv::Rodrigues(rvec_, R);
        tvec_delta = -R * tvec_delta;
    } else {
        tvec_delta *= -1;
    }
    tvec_ += tvec_delta;

}

void Graph::TreatKeyInputMain(int32_t key)
{   
    key &= 0xFF;
    switch (key) {
    case 'w':
        MoveCameraPos(0, 0, kIncPosPerFrame, false);
        break;
    case 'W':
        MoveCameraPos(0, 0, kIncPosPerFrame, true);
        break;
    case 's':
        MoveCameraPos(0, 0, -kIncPosPerFrame, false);
        break;
    case 'S':
        MoveCameraPos(0, 0, -kIncPosPerFrame, true);
        break;
    case 'a':
        MoveCameraPos(-kIncPosPerFrame, 0, 0, false);
        break;
    case 'A':
        MoveCameraPos(-kIncPosPerFrame, 0, 0, true);
        break;
    case 'd':
        MoveCameraPos(kIncPosPerFrame, 0, 0, false);
        break;
    case 'D':
        MoveCameraPos(kIncPosPerFrame, 0, 0, true);
        break;
    case 'z':
        MoveCameraPos(0, -kIncPosPerFrame, 0, false);
        break;
    case 'Z':
        MoveCameraPos(0, -kIncPosPerFrame, 0, true);
        break;
    case 'x':
        MoveCameraPos(0, kIncPosPerFrame, 0, false);
        break;
    case 'X':
        MoveCameraPos(0, kIncPosPerFrame, 0, true);
        break;
    case 'q':
        RotateCameraAngle(0, 0, 2.0f);
        break;
    case 'e':
        RotateCameraAngle(0, 0, -2.0f);
        break;
    }
}

void Graph::CallbackMouse(int32_t event, int32_t x, int32_t y, int32_t flags)
{
    static constexpr float kIncAnglePerPx = 0.1f;

    static cv::Point drag_previous_point_ = { kInvalidValue, kInvalidValue };
    if (event == cv::EVENT_LBUTTONUP) {
        drag_previous_point_.x = kInvalidValue;
        drag_previous_point_.y = kInvalidValue;
    } else if (event == cv::EVENT_LBUTTONDOWN) {
        drag_previous_point_.x = x;
        drag_previous_point_.y = y;
    } else {
        if (drag_previous_point_.x != kInvalidValue) {
            float delta_yaw = kIncAnglePerPx * (x - drag_previous_point_.x);
            float pitch_delta = -kIncAnglePerPx * (y - drag_previous_point_.y);
            RotateCameraAngle(pitch_delta, delta_yaw, 0);
            drag_previous_point_.x = x;
            drag_previous_point_.y = y;
        }
    }
}


void Graph::CallbackMouseHandler(int32_t event, int32_t x, int32_t y, int32_t flags, void* userdata)
{
    Graph* p = (Graph*)userdata;
    name_focused_ = p->name_;
    p->CallbackMouse(event, x, y, flags);
}
