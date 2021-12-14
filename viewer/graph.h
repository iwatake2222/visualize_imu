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
#ifndef GRAPH_
#define GRAPH_

/*** Include ***/
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <opencv2/opencv.hpp>

class Graph
{
public:
    Graph() : width_(0), height_(0), kIncPosPerFrame(0), axis_size_(0) {};
    Graph(std::string name, float axis_size = 1.0, int32_t width = 400, int32_t height = 400, int32_t focal_length = 400);
    virtual ~Graph();
    void DrawAxes(cv::Mat& mat);
    void Update(int32_t key);
    void RotateAxis(float dpitch_deg, float dyaw_deg, float droll_deg);
    void MoveCameraPos(float dtx, float dty, float dtz, bool is_on_world = true);    /* Oc - Ow */
    void TreatKeyInputMain(int32_t key);
    void CallbackMouse(int32_t event, int32_t x, int32_t y, int32_t flags);
    void SetViewOnX();
    void SetViewOnY();
    void SetViewOnZ();

protected:
    static void CallbackMouseHandler(int32_t event, int32_t x, int32_t y, int32_t flags, void* userdata);
    static std::string name_focused_;

public:
    std::string name_;
    int32_t width_;
    int32_t height_;
    cv::Mat K_;
    cv::Mat rvec_;
    cv::Mat tvec_;
    std::vector<cv::Point3f> object_axes_list_;
    cv::Point drag_previous_point_;
    float kIncPosPerFrame;
    float axis_size_;
};

#endif
