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

void GraphScatter::Update(int32_t key, std::vector<cv::Point3f>& object_point_list)
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
}

