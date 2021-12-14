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
#ifndef GRAPH_OBJECT_
#define GRAPH_OBJECT_

/*** Include ***/
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <opencv2/opencv.hpp>

#include "graph.h"

class GraphObject : public Graph
{
public:
    GraphObject() {};
    GraphObject(std::string name, float axis_size = 1.0, int32_t width = 400, int32_t height = 400, int32_t focal_length = 400);
    virtual ~GraphObject();
    void Update(int32_t key, float pitch, float yaw, float roll, float x, float y, float z);
private:
    cv::Mat image_;
};

#endif
