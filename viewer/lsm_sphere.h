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
#ifndef LSM_SPHERE_
#define LSM_SPHERE_

/*** Include ***/
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <deque>

#include <opencv2/opencv.hpp>

#include "type.h"

class LsmSphere
{
public:
    LsmSphere() {};
    void Calculate(std::deque<ValImu>& val_imu_list);
    void Clear();
    const cv::Point3f& Get() const;
    
private:
    cv::Point3f origin_;
};

#endif