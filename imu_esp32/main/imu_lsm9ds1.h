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

esp_err_t imu_initialize(void);

int16_t imu_read_temperature();
void imu_read_gyro(int16_t& x, int16_t& y, int16_t& z);
void imu_read_acc(int16_t& x, int16_t& y, int16_t& z);
void imu_read_mag(int16_t& x, int16_t& y, int16_t& z);