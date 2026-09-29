# Copyright (C) 2026 NAVRobotec Pvt Ltd
# Author: Ragnar Vallhala
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Which sensor backends this board carries.
#
# Each name is a file in firmware/src/driver/sensor/. Only the ones listed are
# compiled; the rest contribute no flash, no RAM and no entry in the
# vayu_sensors registry, because they are never built. Nothing in the firmware
# names these drivers -- they describe themselves (sensor/sensor.h), so this
# list is the ONLY place the choice is made.
#
# Swapping the IMU is editing one word here, once a driver for the new part
# exists beside the old one.
set(BOARD_SENSOR_DRIVERS
    bmx160  # IMU:   gyro/accel/mag, owns the I2C loop
    bme280  # baro:  pressure/temp/humidity, rides the IMU's loop
    vl53l0x # range: downward ToF, rides the IMU's loop
)
