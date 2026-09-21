/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef VAYU_HOST_BARO_H
#define VAYU_HOST_BARO_H

/* Launch the SITL barometer feeder thread: reads modelled pressure frames from
 * vsim_d's /tmp/vsim_baro FIFO and injects them into the REAL firmware bme280
 * path (bme280_publish), so the FC's own altitude derivation + BARO telemetry
 * run in SITL exactly as on hardware. Mirrors host_imu_feeder_start(). */
void host_baro_start(void);

#endif /* VAYU_HOST_BARO_H */
