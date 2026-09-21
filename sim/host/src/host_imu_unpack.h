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
/**
 * @file host_imu_unpack.h
 * @brief Turn a VSIM_FRAME_IMU payload into the SI sample the hub carries.
 *
 * The wire payload is 22 floats / 88 bytes (VSIM_IMU_PAYLOAD_BYTES) laid out
 * as acc, gyr, mag, acc_raw, gyr_raw, mag_compensated, mag_fusion, temp. That
 * happens to be the firmware's old internal IMU struct, because the frame was
 * originally produced by casting bytes straight onto it -- but it is now part
 * of the published SITL SDK contract that the ground station builds against,
 * so it stays exactly as it is.
 *
 * What changes is this side: the firmware's queues carry imu_sample_t now, so
 * the payload is unpacked field by field instead of memcpy'd onto a struct and
 * hoped for. The layout is written down here once rather than implied by
 * whatever the firmware's internal struct happens to be this month.
 */
#ifndef VAYU_HOST_IMU_UNPACK_H
#define VAYU_HOST_IMU_UNPACK_H

#include "hub/sample.h"

/** Number of floats in a VSIM_FRAME_IMU payload (88 bytes). */
#define HOST_IMU_WIRE_FLOATS 22

/* Field offsets within the payload, named so a wire change is a one-line edit
 * here rather than a hunt through three call sites. */
enum {
  HOST_IMU_W_ACC = 0,        /* [0..2]   m/s^2                */
  HOST_IMU_W_GYR = 3,        /* [3..5]   deg/s                */
  HOST_IMU_W_MAG = 6,        /* [6..8]   uT                   */
  HOST_IMU_W_MAG_FUSION = 18,/* [18..20] unit-normalised mag  */
  HOST_IMU_W_TEMP = 21       /* [21]     degrees Celsius      */
};

/**
 * Unpack one wire payload. Leaves `t_cyc` alone -- the caller owns the sim
 * clock and stamps the sample itself.
 */
static inline void host_imu_unpack(const float f[HOST_IMU_WIRE_FLOATS],
                                   imu_sample_t *out) {
  for (int i = 0; i < 3; i++) {
    out->acc[i] = f[HOST_IMU_W_ACC + i];
    out->gyr[i] = f[HOST_IMU_W_GYR + i];
    out->mag[i] = f[HOST_IMU_W_MAG + i];
  }
  out->temp_c = f[HOST_IMU_W_TEMP];
  /* The producer zeroes mag_fusion when the field is unusable, which is the
   * only health signal the wire carries; on hardware the driver decides this
   * from magnitude and disturbance checks it can actually make. */
  out->mag_valid = (f[HOST_IMU_W_MAG_FUSION + 0] != 0.0f ||
                    f[HOST_IMU_W_MAG_FUSION + 1] != 0.0f ||
                    f[HOST_IMU_W_MAG_FUSION + 2] != 0.0f);
  out->instance = 0;
}

#endif /* VAYU_HOST_IMU_UNPACK_H */
