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
#ifndef VAYU_BATTERY_CALIB_H
#define VAYU_BATTERY_CALIB_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Persistence for the battery divider's calibration.
 *
 * The scale is not derivable: on this board the nominal 37k/4k ratio predicts
 * 0.0976 V/count and the bench measures 0.0114 -- 37% out, because PA0 is also
 * SYS_WKUP1 and something loads the low side. One constant absorbs the divider,
 * the resistor tolerances, the real VREF (the 3.3 V rail is a regulator, not a
 * reference) and any stray load.
 *
 * Which is exactly why it must not be compiled in. A calibration that lives in
 * a header is one that is wrong for every board but the one it was measured on,
 * and re-measuring means a rebuild and a reflash -- so in practice it does not
 * get re-measured, and the firmware quietly reports a voltage nobody has
 * checked. On the card it belongs to the AIRCRAFT, like its tune and its IMU
 * offsets, and the same binary is then correct on every board.
 *
 * TWO terms, not one. A pure scale assumes the line passes through the origin,
 * and it does not: the converter and the divider both contribute a zero error.
 * ArduPilot carries the same pair (_volt_offset then _volt_multiplier in
 * AP_BattMonitor_Analog) and PX4 carries v_div beside a current offset, for this
 * reason. Here the offset is in COUNTS, because that is where the error
 * physically is and because it falls straight out of a two-point fit:
 *
 *     volts = (counts - offset_counts) * volts_per_count
 *
 * Its own file, deliberately NOT a field in cal.bin: adding one there means
 * bumping CALIB_FILE_VERSION, which resets every sensor calibration on the card
 * and makes the operator re-level the board and re-do the gyro and mag. A
 * divider scale is not worth invalidating an IMU calibration for -- the same
 * argument hover.bin makes for staying out of pid.bin.
 */

/* 8.3 name -- FF_USE_LFN is 0, so a longer name fails vfs_open with -6. */
#define BATTERY_CALIB_PATH "0:batcal.bin"
#define BATTERY_CALIB_MAGIC 0x4C414342u /* 'B''C''A''L' */
#define BATTERY_CALIB_VERSION 1u

/*
 * Sanity bands. A plausible-looking file with a wild scale would mis-report the
 * pack by whatever factor it is wrong by -- and unlike a wild hover estimate,
 * nothing downstream would look odd enough to notice. So the bands are wide
 * enough for any sane divider and narrow enough to catch a corrupt float.
 *
 * At the pin one count is 3.3/4096 = 0.000806 V, so volts_per_count is that
 * times the divider ratio. The band spans ratios of roughly 1.2 (direct sense
 * of a 1S cell) to 62 (an 8S pack on a tall divider).
 */
#define BATTERY_CALIB_VPC_MIN 0.001f
#define BATTERY_CALIB_VPC_MAX 0.050f
/* A zero error beyond a few percent of full scale is a broken divider, not a
 * calibration -- past this the fit is describing a fault. */
#define BATTERY_CALIB_OFFSET_ABS_MAX 200.0f

/**
 * Load the persisted divider calibration.
 *
 * Call once, after the scheduler is up (fs_owner_read_at is the sanctioned
 * reader from that position). Both outputs are left UNTOUCHED when there is no
 * file, the magic or version is wrong, or either term is out of band -- so the
 * caller seeds them with the board defaults and a corrupt store degrades to
 * those rather than to a wild reading.
 *
 * @return true when the stored values were applied.
 */
bool battery_calib_load(float *volts_per_count, float *offset_counts);

/**
 * Queue a save through fs_owner. Rejects out-of-band values rather than
 * persisting something the loader would refuse.
 */
bool battery_calib_save(float volts_per_count, float offset_counts);

#endif /* VAYU_BATTERY_CALIB_H */
