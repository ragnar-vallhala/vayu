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
#include "storage/battery_calib.h"

#include "storage/fs_owner.h"

#include <stddef.h> /* NULL */

/* 16 bytes. Explicit version and padding so the struct is the same shape on the
 * host tool that writes it as on the target that reads it -- the file is a wire
 * format between two programs, not a private dump. */
typedef struct {
  uint32_t magic;
  uint16_t version;
  uint16_t _pad;
  float volts_per_count;
  float offset_counts;
} battery_calib_store_t;

/* A NaN fails every comparison, so bounds have to be written as "is inside",
 * never as "is not outside" -- !(x > lo) catches NaN where (x <= lo) does not.
 * A NaN scale would otherwise sail through and make every reading NaN. */
static bool in_band(float vpc, float offset) {
  return (vpc > BATTERY_CALIB_VPC_MIN) && (vpc < BATTERY_CALIB_VPC_MAX) &&
         (offset > -BATTERY_CALIB_OFFSET_ABS_MAX) &&
         (offset < BATTERY_CALIB_OFFSET_ABS_MAX);
}

/** @noreq Load the persisted divider calibration. */
bool battery_calib_load(float *volts_per_count, float *offset_counts) {
  if (volts_per_count == NULL || offset_counts == NULL) {
    return false;
  }
  battery_calib_store_t s = {0};
  /* Through fs_owner, not vfs_* directly: this runs at the top of battery_task,
   * after the scheduler, so concurrently with the FS task and whatever it has
   * open. A missing card or file comes back <0 and the caller's defaults stand. */
  const int n = fs_owner_read_at(BATTERY_CALIB_PATH, 0, &s, sizeof s);

  if (n != (int)sizeof s || s.magic != BATTERY_CALIB_MAGIC ||
      s.version != BATTERY_CALIB_VERSION) {
    vayu_log("batcal: absent/bad header, using compiled %d uV/count",
             (int)(*volts_per_count * 1000000.0f));
    return false;
  }
  if (!in_band(s.volts_per_count, s.offset_counts)) {
    vayu_log("batcal: stored %d uV/count ofs %d out of band, keeping compiled",
             (int)(s.volts_per_count * 1000000.0f), (int)s.offset_counts);
    return false;
  }
  *volts_per_count = s.volts_per_count;
  *offset_counts = s.offset_counts;
  vayu_log("batcal: %d uV/count, offset %d counts from SD",
           (int)(s.volts_per_count * 1000000.0f), (int)s.offset_counts);
  return true;
}

/** @noreq Queue a calibration save through the FS owner. */
bool battery_calib_save(float volts_per_count, float offset_counts) {
  if (!in_band(volts_per_count, offset_counts)) {
    return false;
  }
  const battery_calib_store_t s = {BATTERY_CALIB_MAGIC, BATTERY_CALIB_VERSION,
                                   0u, volts_per_count, offset_counts};
  /* Internal slot, never session 0: a write here during a GCS upload would
   * corrupt that transfer's pending/committed accounting. */
  return fs_owner_enqueue_write_at(FS_WA_SESSION_INTERNAL, BATTERY_CALIB_PATH,
                                   0, &s, sizeof s);
}
