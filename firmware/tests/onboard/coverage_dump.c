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
/* On-target gcov dump (C3). Built only with -DVAYU_HW_TEST_COV. */
#include "coverage_dump.h"

#ifdef VAYU_HW_TEST_COV

#include "memory.h"           /* v_malloc */
#include "storage/fs_owner.h" /* fs_owner write-at + vayu_log */
#include "vaios.h"            /* v_delay */

#include <stdint.h>
#include <string.h>

/* libgcov (arm-none-eabi-gcc 13): stream .gcda bytes through callbacks, no
 * fopen / filesystem. -fprofile-info-section drops each TU's gcov_info into the
 * .gcov_info section the linker brackets with these symbols (array of pointers). */
struct gcov_info;
extern const struct gcov_info *const __gcov_info_start[];
extern const struct gcov_info *const __gcov_info_end[];
extern void __gcov_info_to_gcda(const struct gcov_info *info,
                                void (*filename_fn)(const char *, void *),
                                void (*dump_fn)(const void *, unsigned, void *),
                                void *(*allocate_fn)(unsigned, void *),
                                void *arg);
extern void __gcov_filename_to_gcfn(const char *filename,
                                    void (*dump_fn)(const void *, unsigned,
                                                    void *),
                                    void *arg);

/* 8.3 name only: FatFS here is FF_USE_LFN=0, so the extension must be <=3 chars.
 * ".gcda" (4 chars) makes f_open return FR_INVALID_NAME (-6) and every write-at
 * fails -> empty file. Dump to "cov.gcd" on the card; the host renames it back to
 * cov.gcda (or feeds it straight to arm-none-eabi-gcov-tool merge-stream). */
#define COV_PATH "0:cov.gcd"
#define COV_STAGE 512u

static uint8_t s_stage[COV_STAGE];
static uint32_t s_stage_len;
static uint32_t s_off; /* committed bytes written to SD */

/* Flush the staging buffer to SD via the FS-owner write-at lane (session 0). */
static void flush_stage(void) {
  uint32_t o = 0;
  while (o < s_stage_len) {
    uint32_t n = s_stage_len - o;
    if (n > 200u) {
      n = 200u;
    }
    fs_owner_enqueue_write_at(0, COV_PATH, s_off, s_stage + o, n);
    s_off += n;
    o += n;
    for (int i = 0; i < 80 && fs_owner_writeat_pending(0); i++) {
      v_delay(5);
    }
  }
  s_stage_len = 0;
}

/* libgcov writes the .gcda byte stream here in small runs; stage + batch. */
static void dump_fn(const void *data, unsigned len, void *arg) {
  (void)arg;
  const uint8_t *p = (const uint8_t *)data;
  while (len) {
    uint32_t space = COV_STAGE - s_stage_len;
    uint32_t n = len < space ? len : space;
    memcpy(s_stage + s_stage_len, p, n);
    s_stage_len += n;
    p += n;
    len -= (unsigned)n;
    if (s_stage_len == COV_STAGE) {
      flush_stage();
    }
  }
}

/* Emit the per-TU gcfn (filename) record ahead of its gcda, for merge-stream. */
static void filename_fn(const char *filename, void *arg) {
  __gcov_filename_to_gcfn(filename, dump_fn, arg);
}

static void *allocate_fn(unsigned length, void *arg) {
  (void)arg;
  return v_malloc(length); /* one-shot dump; not freed (leak is fine here) */
}

void coverage_dump(void) {
  fs_owner_writeat_reset(0);
  fs_owner_truncate(COV_PATH);
  s_off = 0;
  s_stage_len = 0;

  const struct gcov_info *const *p = __gcov_info_start;
  for (; p != __gcov_info_end; p++) {
    if (*p) {
      __gcov_info_to_gcda(*p, filename_fn, dump_fn, allocate_fn, NULL);
    }
  }
  flush_stage();
  for (int i = 0; i < 500 && fs_owner_writeat_pending(0); i++) {
    v_delay(10);
  }
  vayu_log("[HWTEST] coverage dumped to %s: %u bytes", COV_PATH,
           (unsigned)s_off);
}

#else /* !VAYU_HW_TEST_COV */
void coverage_dump(void) {}
#endif
