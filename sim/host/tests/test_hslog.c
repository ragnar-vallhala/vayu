/**
 * @file sim/host/tests/test_hslog.c
 * @brief Wire-format verification for the high-speed IMU stream (HSL).
 *
 * Drives the REAL firmware encoder (src/storage/imu_hs_log.c) against the host
 * VFS and asserts the bytes it lays down, because the encoder's only consumer
 * is an offline decoder in another language -- nothing at runtime would ever
 * notice a layout mistake.
 *
 * Under VAYU_SIM the ring is 63 slots, so two armed sessions of four streams
 * overrun it and it actually WRAPS. That is the case worth testing: slot order
 * stops being time order, and a session's own SESSION frame can be overwritten
 * while its blocks are still live.
 *
 * Structural claims checked here (see include/storage/imu_hs_log.h):
 *   - the preamble is exactly one sector and declares ALL THREE streams
 *   - every frame declares its own length, so an unknown type is skippable
 *   - every ring frame carries the sentinel, the seq and the session tag that
 *     let a reader undo the rotation and split sessions
 *   - the decimated streams land at their stated rates (cycle-stamp decimation)
 *   - state changes come out as EVENT frames
 *   - the cursor hint survives for the next boot, and resuming clears the
 *     seq margin that keeps orphaned sectors distinguishable
 *
 * The file it leaves behind is decoded by tools/telemetry/hslog.py in the
 * `hslog_decode` ctest -- that pair is what pins the C writer and the Python
 * reader to the same format.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "storage/imu_hs_log.h"
#include "sys/state.h"
#include "variables.h"
#include "vfs.h"

static int g_checks = 0;
static int g_fails = 0;

#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    g_checks++;                                                                \
    if (cond) {                                                                \
      printf("    ok   %s\n", (msg));                                          \
    } else {                                                                   \
      g_fails++;                                                               \
      printf("    FAIL %s   (%s:%d)\n", (msg), __FILE__, __LINE__);            \
    }                                                                          \
  } while (0)

static uint16_t rd16(const uint8_t *p) {
  return (uint16_t)(p[0] | (p[1] << 8));
}
static uint32_t rd32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

#define RING_SLOTS                                                             \
  ((uint32_t)(HSL_FILE_SIZE / HSL_SECTOR_BYTES) - HSL_PREAMBLE_SECTORS)
#define CYC_PER_SAMPLE (84000000u / 2000u)
/* Values the decoder re-checks; distinct per motor so a mis-ordered field
 * cannot pass. */
static const float MOTORS[4] = {0.1f, 0.2f, 0.3f, 0.4f};
#define THROTTLE 0.5f

/* One armed period: N IMU samples, with act/vrt offered at the SAME rate so
 * the module's own cycle-stamp decimation is what produces 400/20 Hz. That is
 * exactly how the firmware calls them. */
static void run_session(int n_samples) {
  _system_current_status = SYSTEM_STATE_ARMED;
  imu_hs_log_drain(); /* opens the file, emits SESSION + the state EVENT */

  for (int i = 0; i < n_samples; i++) {
    const uint32_t t = (uint32_t)i * CYC_PER_SAMPLE;
    int16_t g[3] = {(int16_t)i, 0, 0};
    int16_t a[3] = {0, 0, 0};
    imu_hs_log_sample(g, a, t);
    imu_hs_log_act(MOTORS, THROTTLE, HSL_ACT_F_ARMED, t);
    hsl_vert_sample_t v = {.baro_altitude = 1.5f,
                           .agl = 2.5f,
                           .agl_tof = 3.5f,
                           .altitude = 4.5f,
                           .climb_rate = 5.5f,
                           .accel_bias = 6.5f,
                           .flags = HSL_VRT_F_VALID | HSL_VRT_F_TOF_VALID};
    imu_hs_log_vert(&v, t);
    /* "ctl" is undecimated, so unlike act/vrt the caller sets its rate. The
     * firmware calls it from the 1 kHz rate loop; halving the 2 kHz sample
     * rate here reproduces that relationship. */
    if ((i % 2) == 0) {
      imu_hs_log_ctl((const float[3]){10.0f, -20.0f, 30.0f},
                     (const float[3]){0.25f, -0.5f, 0.125f}, t);
    }
    if ((i % 41) == 40) {
      imu_hs_log_drain(); /* the FS task runs far more often than this */
    }
  }
  _system_current_status = SYSTEM_STATE_STANDBY;
  imu_hs_log_drain(); /* flushes partial sectors, commits the cursor hint */
}

int main(void) {
  printf("== HSL wire-format verification ==\n");
  setenv("VAYU_VFS_DIR", "/tmp/vayu_hslog_test", 1);
  remove("/tmp/vayu_hslog_test/0_imuhs.bin"); /* isolate from a previous run */

  imu_hs_log_set_scale(0.0610351562f, 0.0047884034f); /* 2000 dps, 16 g */
  imu_hs_log_boot_init();

  _system_current_status = SYSTEM_STATE_STANDBY;
  imu_hs_log_sample((int16_t[3]){1, 2, 3}, (int16_t[3]){4, 5, 6}, 0);
  imu_hs_log_drain();
  CHECK(!imu_hs_log_active(), "disarmed: nothing recorded");
  CHECK(imu_hs_log_wraps() == 0, "fresh file starts unwrapped");

  run_session(860);
  CHECK(imu_hs_log_wraps() == 0, "one session fits without wrapping");
  run_session(860); /* pushes past 63 slots: must wrap */
  CHECK(imu_hs_log_wraps() == 1u, "second session wraps the ring");
  CHECK(imu_hs_log_dropped() == 0u, "no sectors dropped when drained promptly");
  CHECK(!imu_hs_log_active(), "disarm closes the recording");

  /* ---- read it back ---------------------------------------------------- */
  static uint8_t buf[HSL_FILE_SIZE];
  vfs_fd_t fd = vfs_open(HSL_FILENAME, VFS_O_RDONLY);
  int n = (fd < 0) ? -1 : vfs_read(fd, buf, sizeof(buf));
  if (fd >= 0) {
    vfs_close(fd);
  }
  CHECK(n == (int)HSL_FILE_SIZE, "file is its full preallocated size");
  if (n < (int)HSL_SECTOR_BYTES) {
    printf("\n%d checks, %d failures\n", g_checks, ++g_fails);
    return 1;
  }

  CHECK(rd32(&buf[0]) == HSL_MAGIC, "file header magic");
  CHECK(rd16(&buf[4]) == HSL_VERSION, "file header version");
  CHECK(rd16(&buf[6]) == HSL_FILE_HDR_BYTES, "file header declares its length");
  CHECK(rd32(&buf[8]) == 84000000u, "clock_hz recorded");
  CHECK(rd32(&buf[12]) == HSL_PREAMBLE_BYTES, "ring starts after the preamble");
  CHECK(rd32(&buf[16]) == RING_SLOTS, "ring size recorded");
  CHECK(rd32(&buf[20]) == imu_hs_log_head_slot(), "cursor hint persisted");
  CHECK(rd32(&buf[28]) == 1u, "wrap count persisted");

  /* Walk the preamble by length alone -- the property that makes an unknown
   * frame type skippable -- and collect the FMTs. */
  uint32_t off = HSL_FILE_HDR_BYTES;
  int n_fmt = 0, pad_seen = 0;
  uint8_t fmt_stream[4] = {0};
  uint16_t fmt_rate[4] = {0};
  uint8_t fmt_recb[4] = {0};
  while (off + HSL_FRAME_HDR_BYTES <= HSL_PREAMBLE_BYTES) {
    const uint8_t *f = &buf[off];
    const uint16_t len = rd16(&f[2]);
    if (f[0] == HSL_TYPE_FMT && n_fmt < 4) {
      fmt_stream[n_fmt] = f[4];
      fmt_recb[n_fmt] = f[5];
      fmt_rate[n_fmt] = rd16(&f[6]);
      n_fmt++;
    } else if (f[0] == HSL_TYPE_PAD) {
      pad_seen = 1;
      off += HSL_FRAME_HDR_BYTES + len;
      break;
    }
    off += HSL_FRAME_HDR_BYTES + len;
  }
  CHECK(n_fmt == 4, "preamble declares all four streams");
  CHECK(pad_seen, "preamble is padded");
  CHECK(off == HSL_PREAMBLE_BYTES, "preamble frames fill the preamble exactly");
  CHECK(fmt_stream[0] == HSL_STREAM_IMU && fmt_recb[0] == HSL_IMU_REC_BYTES &&
            fmt_rate[0] == 2000,
        "FMT[imu] 12 B @ 2000 Hz");
  CHECK(fmt_stream[1] == HSL_STREAM_ACT && fmt_recb[1] == HSL_ACT_REC_BYTES &&
            fmt_rate[1] == HSL_ACT_RATE_HZ,
        "FMT[act] 12 B @ 400 Hz");
  CHECK(fmt_stream[2] == HSL_STREAM_VRT && fmt_recb[2] == HSL_VRT_REC_BYTES &&
            fmt_rate[2] == HSL_VRT_RATE_HZ,
        "FMT[vrt] 28 B @ 20 Hz");
  /* The notch centre rides in the u16 that used to be padding, so the record
   * size and the field count must NOT have moved -- the preamble has 16 B
   * spare and one more FMT field would need 16 of them plus a new frame. */
  CHECK(HSL_VRT_REC_BYTES == 28u, "vrt record is unchanged at 28 B");
  CHECK(fmt_stream[3] == HSL_STREAM_CTL && fmt_recb[3] == HSL_CTL_REC_BYTES &&
            fmt_rate[3] == HSL_CTL_RATE_HZ,
        "FMT[ctl] 12 B @ 1000 Hz");

  /* The vrt records must carry the notch axis round-robin: a log that cannot
   * say WHICH axis a centre belongs to cannot be read back. */
  {
    int seen_axis[3] = {0, 0, 0};
    for (uint32_t i = 0; i < RING_SLOTS; i++) {
      const uint8_t *f = &buf[HSL_PREAMBLE_BYTES + HSL_SECTOR_BYTES * i];
      if (f[0] != HSL_TYPE_BLOCK || f[4] != HSL_STREAM_VRT)
        continue;
      uint16_t n = rd16(&f[6]);
      for (uint16_t k = 0; k < n; k++) {
        const uint8_t *r = f + HSL_FRAME_HDR_BYTES + HSL_BLOCK_HDR_BYTES +
                           (size_t)k * HSL_VRT_REC_BYTES;
        uint16_t fl = rd16(&r[24]);
        uint8_t ax = (uint8_t)((fl & HSL_VRT_F_NOTCH_AXIS_MASK) >>
                               HSL_VRT_F_NOTCH_AXIS_SHIFT);
        if (ax < 3u)
          seen_axis[ax]++;
      }
    }
    CHECK(seen_axis[0] > 0 && seen_axis[1] > 0 && seen_axis[2] > 0,
          "vrt records cycle the notch centre through all three axes");
  }

  /* Every ring slot: sentinel, sector-length frame, a known type, a seq. */
  static uint32_t seq[RING_SLOTS];
  static uint8_t tags[RING_SLOTS];
  int bad = -1, n_ev = 0;
  int per_stream[5] = {0};
  for (uint32_t i = 0; i < RING_SLOTS; i++) {
    const uint8_t *fr =
        &buf[(size_t)HSL_PREAMBLE_BYTES + (size_t)HSL_SECTOR_BYTES * i];
    if (fr[1] != HSL_RING_SENTINEL ||
        rd16(&fr[2]) != HSL_SECTOR_BYTES - HSL_FRAME_HDR_BYTES) {
      bad = (int)i;
      break;
    }
    const uint8_t *p = &fr[HSL_FRAME_HDR_BYTES];
    seq[i] = rd32(&p[4]);
    tags[i] = p[1];
    switch (fr[0]) {
    case HSL_TYPE_SESSION:
      /* Not counted: this scan runs after a deliberate wrap, which overwrites
       * an unknown number of SESSION frames. Only the type is checkable here. */
      break;
    case HSL_TYPE_EVENT:
      n_ev++;
      break;
    case HSL_TYPE_BLOCK:
      if (p[0] < 5) {
        per_stream[p[0]]++;
      }
      break;
    default:
      bad = (int)i;
      break;
    }
    if (bad >= 0) {
      break;
    }
  }
  CHECK(bad < 0, "every ring slot carries the sentinel and a known frame type");
  CHECK(n_ev > 0, "state changes produced EVENT frames");
  CHECK(per_stream[HSL_STREAM_IMU] > per_stream[HSL_STREAM_CTL] &&
            per_stream[HSL_STREAM_CTL] > per_stream[HSL_STREAM_ACT] &&
            per_stream[HSL_STREAM_ACT] > per_stream[HSL_STREAM_VRT],
        "sector counts follow the four streams' rates");

  /* The ring wrapped, so exactly one slot boundary goes backwards in seq --
   * that boundary is where the newest data abuts the oldest. */
  int descents = 0;
  uint32_t rot = 0; /* slot holding the OLDEST frame */
  for (uint32_t i = 0; i + 1 < RING_SLOTS; i++) {
    if (seq[i + 1] < seq[i]) {
      descents++;
      rot = i + 1;
    }
  }
  CHECK(descents == 1, "wrapped ring has exactly one seq discontinuity");
  CHECK(rot != 0, "oldest frame is not at slot 0, so slot order != time order");

  /* Across that seam two adjacent slots belong to different armed periods and
   * NOTHING but the session tag says so. This is why the tag exists. */
  /* Guarded: CHECK records a failure and carries on, so without this a rot of 0
   * indexes tags[-1] and takes the whole suite down with a segfault instead of
   * reporting which property broke. */
  CHECK(rot > 0 && tags[rot - 1] != tags[rot],
        "session tag separates the two laps where they abut");

  /* ---- simulated reboot ------------------------------------------------ */
  uint32_t max_seq = 0;
  for (uint32_t i = 0; i < RING_SLOTS; i++) {
    if (seq[i] > max_seq) {
      max_seq = seq[i];
    }
  }
  const uint32_t slot_before = imu_hs_log_head_slot();
  const uint32_t wraps_before = imu_hs_log_wraps();
  imu_hs_log_boot_init();
  CHECK(imu_hs_log_head_slot() == slot_before, "boot recovers the ring slot");
  CHECK(imu_hs_log_wraps() == wraps_before, "boot recovers the wrap count");

  run_session(860);
  fd = vfs_open(HSL_FILENAME, VFS_O_RDONLY);
  n = (fd < 0) ? -1 : vfs_read(fd, buf, sizeof(buf));
  if (fd >= 0) {
    vfs_close(fd);
  }
  uint32_t resumed = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < RING_SLOTS && n > 0; i++) {
    const uint8_t *fr =
        &buf[(size_t)HSL_PREAMBLE_BYTES + (size_t)HSL_SECTOR_BYTES * i];
    uint32_t sq = rd32(&fr[HSL_FRAME_HDR_BYTES + 4]);
    if (sq > max_seq && sq < resumed) {
      resumed = sq;
    }
  }
  CHECK(resumed > max_seq + HSL_HDR_SYNC_FRAMES,
        "resumed seq clears any sector orphaned by a power cut");

  printf("\n%d checks, %d failures\n", g_checks, g_fails);
  return g_fails == 0 ? 0 : 1;
}
