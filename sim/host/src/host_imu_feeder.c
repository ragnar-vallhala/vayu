/*
 * host_imu_feeder.c -- read IMU samples from /tmp/vsim_imu (binary
 * vsim_imu_frame_t per the vsim_proto wire spec), push them into
 * vayu's imu_queue, run mahony to derive attitude, push the attitude
 * into vayu's attitude_queue.
 *
 * Wire format: vsim_imu_frame_t = 16-byte vsim_hdr_t + 76-byte
 * bmx160_all_converted_reading_t payload (little-endian, gyr in deg/s).
 * Older builds used a raw 76-byte frame on /tmp/vayu_imu.fifo; that
 * path is gone -- the vsim_d daemon and the Gazebo bridge both now
 * speak the framed protocol.
 *
 * The mahony filter holds its quaternion state inside the attitude_t
 * we pass in - the same pattern bmx160.c uses on real hardware. We
 * keep one static attitude here, seeded to identity.
 *
 * If the FIFO doesn't exist yet we create it; if the producer hasn't
 * connected, open(O_RDONLY) blocks until it does. That's fine: this
 * runs in its own pthread and doesn't gate the rest of the SITL.
 */
#define _GNU_SOURCE
#include "host_clock.h"
#include "host_imu_feeder.h"
#include "vsim_iface.h"
#include "vsim_proto.h"

#include "est/est.h"
#include "sensor/sensor.h"
#include "variables.h" /* vayu_dt_from_cycles, SYS_CLOCK_FREQ */

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stddef.h> /* offsetof */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "task.h"  /* v_delay */
#include "utils.h" /* v_get_ticks */

/* vsim emits IMU at this rate (sim/vsim/src/main.cpp kImuHz). Each sample
 * therefore represents 1/SITL_IMU_FEED_HZ of SIM time — used to stamp a fixed
 * sensor cadence so the firmware estimator's dt is correct regardless of how
 * the host/FIFO is scheduled in wall-clock. Must track vsim's emit rate. */
#ifndef SITL_IMU_FEED_HZ
#define SITL_IMU_FEED_HZ 1000
#endif

/* IMU payload (within the framed protocol) is the firmware's converted
 * reading struct -- 76 B little-endian. The static_assert below couples
 * the wire layout to the firmware's struct so any drift is a build
 * failure, not a runtime mystery. */
#define EXPECTED_FRAME_BYTES VSIM_IMU_PAYLOAD_BYTES

/* Per-instance IMU FIFO (roadmap sim-integration #1 / HANDOFF §5.1):
 * append $VSIM_FIFO_SUFFIX to the shared base so colliding daemons can't
 * cross-feed torn frames (-> NaN attitude). Must match host_navhal's
 * scheme and the suffix SimWorker passes to vsim_d. */
static const char *imu_fifo_path(void) {
  static char p[128];
  if (p[0] == '\0') {
    const char *s = getenv("VSIM_FIFO_SUFFIX");
    snprintf(p, sizeof p, "%s%s", VSIM_FIFO_IMU, (s && *s) ? s : "");
  }
  return p;
}

static void ensure_fifo(void) {
  const char *path = imu_fifo_path();
  struct stat st;
  if (stat(path, &st) != 0) {
    if (mkfifo(path, 0666) != 0 && errno != EEXIST) {
      fprintf(stderr, "host_imu_feeder: mkfifo %s failed: %s\n", path,
              strerror(errno));
    }
  }
}

static int read_full(int fd, void *buf, size_t n) {
  size_t got = 0;
  while (got < n) {
    ssize_t r = read(fd, (char *)buf + got, n - got);
    if (r > 0) {
      got += (size_t)r;
      continue;
    }
    if (r == 0)
      return 0; /* EOF (writer closed) */
    if (errno == EINTR)
      continue;
    return -1;
  }
  return 1;
}

/* Give up after this many CONSECUTIVE rejected frames. A torn frame at
 * connect time is recoverable (a handful of bad reads, then we realign),
 * but a steady stream of rejects means a genuine wire mismatch -- a stale
 * vsim_d built against a different VSIM_PROTO_VERSION, most often. Spinning
 * on that forever is pointless; we surface it as a fatal error so the feeder
 * thread exits with a clear diagnosis instead of looping. */
#define MAX_CONSECUTIVE_BAD_FRAMES 64

/* Read one framed IMU message from `fd`. Hunts forward byte-by-byte
 * until VSIM_MAGIC appears, then validates type/version/length and
 * copies the 88-byte payload into `out_payload`. Returns 1 on a good
 * frame, 0 on EOF (writer closed), -1 on a read error OR a persistent
 * wire mismatch.
 *
 * Resync is an ITERATIVE loop, not recursion: a persistent mismatch
 * (e.g. a stale producer at the wrong proto version) yields a bad frame
 * on every read at ~1 kHz, and the old `return read_framed_imu(...)`
 * tail-call recursed once per reject -- with no guaranteed TCO that
 * overflows the stack and SIGSEGVs the host. Looping bounds the work to
 * O(1) stack regardless of how long the mismatch lasts. */
static int read_framed_imu(int fd, void *out_payload) {
  vsim_hdr_t hdr;
  unsigned bad = 0;
  while (1) {
    /* Magic resync: read until we land on the magic word. The producer
     * (vsim_d) only writes whole frames, but if the reader connects
     * mid-stream we may need to walk to the next boundary. */
    while (1) {
      int rc = read_full(fd, &hdr, sizeof(hdr));
      if (rc <= 0)
        return rc;
      if (hdr.magic == VSIM_MAGIC)
        break;
      /* Shift one byte forward and refill. Slow but only runs at
       * connect time / after a producer crash. */
      memmove(&hdr, ((char *)&hdr) + 1, sizeof(hdr) - 1);
      char extra;
      rc = read_full(fd, &extra, 1);
      if (rc <= 0)
        return rc;
      ((char *)&hdr)[sizeof(hdr) - 1] = extra;
    }

    if (hdr.version == VSIM_PROTO_VERSION && hdr.type == VSIM_FRAME_IMU &&
        hdr.payload_bytes == EXPECTED_FRAME_BYTES) {
      return read_full(fd, out_payload, EXPECTED_FRAME_BYTES);
    }

    /* Wire mismatch -- consume the body to stay aligned, then try the
     * next frame. */
    char drop[256];
    size_t remain = hdr.payload_bytes;
    while (remain > 0) {
      size_t chunk = remain > sizeof(drop) ? sizeof(drop) : remain;
      int rc = read_full(fd, drop, chunk);
      if (rc <= 0)
        return rc;
      remain -= chunk;
    }
    /* Log the first reject and then rate-limit, so a persistent mismatch
     * doesn't flood the log thousands of times a second before we bail. */
    if (bad == 0)
      fprintf(stderr,
              "host_imu_feeder: bad frame "
              "(ver=%u type=%u len=%u); resyncing "
              "(expected ver=%u type=%u len=%u -- stale vsim_d?)\n",
              hdr.version, hdr.type, hdr.payload_bytes, VSIM_PROTO_VERSION,
              VSIM_FRAME_IMU, EXPECTED_FRAME_BYTES);
    if (++bad >= MAX_CONSECUTIVE_BAD_FRAMES) {
      fprintf(stderr,
              "host_imu_feeder: %u consecutive bad frames -- giving up "
              "(producer proto mismatch). Rebuild vsim_d.\n",
              bad);
      return -1;
    }
  }
}

static void *imu_feeder_thread(void *arg) {
  (void)arg;

  /* Compile-time guarantee that the on-wire layout matches our struct. The
   * sim transmits the original 76-B converted reading (acc..temp); the
   * `timestamp` field appended for the dt refactor is host-set after the read,
   * so it must sit exactly at offset 76 (the wire fields stay byte-identical). */
  _Static_assert(offsetof(bmx160_all_converted_reading_t, timestamp) ==
                     EXPECTED_FRAME_BYTES,
                 "wire fields of bmx160_all_converted_reading_t must be 76 B");

  bmx160_all_reading_t sample;

  uint32_t frames = 0;
  uint32_t last_log_t = 0;

  /* IMU transport is FIFO-only as of the vsim_d split. Whether the
   * firmware is the standalone vayu_sitl binary or living inside
   * Navigator, samples arrive on /tmp/vsim_imu in the framed
   * protocol. */
  ensure_fifo();
  const char *imu_path = imu_fifo_path();
  fprintf(stderr, "host_imu_feeder: waiting for producer on %s\n", imu_path);
  int fd = open(imu_path, O_RDONLY);
  if (fd < 0) {
    fprintf(stderr, "host_imu_feeder: open failed: %s\n", strerror(errno));
    return NULL;
  }
  fprintf(stderr, "host_imu_feeder: producer connected, draining frames\n");

  while (1) {
    int rc = read_framed_imu(fd, &sample.converted);
    if (rc == 0) {
      /* Producer disconnected -- reopen and keep going. */
      fprintf(stderr, "host_imu_feeder: producer closed, reopening\n");
      close(fd);
      fd = open(imu_path, O_RDONLY);
      if (fd < 0) {
        fprintf(stderr, "host_imu_feeder: reopen failed\n");
        return NULL;
      }
      continue;
    }
    if (rc < 0) {
      fprintf(stderr, "host_imu_feeder: read failed: %s\n", strerror(errno));
      break;
    }

    /* Stamp each sample with a FIXED sim-time increment, modelling a real IMU's
     * constant ODR. The firmware derives dt from these stamps; the old code used
     * wall-clock, which jitters dt and COLLAPSES it when the vsim FIFO bursts
     * (the sim isn't perfectly real-time paced) — that under-integrated the
     * gyro ~10x in the estimator. SITL_IMU_FEED_HZ must match vsim's emit rate. */
    static uint32_t s_imu_cyc = 0;
    s_imu_cyc += (uint32_t)(SYS_CLOCK_FREQ / SITL_IMU_FEED_HZ);
    sample.converted.timestamp = s_imu_cyc;

    /* Inject RAW IMU ONLY — exactly what a real sensor provides. The firmware's
     * own attitude_task/EKF does the estimation (the previous host-side mahony
     * here BYPASSED the estimator SITL exists to test — a fidelity violation). */
    imu_queue_control_push(&sample);
    imu_queue_telemetry_push(&sample);
    imu_queue_attitude_push(&sample);

    /* Phase 1 lockstep (firmware/docs/plans/sitl-lockstep-sim.md): one IMU sample = one
     * step of SIM time. Advance the virtual clock AFTER the sample is queued
     * (so consumers see the data at the new time), which wakes the firmware's
     * control loops blocked in task_delay_until. Also drive the high-frequency
     * timestamp counter from the SAME clock — this replaces the old wall-clock
     * hf_timer_thread, so telemetry timestamps track sim time at any speed. */
    host_clock_advance_us((uint64_t)1000000 / SITL_IMU_FEED_HZ);
    {
      extern void increment_high_freq_timer(void);
      static uint32_t hf_carry = 0; /* fractional-tick accumulator */
      hf_carry += HIGH_FREQ_TIMER_FREQ;        /* HF ticks per second ... */
      while (hf_carry >= SITL_IMU_FEED_HZ) {   /* ... emit per-sample share */
        increment_high_freq_timer();
        hf_carry -= SITL_IMU_FEED_HZ;
      }
    }

    frames++;
    uint32_t now = v_get_ticks();
    if (now - last_log_t >= 1000) {
      //  fprintf(stderr,
      //         "host_imu_feeder: %u frames, last roll=%.2f pitch=%.2f
      //         yaw=%.2f\n",
      //        frames, (double)att.roll, (double)att.pitch, (double)att.yaw);
      last_log_t = now;
    }
  }

  if (fd >= 0)
    close(fd);
  return NULL;
}

void host_imu_feeder_start(void) {
  pthread_t th;
  pthread_create(&th, NULL, imu_feeder_thread, NULL);
  pthread_detach(th);
}

/* ---- single-shot pump for the RTOS cooperative stepper (Phase 4) ------ *
 * The real-vaios SITL drives sensors from one thread, so it reads + injects one
 * IMU frame synchronously via these instead of running imu_feeder_thread. The
 * stepper owns the SysTick + HF clock, so pump does NOT advance them here.
 * v_semaphore_give (in the queue pushes) only enqueues the woken attitude task
 * to the ready list — no context switch — so calling these from the stepper
 * (outside the scheduler) is safe; the scheduler runs them on the next
 * host_rtos_run_until_idle(). */
static int s_step_imu_fd = -1;

int host_imu_feeder_open(void) {
  ensure_fifo();
  s_step_imu_fd = open(imu_fifo_path(), O_RDONLY);
  return s_step_imu_fd;
}

int host_imu_feeder_pump(void) {
  static bmx160_all_reading_t sample;
  static uint32_t cyc = 0;
  if (s_step_imu_fd < 0)
    return 0;
  int rc = read_framed_imu(s_step_imu_fd, &sample.converted);
  if (rc <= 0)
    return 0;                       /* EOF / wire error */
  cyc += (uint32_t)(SYS_CLOCK_FREQ / SITL_IMU_FEED_HZ);
  sample.converted.timestamp = cyc; /* fixed-ODR sim stamp (drives estimator dt) */
  imu_queue_control_push(&sample);
  imu_queue_telemetry_push(&sample);
  imu_queue_attitude_push(&sample);
  return 1;
}
