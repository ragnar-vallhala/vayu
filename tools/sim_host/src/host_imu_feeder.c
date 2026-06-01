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
#include "host_imu_feeder.h"
#include "vsim_iface.h"
#include "vsim_proto.h"

#include "est/est.h"
#include "sensor/sensor.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "task.h"  /* v_delay */
#include "utils.h" /* v_get_ticks */

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

/* Read one framed IMU message from `fd`. Hunts forward byte-by-byte
 * until VSIM_MAGIC appears, then validates type/version/length and
 * copies the 76-byte payload into `out_payload`. Returns 1 on a good
 * frame, 0 on EOF (writer closed), -1 on a read error. */
static int read_framed_imu(int fd, void *out_payload) {
  vsim_hdr_t hdr;
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

  if (hdr.version != VSIM_PROTO_VERSION || hdr.type != VSIM_FRAME_IMU ||
      hdr.payload_bytes != EXPECTED_FRAME_BYTES) {
    /* Wire mismatch -- consume the body to stay aligned, then bail
     * on the caller's next call. */
    char drop[256];
    size_t remain = hdr.payload_bytes;
    while (remain > 0) {
      size_t chunk = remain > sizeof(drop) ? sizeof(drop) : remain;
      int rc = read_full(fd, drop, chunk);
      if (rc <= 0)
        return rc;
      remain -= chunk;
    }
    fprintf(stderr,
            "host_imu_feeder: bad frame "
            "(ver=%u type=%u len=%u); resyncing\n",
            hdr.version, hdr.type, hdr.payload_bytes);
    return read_framed_imu(fd, out_payload);
  }

  return read_full(fd, out_payload, EXPECTED_FRAME_BYTES);
}

static void *imu_feeder_thread(void *arg) {
  (void)arg;

  /* Compile-time guarantee that the on-wire layout matches our struct. */
  _Static_assert(sizeof(bmx160_all_converted_reading_t) == EXPECTED_FRAME_BYTES,
                 "bmx160_all_converted_reading_t must be 76 B on this build");

  bmx160_all_reading_t sample;
  attitude_t att = {
      .roll = 0, .pitch = 0, .yaw = 0, .q = {1.0f, 0.0f, 0.0f, 0.0f}};

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

    /* The IMU sample feeds the angle_rate_controller directly. */
    imu_queue_control_push(&sample);
    imu_queue_telemetry_push(&sample);

    /* Mahony updates `att` in place (its quaternion is the filter state).
     * dt is taken from hal_cycle_counter_get(), which our host shim
     * derives from CLOCK_MONOTONIC. */
    m_mahony_filter(sample.converted.acc[0], sample.converted.acc[1],
                    sample.converted.acc[2], sample.converted.gyr[0],
                    sample.converted.gyr[1], sample.converted.gyr[2],
                    sample.converted.mag[0], sample.converted.mag[1],
                    sample.converted.mag[2], &att);

    attitude_queue_control_push(&att);
    attitude_queue_telemetry_push(&att);

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
