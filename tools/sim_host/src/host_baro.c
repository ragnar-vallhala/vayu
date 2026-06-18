/*
 * host_baro.c -- SITL barometer feeder. Reads modelled barometric pressure
 * frames (vsim_baro_frame_t) from vsim_d's /tmp/vsim_baro FIFO and injects the
 * PHYSICAL pressure/temperature/humidity into the REAL firmware bme280 driver
 * via bme280_publish(). The firmware then derives altitude and emits the BARO
 * telemetry through its normal path (telemetry_task -> navlink_tx_baro), so
 * SITL exercises the actual FC baro code — no host-side altitude, mirroring how
 * host_imu_feeder hands over physical IMU and lets the FC estimator run.
 *
 * This file also defines the i2c_manager_* stubs: the real src/sensor/bme280.c
 * is compiled into vayu_sitl_core (for its altitude/publish/getters), and its
 * hardware I2C entry points must resolve at link time even though SITL never
 * calls them (the feeder drives bme280_publish directly).
 */
#define _GNU_SOURCE
#include "host_baro.h"
#include "sensor/bme280.h"
#include "sensor/i2c_manager.h"
#include "vsim_proto.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define BARO_PAYLOAD_BYTES ((uint32_t)(sizeof(vsim_baro_frame_t) - sizeof(vsim_hdr_t)))
#define MAX_CONSECUTIVE_BAD_FRAMES 64

/* Per-instance FIFO: append $VSIM_FIFO_SUFFIX so colliding daemons can't
 * cross-feed. Must match host_imu_feeder's scheme and vsim_d's suffix. */
static const char *baro_fifo_path(void) {
  static char p[128];
  if (p[0] == '\0') {
    const char *s = getenv("VSIM_FIFO_SUFFIX");
    snprintf(p, sizeof p, "%s%s", VSIM_FIFO_BARO, (s && *s) ? s : "");
  }
  return p;
}

static void ensure_fifo(void) {
  const char *path = baro_fifo_path();
  struct stat st;
  if (stat(path, &st) != 0) {
    if (mkfifo(path, 0666) != 0 && errno != EEXIST) {
      fprintf(stderr, "host_baro: mkfifo %s failed: %s\n", path,
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

/* Read one framed BARO message: hunt forward to VSIM_MAGIC, validate
 * type/version/length, copy the 12-byte payload. Returns 1 ok, 0 EOF, -1 err. */
static int read_framed_baro(int fd, vsim_baro_frame_t *out) {
  vsim_hdr_t hdr;
  unsigned bad = 0;
  while (1) {
    while (1) {
      int rc = read_full(fd, &hdr, sizeof(hdr));
      if (rc <= 0)
        return rc;
      if (hdr.magic == VSIM_MAGIC)
        break;
      memmove(&hdr, ((char *)&hdr) + 1, sizeof(hdr) - 1);
      char extra;
      rc = read_full(fd, &extra, 1);
      if (rc <= 0)
        return rc;
      ((char *)&hdr)[sizeof(hdr) - 1] = extra;
    }

    if (hdr.version == VSIM_PROTO_VERSION && hdr.type == VSIM_FRAME_BARO &&
        hdr.payload_bytes == BARO_PAYLOAD_BYTES) {
      out->hdr = hdr;
      return read_full(fd, &out->pressure_pa, BARO_PAYLOAD_BYTES);
    }

    /* Wrong frame on this FIFO — consume the body and resync. */
    char drop[256];
    size_t remain = hdr.payload_bytes;
    while (remain > 0) {
      size_t chunk = remain > sizeof(drop) ? sizeof(drop) : remain;
      int rc = read_full(fd, drop, chunk);
      if (rc <= 0)
        return rc;
      remain -= chunk;
    }
    if (bad == 0)
      fprintf(stderr,
              "host_baro: bad frame (ver=%u type=%u len=%u); resyncing "
              "(expected ver=%u type=%u len=%u -- stale vsim_d?)\n",
              hdr.version, hdr.type, hdr.payload_bytes, VSIM_PROTO_VERSION,
              VSIM_FRAME_BARO, BARO_PAYLOAD_BYTES);
    if (++bad >= MAX_CONSECUTIVE_BAD_FRAMES) {
      fprintf(stderr, "host_baro: %u consecutive bad frames -- giving up "
                      "(producer proto mismatch). Rebuild vsim_d.\n", bad);
      return -1;
    }
  }
}

static void *baro_feeder_thread(void *arg) {
  (void)arg;
  ensure_fifo();
  const char *path = baro_fifo_path();
  fprintf(stderr, "host_baro: waiting for producer on %s\n", path);
  int fd = open(path, O_RDONLY);
  if (fd < 0) {
    fprintf(stderr, "host_baro: open failed: %s\n", strerror(errno));
    return NULL;
  }
  fprintf(stderr, "host_baro: producer connected, draining frames\n");

  vsim_baro_frame_t frame;
  while (1) {
    int rc = read_framed_baro(fd, &frame);
    if (rc == 0) {
      fprintf(stderr, "host_baro: producer closed, reopening\n");
      close(fd);
      fd = open(path, O_RDONLY);
      if (fd < 0) {
        fprintf(stderr, "host_baro: reopen failed\n");
        return NULL;
      }
      continue;
    }
    if (rc < 0) {
      fprintf(stderr, "host_baro: read failed: %s\n", strerror(errno));
      break;
    }
    /* Hand the modelled physical reading to the REAL FC driver; it derives
     * altitude and the telemetry task emits the BARO packet. */
    bme280_publish(frame.pressure_pa, frame.temperature_c, frame.humidity_rh);
  }

  if (fd >= 0)
    close(fd);
  return NULL;
}

void host_baro_start(void) {
  pthread_t th;
  pthread_create(&th, NULL, baro_feeder_thread, NULL);
  pthread_detach(th);
}

/* ---- i2c_manager stubs (see file header) ---- */
hal_status_t i2c_manager_write(uint8_t addr, uint8_t *data, uint16_t len) {
  (void)addr; (void)data; (void)len;
  return HAL_ERR_NOT_INITIALIZED;
}
hal_status_t i2c_manager_read(uint8_t addr, uint8_t *data, uint16_t len) {
  (void)addr; (void)data; (void)len;
  return HAL_ERR_NOT_INITIALIZED;
}
hal_status_t i2c_manager_write_read(uint8_t addr, uint8_t *tx_data,
                                    uint16_t tx_len, uint8_t *rx_data,
                                    uint16_t rx_len) {
  (void)addr; (void)tx_data; (void)tx_len; (void)rx_data; (void)rx_len;
  return HAL_ERR_NOT_INITIALIZED;
}
