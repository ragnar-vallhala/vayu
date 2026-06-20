/*
 * host_rc_feeder.c -- read CSV RC frames from the sim_bridge MCU on a
 * serial port and push them into vayu's rc_queue_control.
 *
 * sim_bridge.ino (tools/sim_bridge/) decodes PPM from an FS-iA6B
 * receiver and emits one CSV line per frame at ~50 Hz, like:
 *
 *     # sim_bridge ready                  (startup banner, one-shot)
 *     1500,1500,1000,1500,2000,1000       (us per channel)
 *     NO_SIGNAL                            (no valid PPM for >100 ms)
 *
 * Channel order is the FS-i6 default (Mode 2): roll, pitch, throttle,
 * yaw, SwA, VrA/SwB/SwD. We map them 1:1 into ibus_data_t.channels
 * because the PPM and iBus pulse-width ranges (1000..2000 us) align.
 *
 * If VAYU_UART_RC_PATH is unset, defaults to /dev/ttyUSB0. If the port
 * doesn't exist or disappears, the thread falls back to a synthetic
 * hover frame so the SITL stays useful standalone (no Arduino plugged
 * in). It will keep retrying the port in the background and switch to
 * real RC the moment it appears.
 */
#define _GNU_SOURCE
#include "host_rc_feeder.h"

#include "comm/comm.h"
#include "sys/state.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "host_clock.h" /* host_wall_delay_ms */
#include "task.h"
#include "utils.h"
#include "vaios.h"

#define DEFAULT_UART_PATH "/dev/ttyUSB0"

/* Hover frame for standalone fallback. Throttle=1300 (just above min),
 * SwA=2000 (arm). Matches the original synthetic feeder in host_main.c. */
static void fill_hover(ibus_data_t *rc) {
  memset(rc, 0, sizeof(*rc));
  rc->channels[0] = 1500;
  rc->channels[1] = 1500;
  rc->channels[2] = 1300;
  rc->channels[3] = 1500;
  rc->channels[4] = 2000;
  for (int i = 5; i < 14; i++)
    rc->channels[i] = 1500;
  rc->is_failsafe = false;
}

static int open_serial(const char *path) {
  int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    return -1;

  struct termios tio;
  if (tcgetattr(fd, &tio) != 0) {
    close(fd);
    return -1;
  }
  cfmakeraw(&tio);
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= (tcflag_t)~CRTSCTS;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    close(fd);
    return -1;
  }

  /* Keep the fd in O_NONBLOCK mode: read_line() polls and sleeps via
   * v_delay() while waiting for the next character, and uses EAGAIN
   * vs. read-returns-0 to distinguish "no data right now" from a
   * genuine writer-closed EOF. Clearing O_NONBLOCK with VMIN=VTIME=0
   * would make read() return 0 on every idle gap and we'd
   * reopen-storm. */

  return fd;
}

/* Parse a CSV line of microsecond values into rc->channels[0..13].
 * Missing channels are filled with 1500. Returns 1 on success, 0 on
 * "NO_SIGNAL", -1 on malformed input. */
static int parse_csv_line(const char *line, ibus_data_t *rc) {
  if (strncmp(line, "NO_SIGNAL", 9) == 0)
    return 0;
  if (line[0] == '#' || line[0] == '\0')
    return -1;

  memset(rc, 0, sizeof(*rc));
  int ch_idx = 0;
  const char *p = line;
  while (*p && ch_idx < 14) {
    char *end;
    long v = strtol(p, &end, 10);
    if (end == p)
      break;
    if (v < 800 || v > 2200)
      v = 1500;
    rc->channels[ch_idx++] = (uint16_t)v;
    p = end;
    while (*p == ',' || *p == ' ')
      p++;
  }
  if (ch_idx == 0)
    return -1;
  for (; ch_idx < 14; ch_idx++)
    rc->channels[ch_idx] = 1500;
  rc->is_failsafe = false;
  return 1;
}

/* Read a single line ('\n'-terminated, '\r' tolerant) from fd into buf.
 * Returns the line length on success (line NUL-terminated, sans newline),
 * -1 on real I/O error, -2 on timeout (caller can keep the fd open).
 *
 * On a non-blocking tty, "no data right now" arrives as both EAGAIN
 * and (Linux pty quirk) read-returns-0 in the middle of an idle gap.
 * We treat both as "wait and retry"; a real writer-closed EOF surfaces
 * as -1/EIO instead (or stays as the harmless r==0 polling). */
static ssize_t read_line(int fd, char *buf, size_t bufsz, int timeout_ms) {
  size_t n = 0;
  int waited_ms = 0;
  while (n + 1 < bufsz) {
    char c;
    ssize_t r = read(fd, &c, 1);
    if (r == 1) {
      waited_ms = 0;
      if (c == '\r')
        continue;
      if (c == '\n') {
        buf[n] = '\0';
        return (ssize_t)n;
      }
      buf[n++] = c;
      continue;
    }
    if (r == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      if (waited_ms >= timeout_ms)
        return -2;
      host_wall_delay_ms(1);  /* serial I/O backoff: real time, not sim time */
      waited_ms++;
      continue;
    }
    if (errno == EINTR)
      continue;
    return -1;
  }
  buf[bufsz - 1] = '\0';
  return (ssize_t)n;
}

/* Apply vayu's rc_task arm/disarm state machine to a freshly-read frame.
 * Same logic as src/comm/rc_task.c's parser-driven path. */
static void apply_arm_logic(const ibus_data_t *rc) {
  sys_state_t cur = system_state_get();
  if (rc_arm_engaged(rc)) {
    if (cur == SYSTEM_STATE_STANDBY && rc->channels[2] < 1100) {
      VAYU_DISCARD(system_state_set(SYSTEM_STATE_ARMED));
    } else if (cur == SYSTEM_STATE_STANDBY && rc->channels[2] > 1100) {
      VAYU_DISCARD(system_state_set(SYSTEM_STATE_FAILSAFE));
    }
  } else {
    if (cur == SYSTEM_STATE_ARMED || cur == SYSTEM_STATE_FAILSAFE) {
      VAYU_DISCARD(system_state_set(SYSTEM_STATE_STANDBY));
    }
  }
}

static void *rc_feeder_thread(void *arg) {
  (void)arg;
  const char *path = getenv("VAYU_UART_RC_PATH");
  if (!path || !*path)
    path = DEFAULT_UART_PATH;

  ibus_data_t rc;
  fill_hover(&rc);

  int fd = -1;
  char line[128];
  uint32_t pushed_real = 0, pushed_synth = 0;
  uint32_t last_log_t = 0;
  int last_open_failed_log = -1;

  while (1) {
    if (fd < 0) {
      fd = open_serial(path);
      if (fd < 0) {
        /* Log the open failure once per ~5 s; otherwise just push
         * synthetic hover and try again later. */
        uint32_t now = v_get_ticks();
        if (last_open_failed_log < 0 ||
            (int)(now - (uint32_t)last_open_failed_log) > 5000) {
          fprintf(stderr,
                  "host_rc_feeder: %s not available (%s) - "
                  "synthesizing hover frame; retrying\n",
                  path, strerror(errno));
          last_open_failed_log = (int)now;
        }
        fill_hover(&rc);
        rc_queue_control_push(&rc);
        rc_queue_telemetry_push(&rc);
        pushed_synth++;
        host_wall_delay_ms(20);  /* port-reopen retry: real time, not sim time */
        continue;
      }
      fprintf(stderr, "host_rc_feeder: opened %s @ 115200 8N1\n", path);
      last_open_failed_log = -1;
    }

    ssize_t n = read_line(fd, line, sizeof(line), 500);
    if (n == -2) {
      /* No frame for 500 ms — the producer is silent but the
       * port is still open. Don't reopen; just emit a failsafe
       * frame and keep polling. */
      rc.is_failsafe = true;
      rc_queue_control_push(&rc);
      rc_queue_telemetry_push(&rc);
      rc.is_failsafe = false;
      continue;
    }
    if (n < 0) {
      fprintf(stderr, "host_rc_feeder: read err on %s (%s), reopening\n", path,
              strerror(errno));
      close(fd);
      fd = -1;
      continue;
    }

    int rv = parse_csv_line(line, &rc);
    if (rv == 1) {
      apply_arm_logic(&rc);
      rc_queue_control_push(&rc);
      rc_queue_telemetry_push(&rc);
      pushed_real++;
    } else if (rv == 0) {
      /* NO_SIGNAL: hold last frame; mark failsafe so the firmware
       * downstream can react if it wants to. */
      rc.is_failsafe = true;
      rc_queue_control_push(&rc);
      rc_queue_telemetry_push(&rc);
      rc.is_failsafe = false;
    } /* rv < 0 -> banner / blank, ignore */

    uint32_t now = v_get_ticks();
    if (now - last_log_t >= 1000) {
      //  fprintf(stderr, "host_rc_feeder: real=%u synth=%u "
      //                 "ch[0..4]=%u,%u,%u,%u,%u\n",
      //        pushed_real, pushed_synth,
      //       rc.channels[0], rc.channels[1], rc.channels[2],
      //      rc.channels[3], rc.channels[4]);
      last_log_t = now;
    }
  }
  return NULL;
}

void host_rc_feeder_start(void) {
  pthread_t th;
  pthread_create(&th, NULL, rc_feeder_thread, NULL);
  pthread_detach(th);
}
