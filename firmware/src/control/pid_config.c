/**
 * @file src/control/pid_config.c
 * @brief CMD_SET_PID dispatch, live gain apply, and SD persistence.
 *
 * @implements COMM-CMD-003
 *
 * Owns the persisted PID gain store. Gains set over the link are applied
 * to the live controllers immediately and written to SD (0:pid.bin),
 * mirroring the IMU-calibration persistence in src/sensor/bmx160.c. At
 * boot pid_config_init() reads the file; the controllers consult the
 * store during their own init (pid_config_get_*), so a stored tune
 * survives reboot.
 */
#include "control/pid_config.h"
#include "control/angle_controller.h"
#include "control/angle_rate_controller.h"
#include "memory.h"                   /* v_memcpy */
#include "storage/fs_owner.h"              /* vayu_log */
#include "variables.h"                /* NUM_AXES */
#include "vfs.h"
#include <math.h>

#define PID_CONFIG_MAGIC     0x50494433u /* 'P''I''D''3' */
#define PID_CONFIG_FILE_PATH "0:pid.bin"

typedef struct {
  float kp, ki, kd, kff;
  uint8_t valid; /* 0 until a value has been stored for this slot */
} pid_gains_t;

typedef struct {
  uint32_t magic;
  pid_gains_t gains[PID_CTRL_COUNT][NUM_AXES];
  float   gyro_lpf[NUM_AXES];        /* rate-loop gyro LPF time constant [s] */
  uint8_t gyro_lpf_valid[NUM_AXES];
  float   d_lpf[NUM_AXES];           /* rate-loop D-term LPF time constant [s] */
  uint8_t d_lpf_valid[NUM_AXES];
} pid_store_t;

/* Zero-init: magic 0, every slot valid == 0 → controllers keep defaults
 * until either a load restores values or a command sets them. */
static pid_store_t s_store;

/* ------------------------------------------------------------------ load */
void pid_config_init(void) {
  vfs_fd_t fd = vfs_open(PID_CONFIG_FILE_PATH, VFS_O_RDONLY);
  if (fd < 0) {
    return; /* no persisted tune — defaults stand */
  }
  pid_store_t tmp;
  int n = vfs_read(fd, &tmp, sizeof tmp);
  vfs_close(fd);
  if (n == (int)sizeof tmp && tmp.magic == PID_CONFIG_MAGIC) {
    s_store = tmp;
  } else {
    vayu_log("[PID] config file present but invalid; using defaults");
  }
}

/* ------------------------------------------------------------------- get */
static bool get_slot(pid_ctrl_sel_t ctrl, uint8_t axis, float *kp, float *ki,
                     float *kd, float *kff) {
  if (ctrl >= PID_CTRL_COUNT || axis >= NUM_AXES) {
    return false;
  }
  const pid_gains_t *g = &s_store.gains[ctrl][axis];
  if (!g->valid) {
    return false;
  }
  *kp = g->kp;
  *ki = g->ki;
  *kd = g->kd;
  *kff = g->kff;
  return true;
}

bool pid_config_get_rate(uint8_t axis, float *kp, float *ki, float *kd,
                         float *kff) {
  return get_slot(PID_CTRL_RATE, axis, kp, ki, kd, kff);
}

bool pid_config_get_angle(uint8_t axis, float *kp, float *ki, float *kd,
                          float *kff) {
  return get_slot(PID_CTRL_ANGLE, axis, kp, ki, kd, kff);
}

bool pid_config_get_gyro_lpf(uint8_t axis, float *rc) {
  if (axis >= NUM_AXES || rc == NULL || !s_store.gyro_lpf_valid[axis]) {
    return false;
  }
  *rc = s_store.gyro_lpf[axis];
  return true;
}

bool pid_config_get_d_lpf(uint8_t axis, float *rc) {
  if (axis >= NUM_AXES || rc == NULL || !s_store.d_lpf_valid[axis]) {
    return false;
  }
  *rc = s_store.d_lpf[axis];
  return true;
}

/* ------------------------------------------------------------------ save */
/* Hand a snapshot of the store to the centralised FS owner. The live-controller
 * apply at the call site already happened and is authoritative; persistence is
 * asynchronous (off the comm task, so a slow SD write can't stall RX) and
 * best-effort. Snapshotting is what makes the async write safe against a
 * concurrent CMD_SET_PID mutating s_store before the FS task runs. */
static void pid_config_save(void) {
  s_store.magic = PID_CONFIG_MAGIC;
  fs_owner_enqueue_pid_save(&s_store, sizeof s_store);
}

/* --------------------------------------------------------------- command */
static float arg_f(const uint8_t *payload, int idx) {
  float f;
  v_memcpy(&f, &payload[3 + idx * 4], 4); /* args start at payload[3] */
  return f;
}

vayu_status_t pid_config_apply_command(const uint8_t *payload,
                                       uint16_t payload_len) {
  if (payload == NULL || payload_len < 3) {
    return VAYU_ERR_INVALID; /* no room for cmd_id(2) + argc(1) */
  }

  /* COMM-CMD-002: validate argc and that the payload carries argc 4-byte
   * args before reading any of them. */
  uint8_t argc = payload[2];
  if (argc < PID_SET_ARGC) {
    return VAYU_ERR_INVALID;
  }
  if (payload_len < (uint16_t)argc * 4u + 3u) {
    return VAYU_ERR_INVALID;
  }

  float fctrl = arg_f(payload, 0);
  float faxis = arg_f(payload, 1);
  float kp = arg_f(payload, 2);
  float ki = arg_f(payload, 3);
  float kd = arg_f(payload, 4);
  float kff = arg_f(payload, 5);

  /* Selectors arrive as floats (uniform 4-byte slots); round + range-check. */
  int ctrl = (int)(fctrl + 0.5f);
  int axis = (int)(faxis + 0.5f);
  if (ctrl < 0 || ctrl >= PID_CTRL_COUNT || axis < 0 || axis >= NUM_AXES) {
    return VAYU_ERR_INVALID;
  }
  if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) || !isfinite(kff)) {
    return VAYU_ERR_INVALID; /* reject NaN/Inf gains outright */
  }

  bool applied = (ctrl == PID_CTRL_RATE)
                     ? angle_rate_controller_set_gains((uint8_t)axis, kp, ki,
                                                       kd, kff)
                     : angle_controller_set_gains((uint8_t)axis, kp, ki, kd,
                                                  kff);
  if (!applied) {
    return VAYU_ERR_INVALID;
  }

  pid_gains_t *g = &s_store.gains[ctrl][axis];
  g->kp = kp;
  g->ki = ki;
  g->kd = kd;
  g->kff = kff;
  g->valid = 1;
  pid_config_save();

  vayu_log("[PID] set ctrl=%d axis=%d Kp=%.4f Ki=%.4f Kd=%.4f", ctrl, axis,
           (double)kp, (double)ki, (double)kd);
  return VAYU_OK;
}

vayu_status_t pid_config_apply_gyro_lpf_command(const uint8_t *payload,
                                                uint16_t payload_len) {
  if (payload == NULL || payload_len < 3) {
    return VAYU_ERR_INVALID;
  }
  uint8_t argc = payload[2];
  if (argc < GYRO_LPF_ARGC || payload_len < (uint16_t)argc * 4u + 3u) {
    return VAYU_ERR_INVALID;
  }
  float faxis = arg_f(payload, 0);
  float rc = arg_f(payload, 1);
  int axis = (int)(faxis + 0.5f);
  if (axis < 0 || axis >= NUM_AXES || !isfinite(rc) || rc < 0.0f) {
    return VAYU_ERR_INVALID;
  }
  if (!angle_rate_controller_set_gyro_lpf((uint8_t)axis, rc)) {
    return VAYU_ERR_INVALID;
  }
  s_store.gyro_lpf[axis] = rc;
  s_store.gyro_lpf_valid[axis] = 1;
  pid_config_save();
  vayu_log("[PID] set gyro_lpf axis=%d rc=%.4f", axis, (double)rc);
  return VAYU_OK;
}

vayu_status_t pid_config_apply_d_lpf_command(const uint8_t *payload,
                                             uint16_t payload_len) {
  if (payload == NULL || payload_len < 3) {
    return VAYU_ERR_INVALID;
  }
  uint8_t argc = payload[2];
  if (argc < D_LPF_ARGC || payload_len < (uint16_t)argc * 4u + 3u) {
    return VAYU_ERR_INVALID;
  }
  float faxis = arg_f(payload, 0);
  float rc = arg_f(payload, 1);
  int axis = (int)(faxis + 0.5f);
  if (axis < 0 || axis >= NUM_AXES || !isfinite(rc) || rc < 0.0f) {
    return VAYU_ERR_INVALID;
  }
  if (!angle_rate_controller_set_d_lpf((uint8_t)axis, rc)) {
    return VAYU_ERR_INVALID;
  }
  s_store.d_lpf[axis] = rc;
  s_store.d_lpf_valid[axis] = 1;
  pid_config_save();
  vayu_log("[PID] set d_lpf axis=%d rc=%.4f", axis, (double)rc);
  return VAYU_OK;
}
