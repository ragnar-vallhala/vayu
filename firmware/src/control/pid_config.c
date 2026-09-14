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
#include "dsp/gyro_notch.h"   /* gyro_notch set/get params (notch persist) */
#include "memory.h"           /* v_memcpy */
#include "storage/fs_owner.h" /* vayu_log */
#include "variables.h"        /* NUM_AXES */
#include "vfs.h"
#include "maths/maths_interface.h"

/* Bumped PID4 -> PID5 for the motor-geometry fields below (was PID3 -> PID4 for
 * the gyro notch). Any older file fails the exact-size/magic check in
 * pid_config_init and resets to defaults (same policy as the earlier bumps). */
#define PID_CONFIG_MAGIC 0x50494435u /* 'P''I''D''5' */

typedef struct {
  float kp, ki, kd, kff;
  uint8_t valid; /* 0 until a value has been stored for this slot */
} pid_gains_t;

typedef struct {
  uint32_t magic;
  pid_gains_t gains[PID_CTRL_COUNT][NUM_AXES];
  float gyro_lpf[NUM_AXES]; /* rate-loop gyro LPF time constant [s] */
  uint8_t gyro_lpf_valid[NUM_AXES];
  float d_lpf[NUM_AXES]; /* rate-loop D-term LPF time constant [s] */
  uint8_t d_lpf_valid[NUM_AXES];
  /* Dynamic gyro-notch tune. Global (not per-axis): the notch applies the same
   * detection band/Q to every axis. Stores the EFFECTIVE param set (read back
   * from gyro_notch after apply) so a partial command persists a full tune. */
  float notch_q;
  float notch_fmin_hz;
  float notch_fmax_hz;
  float notch_min_ratio;
  uint8_t notch_enabled; /* master enable persisted across boots */
  uint8_t notch_valid;   /* 0 until a notch command has been stored */
  /* Airframe motor geometry (CMD_SET_MOTOR_GEOMETRY): per-motor body position
   * [m] + spin (+1/-1). Persisted so the mixer signs survive a reboot instead of
   * falling back to the compiled default layout. */
  float motor_pos_x[4];
  float motor_pos_y[4];
  int8_t motor_spin[4];     /* +1 CW-sign / -1 per motor */
  uint8_t motor_geom_valid; /* 0 until a geometry has been stored */
} pid_store_t;

/* The store is written verbatim as one FS-owner save payload; keep it within the
 * queue's per-request buffer (FS_SAVE_PAYLOAD_MAX in fs_owner.c). Bump both together. */
_Static_assert(
    sizeof(pid_store_t) <= 216u,
    "pid_store_t exceeds FS_SAVE_PAYLOAD_MAX (raise it in fs_owner.c)");

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
/** @noreq static store-slot accessor (infrastructure). */
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

/** @noreq gyro-LPF store getter (CMD_SET_GYRO_LPF persistence; see CTRL-RATE-105). */
bool pid_config_get_gyro_lpf(uint8_t axis, float *rc) {
  if (axis >= NUM_AXES || rc == NULL || !s_store.gyro_lpf_valid[axis]) {
    return false;
  }
  *rc = s_store.gyro_lpf[axis];
  return true;
}

/** @noreq D-term LPF store getter (CMD_SET_D_LPF persistence). */
bool pid_config_get_d_lpf(uint8_t axis, float *rc) {
  if (axis >= NUM_AXES || rc == NULL || !s_store.d_lpf_valid[axis]) {
    return false;
  }
  *rc = s_store.d_lpf[axis];
  return true;
}

/** @noreq gyro-notch store getter (CMD_SET_GYRO_NOTCH persistence). Fills the
 * effective tune + enable if one was ever stored; false leaves out-params. */
bool pid_config_get_gyro_notch(float *q, float *fmin_hz, float *fmax_hz,
                               float *min_ratio, bool *enabled) {
  if (!s_store.notch_valid) {
    return false;
  }
  if (q) {
    *q = s_store.notch_q;
  }
  if (fmin_hz) {
    *fmin_hz = s_store.notch_fmin_hz;
  }
  if (fmax_hz) {
    *fmax_hz = s_store.notch_fmax_hz;
  }
  if (min_ratio) {
    *min_ratio = s_store.notch_min_ratio;
  }
  if (enabled) {
    *enabled = s_store.notch_enabled != 0;
  }
  return true;
}

/** @noreq motor-geometry store getter (CMD_SET_MOTOR_GEOMETRY persistence). Fills
 * the persisted layout if one was ever stored; false leaves the out-params. */
bool pid_config_get_motor_geometry(float pos_x[4], float pos_y[4],
                                   int spin[4]) {
  if (!s_store.motor_geom_valid) {
    return false;
  }
  for (int i = 0; i < 4; i++) {
    if (pos_x) {
      pos_x[i] = s_store.motor_pos_x[i];
    }
    if (pos_y) {
      pos_y[i] = s_store.motor_pos_y[i];
    }
    if (spin) {
      /* int8_t holding +1/-1: the sign is the payload, so the check's
       * suggested unsigned-char round-trip would invert -1 to 255. */
      spin[i] = (int)s_store.motor_spin[i];
    }
  }
  return true;
}

/* ------------------------------------------------------------------ save */
/* Hand a snapshot of the store to the centralised FS owner. The live-controller
 * apply at the call site already happened and is authoritative; persistence is
 * asynchronous (off the comm task, so a slow SD write can't stall RX) and
 * best-effort. Snapshotting is what makes the async write safe against a
 * concurrent CMD_SET_PID mutating s_store before the FS task runs. */
/** @noreq async store-snapshot persist to SD (glue for COMM-CMD-003). */
static void pid_config_save(void) {
  s_store.magic = PID_CONFIG_MAGIC;
  fs_owner_enqueue_pid_save(&s_store, sizeof s_store);
}

/* --------------------------------------------------------------- command */
/** @noreq payload float-arg extractor (infrastructure). */
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
  if (!m_isfinite(kp) || !m_isfinite(ki) || !m_isfinite(kd) ||
      !m_isfinite(kff)) {
    return VAYU_ERR_INVALID; /* reject NaN/Inf gains outright */
  }

  bool applied =
      (ctrl == PID_CTRL_RATE)
          ? angle_rate_controller_set_gains((uint8_t)axis, kp, ki, kd, kff)
          : angle_controller_set_gains((uint8_t)axis, kp, ki, kd, kff);
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

/**
 * Validate and apply a CMD_SET_GYRO_LPF payload: argc/length checked before any
 * arg read (COMM-CMD-002), then pushed live and persisted. The gyro-LPF feature
 * itself is covered by CTRL-RATE-105.
 *
 * @implements COMM-CMD-002, CTRL-RATE-105
 */
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
  if (axis < 0 || axis >= NUM_AXES || !m_isfinite(rc) || rc < 0.0f) {
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

/**
 * Validate and apply a CMD_SET_D_LPF payload: argc/length checked before any
 * arg read (COMM-CMD-002), then pushed live to the rate PID's D filter
 * (the D-LPF behaviour is CTRL-PID-101) and persisted.
 *
 * @implements COMM-CMD-002
 */
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
  if (axis < 0 || axis >= NUM_AXES || !m_isfinite(rc) || rc < 0.0f) {
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

/**
 * Apply and persist a gyro-notch tune. Unlike the byte-payload commands above,
 * the notch command carries typed fields, so this takes them directly. A <=0
 * detection field means "leave unchanged" (gyro_notch_set_params ignores it);
 * we then read the EFFECTIVE param set back so the store always holds a complete
 * tune. NaN/Inf are rejected outright.
 */
vayu_status_t pid_config_apply_gyro_notch(bool enabled, float q, float fmin_hz,
                                          float fmax_hz, float min_ratio) {
  if (!m_isfinite(q) || !m_isfinite(fmin_hz) || !m_isfinite(fmax_hz) ||
      !m_isfinite(min_ratio)) {
    return VAYU_ERR_INVALID;
  }
  /* Live apply first (partial: non-positive fields are left as-is), then the
   * master gate so an enable takes effect with the freshly-set band/Q. */
  gyro_notch_set_params(q, fmin_hz, fmax_hz, min_ratio);
  gyro_notch_set_enabled(enabled);

  /* Persist the effective set: seed with the command values, then overwrite with
   * whatever the notch is actually running (covers the <=0 "unchanged" fields).
   * If the notch never initialised, the seeds stand and enabling is a harmless
   * no-op on the next boot. */
  float eq = q, efmin = fmin_hz, efmax = fmax_hz, eratio = min_ratio;
  gyro_notch_get_params(&eq, &efmin, &efmax, &eratio);
  s_store.notch_q = eq;
  s_store.notch_fmin_hz = efmin;
  s_store.notch_fmax_hz = efmax;
  s_store.notch_min_ratio = eratio;
  s_store.notch_enabled = enabled ? 1u : 0u;
  s_store.notch_valid = 1;
  pid_config_save();
  vayu_log("[PID] set notch en=%d Q=%.2f band=%.0f-%.0f ratio=%.2f",
           (int)enabled, (double)eq, (double)efmin, (double)efmax,
           (double)eratio);
  return VAYU_OK;
}

/**
 * Persist an already-applied motor geometry. The caller
 * (angle_rate_controller_apply_geometry_command) has already pushed it to the
 * live mixer; this only snapshots it to SD so the layout survives a reboot. Skips
 * the write (returns VAYU_ERR_INVALID) on a non-finite position so a bad command
 * can't poison the store.
 */
vayu_status_t pid_config_store_motor_geometry(const float pos_x[4],
                                              const float pos_y[4],
                                              const int spin[4]) {
  if (pos_x == NULL || pos_y == NULL || spin == NULL) {
    return VAYU_ERR_INVALID;
  }
  for (int i = 0; i < 4; i++) {
    if (!m_isfinite(pos_x[i]) || !m_isfinite(pos_y[i])) {
      return VAYU_ERR_INVALID;
    }
  }
  for (int i = 0; i < 4; i++) {
    s_store.motor_pos_x[i] = pos_x[i];
    s_store.motor_pos_y[i] = pos_y[i];
    s_store.motor_spin[i] = (int8_t)(spin[i] >= 0 ? 1 : -1);
  }
  s_store.motor_geom_valid = 1;
  pid_config_save();
  vayu_log("[PID] store motor geometry (spin %d %d %d %d)", spin[0], spin[1],
           spin[2], spin[3]);
  return VAYU_OK;
}
