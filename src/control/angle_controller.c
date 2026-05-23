#include "control/angle_controller.h"
#include "comm/ibus.h"
#include "comm/rc_buffer.h"
#include "control/angle_rate_controller.h"
#include "maths/maths_interface.h"
#include "maths/pid.h"
#include "maths/sensor_fusion.h"
#include "sensor/imu_buffer.h"
#include "structure.h"
#include "sys/state.h"
#include "vaios.h"
#include "variables.h"

#define ANGLE_CONTROLLER_2_RATE_CONTROLLER_BUFFER_SIZE 4

static angle_controller_outputs_t
    angle_controller_outputs[ANGLE_CONTROLLER_2_RATE_CONTROLLER_BUFFER_SIZE] = {
        0};
static spsc_fifo_t angle_controller_fifo;
static void init_fifo(void) {
  spsc_init(&angle_controller_fifo, angle_controller_outputs,
            ANGLE_CONTROLLER_2_RATE_CONTROLLER_BUFFER_SIZE,
            sizeof(angle_controller_outputs_t));
  spsc_set_policy(&angle_controller_fifo, SPSC_POLICY_OVERWRITE);
}
static void fifo_push(angle_controller_outputs_t *outputs) {
  spsc_write(&angle_controller_fifo, outputs, 1);
}

bool angle_controller_get_outputs(angle_controller_outputs_t *outputs) {
  return spsc_read(&angle_controller_fifo, outputs, 1);
}

#ifdef VAYU_SIM
volatile uint32_t dbg_ang_ctrl_iter = 0;
volatile float dbg_ang_target_throttle = -99.0f;
volatile uint32_t dbg_ang_pop_ok = 0;
volatile uint16_t dbg_ang_rc_ch2 = 0xBEEF;
#endif

static angle_controller_t angle_controller = {
    .pid = {
        {
            .Kp = DEAFULT_ROLL_ANGLE_KP,
            .Ki = 0,
            .Kd = 0,
            .Kff = 0,
            .i_max = 0,
            .d_max = 0,
            .d_lpf_rc = 0,
            .d_filtered = 0,
            .integral = 0,
            .prev_meas = 0,
            .out_min = -DEAFULT_ROLL_ANGLE_OUT_MAX,
            .out_max = DEAFULT_ROLL_ANGLE_OUT_MAX,
            .initialized = false,
        },
        {
            .Kp = DEAFULT_PITCH_ANGLE_KP,
            .Ki = 0,
            .Kd = 0,
            .Kff = 0,
            .i_max = 0,
            .d_max = 0,
            .d_lpf_rc = 0,
            .d_filtered = 0,
            .integral = 0,
            .prev_meas = 0,
            .out_min = -DEAFULT_PITCH_ANGLE_OUT_MAX,
            .out_max = DEAFULT_PITCH_ANGLE_OUT_MAX,
            .initialized = false,
        },
        {
            .Kp = DEAFULT_YAW_ANGLE_KP,
            .Ki = 0,
            .Kd = 0,
            .Kff = 0,
            .i_max = 0,
            .d_max = 0,
            .d_lpf_rc = 0,
            .d_filtered = 0,
            .integral = 0,
            .prev_meas = 0,
            .out_min = -DEAFULT_YAW_ANGLE_OUT_MAX,
            .out_max = DEAFULT_YAW_ANGLE_OUT_MAX,
            .initialized = false,
        },
    }};

void angle_controller_init(void) {
  init_fifo();
  for (int i = 0; i < NUM_AXES; i++) {
    v_pid_init(&angle_controller.pid[i], angle_controller.pid[i].Kp,
               angle_controller.pid[i].Ki, angle_controller.pid[i].Kd,
               angle_controller.pid[i].Kff, angle_controller.pid[i].i_max,
               angle_controller.pid[i].d_max, angle_controller.pid[i].d_lpf_rc,
               angle_controller.pid[i].out_min,
               angle_controller.pid[i].out_max);
  }
}

static inline float get_dt(void) {
  static uint32_t last_time = 0;
  uint32_t current_time = hal_cycle_counter_get();
  float dt = (float)(current_time - last_time) / SYS_CLOCK_FREQ;
  last_time = current_time;
  return dt;
}
typedef struct {
  // normal rc channels
  float channels[4];
  //
} rc_data_t;

static inline rc_data_t normalize_rc_data(ibus_data_t rc_data) {
  rc_data_t normalized_rc_data;
  for (int i = 0; i < 4; i++) {
    if (i != 2) {
      // Apply deadband
      if (rc_data.channels[i] > 1500 + PID_RC_DEADBAND) {
        normalized_rc_data.channels[i] =
            ((float)rc_data.channels[i] - 1500.0f) / 500.0f;
      } else if (rc_data.channels[i] < 1500 - PID_RC_DEADBAND) {
        normalized_rc_data.channels[i] =
            ((float)rc_data.channels[i] - 1500.0f) / 500.0f;
      } else {
        normalized_rc_data.channels[i] = 0;
      }
    } else {
      // Throttle is not deaband at 1500
      normalized_rc_data.channels[i] =
          ((float)rc_data.channels[i] - 1000.0f) / 1000.0f;
    }
  }

  switch (PID_RC2ANGLE_RATE_MODE) {
  case NORMALIZED_RC2ANGLE_RATE_LINEAR:
    break;
  case NORMALIZED_RC2ANGLE_RATE_CUBIC:
    for (int i = 0; i < 4; i++) {
      if (i != 2) {
        normalized_rc_data.channels[i] = normalized_rc_data.channels[i] *
                                         normalized_rc_data.channels[i] *
                                         normalized_rc_data.channels[i];
      }
    }
    break;
  }
  return normalized_rc_data;
}

void angle_controller_task(void *arg) {
  angle_controller_init();
  static attitude_t attitude;
  static attitude_t last_attitude;
  static angle_controller_outputs_t angle_controller_outputs;
  static ibus_data_t rc_data;
  static ibus_data_t prev_rc_data;
  while (1) {
    float dt = get_dt();
    if (rc_queue_control_pop(&rc_data)) {
      prev_rc_data = rc_data;
#ifdef VAYU_SIM
      dbg_ang_pop_ok++;
#endif
    } else {
      rc_data = prev_rc_data;
    }
#ifdef VAYU_SIM
    dbg_ang_rc_ch2 = rc_data.channels[2];
#endif
    // Normalizing the rc data
    rc_data_t normalized_rc_data = normalize_rc_data(rc_data);
    float target_angles[NUM_AXES];

    target_angles[0] =
        normalized_rc_data.channels[0] * DEAFULT_ROLL_ANGLE_TARGET_MAX;
    target_angles[1] =
        -normalized_rc_data.channels[1] *
        DEAFULT_PITCH_ANGLE_TARGET_MAX; // Forward push if acieved by tilting
                                        // back motors up which is negative
                                        // pitch
    float target_throttle = normalized_rc_data.channels[2];
    target_angles[2] =
        normalized_rc_data.channels[3] * DEAFULT_YAW_ANGLE_TARGET_MAX;

    if (!attitude_queue_control_pop(&attitude)) {
      attitude = last_attitude;
    }
    last_attitude = attitude;
    if (m_fabsf(attitude.roll) > MAX_ANGLE_CUTOFF ||
        m_fabsf(attitude.pitch) > MAX_ANGLE_CUTOFF ||
        m_fabsf(attitude.yaw) > MAX_ANGLE_CUTOFF) {
      system_state_set(SYSTEM_STATE_FAILSAFE);
    }
    // Calculate target rates
    float current_angles[NUM_AXES];
    current_angles[0] = attitude.roll;
    current_angles[1] = attitude.pitch;
    current_angles[2] = attitude.yaw;
    for (int i = 0; i < 3; i++) {
      angle_controller_outputs.angle_rates[i] = v_pid_update(
          &angle_controller.pid[i], target_angles[i], current_angles[i], 0, dt);
      angle_controller_outputs.angle_sp[i] = target_angles[i];
      angle_controller_outputs.angle_curr[i] = current_angles[i];
    }
    angle_controller_outputs.throttle = target_throttle;
    angle_controller_outputs.dt = dt;
    fifo_push(&angle_controller_outputs);
#ifdef VAYU_SIM
    dbg_ang_target_throttle = target_throttle;
    dbg_ang_ctrl_iter++;
#endif
    v_delay(2);
  }
}
