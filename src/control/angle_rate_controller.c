#include "control/angle_rate_controller.h"
#include "actuator/motor.h"
#include "comm/ibus.h"
#include "comm/rc_buffer.h"
#include "core/cortex-m4/dwt.h"
#include "sensor/bmx160.h"
#include "sensor/imu_buffer.h"
#include "vaios.h"
#include "variables.h"

static AngleRateController angle_rate_controller = {
    .pid = {// roll angle rate gains
            {
                .Kp = DEAFULT_ROLL_ANGLE_RATE_KP,
                .Ki = DEAFULT_ROLL_ANGLE_RATE_KI,
                .Kd = DEAFULT_ROLL_ANGLE_RATE_KD,
                .Kff = DEAFULT_ROLL_ANGLE_RATE_KFF,
                .i_max = DEAFULT_ROLL_ANGLE_RATE_I_MAX,
                .d_max = DEAFULT_ROLL_ANGLE_RATE_D_MAX,
                .d_lpf_rc = DEAFULT_ROLL_ANGLE_RATE_D_LPF_RC,
                .d_filtered = 0,
                .integral = 0,
                .prev_meas = 0,
                .out_min = DEAFULT_ROLL_ANGLE_RATE_OUT_MIN,
                .out_max = DEAFULT_ROLL_ANGLE_RATE_OUT_MAX,
                .initialized = false,
            },
            // pitch angle rate gains
            {
                .Kp = DEAFULT_PITCH_ANGLE_RATE_KP,
                .Ki = DEAFULT_PITCH_ANGLE_RATE_KI,
                .Kd = DEAFULT_PITCH_ANGLE_RATE_KD,
                .Kff = DEAFULT_PITCH_ANGLE_RATE_KFF,
                .i_max = DEAFULT_PITCH_ANGLE_RATE_I_MAX,
                .d_max = DEAFULT_PITCH_ANGLE_RATE_D_MAX,
                .d_lpf_rc = DEAFULT_PITCH_ANGLE_RATE_D_LPF_RC,
                .d_filtered = 0,
                .integral = 0,
                .prev_meas = 0,
                .out_min = DEAFULT_PITCH_ANGLE_RATE_OUT_MIN,
                .out_max = DEAFULT_PITCH_ANGLE_RATE_OUT_MAX,
                .initialized = false,
            },
            // yaw angle rate gains
            {
                .Kp = DEAFULT_YAW_ANGLE_RATE_KP,
                .Ki = DEAFULT_YAW_ANGLE_RATE_KI,
                .Kd = DEAFULT_YAW_ANGLE_RATE_KD,
                .Kff = DEAFULT_YAW_ANGLE_RATE_KFF,
                .i_max = DEAFULT_YAW_ANGLE_RATE_I_MAX,
                .d_max = DEAFULT_YAW_ANGLE_RATE_D_MAX,
                .d_lpf_rc = DEAFULT_YAW_ANGLE_RATE_D_LPF_RC,
                .d_filtered = 0,
                .integral = 0,
                .prev_meas = 0,
                .out_min = DEAFULT_YAW_ANGLE_RATE_OUT_MIN,
                .out_max = DEAFULT_YAW_ANGLE_RATE_OUT_MAX,
                .initialized = false,
            }}};

static inline float get_dt(void) {
  static uint32_t last_time = 0;
  uint32_t current_time = dwt_get_cycles();
  float dt = (float)(current_time - last_time) / SYS_CLOCK_FREQ;
  last_time = current_time;
  return dt;
}
void angle_rate_controller_init(void) {
  for (int i = 0; i < NUM_AXES; i++) {
    v_pid_init(&angle_rate_controller.pid[i], angle_rate_controller.pid[i].Kp,
               angle_rate_controller.pid[i].Ki, angle_rate_controller.pid[i].Kd,
               angle_rate_controller.pid[i].Kff,
               angle_rate_controller.pid[i].i_max,
               angle_rate_controller.pid[i].d_max,
               angle_rate_controller.pid[i].d_lpf_rc,
               angle_rate_controller.pid[i].out_min,
               angle_rate_controller.pid[i].out_max);
  }
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

void angle_rate_controller_task(void *arg) {
  angle_rate_controller_init();
  static bmx160_all_reading_t imu_data = {0};
  static bmx160_all_reading_t prev_imu_data = {0};
  static ibus_data_t rc_data;
  static ibus_data_t prev_rc_data;
  static float prev_target_rates[NUM_AXES] = {0};
  static motor_outputs_t motor_outputs = {0};
  set_motor_ready(true);
  while (1) {
    // Getting all the data

    // Get IMU data
    if (imu_queue_control_pop(&imu_data)) {
      prev_imu_data = imu_data;
    } else {
      imu_data = prev_imu_data;
    }

    float dt = get_dt();

    if (rc_queue_control_pop(&rc_data)) {
      prev_rc_data = rc_data;
    } else {
      rc_data = prev_rc_data;
    }

    // Applying deadband to the gyro data
    for (int i = 0; i < NUM_AXES; i++) {
      if (m_fabsf(imu_data.converted.gyr[i]) < PID_GYRO_DEADBAND) {
        imu_data.converted.gyr[i] = 0;
      }
    }

    // Normalizing the rc data
    rc_data_t normalized_rc_data = normalize_rc_data(rc_data);
    float target_roll_rate = normalized_rc_data.channels[0];
    float target_pitch_rate = normalized_rc_data.channels[1];
    float target_throttle = normalized_rc_data.channels[2];
    float target_yaw_rate = normalized_rc_data.channels[3];
    float target_rates[NUM_AXES] = {target_roll_rate, target_pitch_rate,
                                    target_yaw_rate};
    float current_rates[NUM_AXES] = {imu_data.converted.gyr[0],
                                     imu_data.converted.gyr[1],
                                     imu_data.converted.gyr[2]};
    // Apply PID to each axis
    float outputs[NUM_AXES] = {0};
    for (int i = 0; i < NUM_AXES; i++) {
      float dot_sp = (target_rates[i] - prev_target_rates[i]) / dt;
      outputs[i] = v_pid_update(&angle_rate_controller.pid[i], target_rates[i],
                                current_rates[i], dot_sp, dt);
      prev_target_rates[i] = target_rates[i];
    }

    // calculate motor outputs
    if (target_throttle < MIN_ARMED_THROTTLE) {
      for (int i = 0; i < NUM_AXES; i++) {
        outputs[i] = 0;
      }
    }
    // Your layout:
    // Front Left  = M4
    // Front Right = M1
    // Rear Left   = M3
    // Rear Right  = M2
    // Motor               Throttle         Roll           Pitch          Yaw
    motor_outputs.m1 = target_throttle - outputs[0] + outputs[1] + outputs[2];
    motor_outputs.m2 = target_throttle - outputs[0] - outputs[1] - outputs[2];
    motor_outputs.m3 = target_throttle + outputs[0] - outputs[1] + outputs[2];
    motor_outputs.m4 = target_throttle + outputs[0] + outputs[1] - outputs[2];

    if (motor_outputs.m1 < 0 || motor_outputs.m2 < 0 || motor_outputs.m3 < 0 ||
        motor_outputs.m4 < 0) {
      float min_output = motor_outputs.m1;
      if (motor_outputs.m2 < min_output) {
        min_output = motor_outputs.m2;
      }
      if (motor_outputs.m3 < min_output) {
        min_output = motor_outputs.m3;
      }
      if (motor_outputs.m4 < min_output) {
        min_output = motor_outputs.m4;
      }
      motor_outputs.m1 -= min_output;
      motor_outputs.m2 -= min_output;
      motor_outputs.m3 -= min_output;
      motor_outputs.m4 -= min_output;
    }

    if (motor_outputs.m1 > 1.0f || motor_outputs.m2 > 1.0f ||
        motor_outputs.m3 > 1.0f || motor_outputs.m4 > 1.0f) {
      float max_output = motor_outputs.m1;
      if (motor_outputs.m2 > max_output) {
        max_output = motor_outputs.m2;
      }
      if (motor_outputs.m3 > max_output) {
        max_output = motor_outputs.m3;
      }
      if (motor_outputs.m4 > max_output) {
        max_output = motor_outputs.m4;
      }
      motor_outputs.m1 /= max_output;
      motor_outputs.m2 /= max_output;
      motor_outputs.m3 /= max_output;
      motor_outputs.m4 /= max_output;
    }
    motor_set_outputs(motor_outputs);
    v_delay(1);
  }
}