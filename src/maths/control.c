#include "maths/control.h"
#include "actuator/esc.h"
#include "comm/ibus.h"
#include "comm/rc_buffer.h"
#include "core/cortex-m4/dwt.h"
#include "maths/control_buffer.h"
#include "sensor/bmx160.h"
#include "sensor/imu_buffer.h"
#include "structure.h"
#include "sys/state.h"
#include "utils.h"
#include "utils/utils.h"
#include "vaios.h"
#include "variables.h"
#include "vfs.h"
#include <math.h>
#include <stdint.h>

#define CONTROL_LOOP_BUFFER_SIZE 6
#define CONTROL_DATA_COUNTER_INTERVAL 25

control_config_t g_control_config = {
    .roll_angle = {.kp = 2.0f,
                   .ki = 0.0f,
                   .kd = 0.0f,
                   .i_limit = 0.0f,
                   .out_limit = MAX_CONTROL_RATE},
    .pitch_angle = {.kp = 2.0f,
                    .ki = 0.0f,
                    .kd = 0.0f,
                    .i_limit = 0.0f,
                    .out_limit = MAX_CONTROL_RATE},
    .yaw_angle = {.kp = 0.0f,
                  .ki = 0.00f,
                  .kd = 0.000f,
                  .i_limit = 0.0f,
                  .out_limit = MAX_CONTROL_RATE,
                  .d_lpf_alpha = 0.0f,
                  .expo = 0.0f},
    .roll_rate = {.kp = 0.045f,
                  .ki = 0.004f,
                  .kd = 0.003f,
                  .i_limit = 0.06f,
                  .out_limit = 0.25f,
                  .d_lpf_alpha = 0.12f},
    .pitch_rate = {.kp = 0.045f,
                   .ki = 0.004f,
                   .kd = 0.003f,
                   .i_limit = 0.06f,
                   .out_limit = 0.25f,
                   .d_lpf_alpha = 0.12f},
    .yaw_rate = {.kp = 0.00f,
                 .ki = 0.0f,
                 .kd = 0.000f,
                 .i_limit = 0.0f,
                 .out_limit = 0.0f,
                 .d_lpf_alpha = 0.0f}};

// PID Controllers
static pid_controller_t pid_roll_angle, pid_roll_rate;
static pid_controller_t pid_pitch_angle, pid_pitch_rate;
static pid_controller_t pid_yaw_angle, pid_yaw_rate;
static ESC_Handle motors[4];
static control_loop_data_t control_loop_data[CONTROL_LOOP_BUFFER_SIZE];
static spsc_fifo_t control_loop_fifo;

void control_loop_fifo_init(void) {
  spsc_init(&control_loop_fifo, control_loop_data, CONTROL_LOOP_BUFFER_SIZE,
            sizeof(control_loop_data_t));
  spsc_set_policy(&control_loop_fifo, SPSC_POLICY_OVERWRITE);
}

bool control_loop_fifo_push(control_loop_data_t *data) {
  return spsc_write(&control_loop_fifo, data, 1) == 1;
}

bool control_loop_fifo_pop(control_loop_data_t *data) {
  return spsc_read(&control_loop_fifo, data, 1) == 1;
}

void pid_init(pid_controller_t *pid, const pid_params_t *params) {
  pid->kp = params->kp;
  pid->ki = params->ki;
  pid->kd = params->kd;
  pid->i_limit = params->i_limit;
  pid->output_limit = params->out_limit;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  lpf_init(&pid->lpf_d, params->d_lpf_alpha);
}
static float expo(float x, float expo) {
  return x * (1.0f - expo) + x * x * x * expo;
}
void pid_reset(pid_controller_t *pid) {
  pid->integral = 0.0f;
  pid->prev_value = 0.0f;
}

float pid_calculate(pid_controller_t *pid, float setpoint, float current_value,
                    float dt) {
  if (!isfinite(setpoint) || !isfinite(current_value) || !isfinite(dt) ||
      dt <= 0.0f) {
    return 0.0f;
  }

  float error = setpoint - current_value;

  // --- P ---
  float p_out = pid->kp * error;

  // --- D ---
  current_value = lpf_apply(&pid->lpf_d, current_value);

  float derivative = (current_value - pid->prev_value) / dt;

  if (!isfinite(derivative) || fabsf(derivative) > 10000.0f) {
    derivative = 0.0f;
  }

  float d_out = pid->kd * derivative;

  // --- I (with proper anti-windup) ---
  float i_candidate = pid->integral + error * dt;

  // clamp integral candidate
  if (i_candidate > pid->i_limit)
    i_candidate = pid->i_limit;
  else if (i_candidate < -pid->i_limit)
    i_candidate = -pid->i_limit;

  float i_out = pid->ki * i_candidate;

  // --- Combine ---
  float output = p_out + i_out - d_out;

  if (!isfinite(output)) {
    return 0.0f;
  }

  // --- Output saturation + anti-windup ---
  if (output > pid->output_limit) {
    output = pid->output_limit;
    // DON'T accept integral if pushing further into saturation
    if (error * output < 0) {
      pid->integral = i_candidate;
    }
  } else if (output < -pid->output_limit) {
    output = -pid->output_limit;
    if (error * output > 0) {
      pid->integral = i_candidate;
    }
  } else {
    // safe region → accept integral
    pid->integral = i_candidate;
  }

  pid->prev_value = current_value;

  return output;
}

static void pid_update_gains(pid_controller_t *pid,
                             const pid_params_t *params) {
  pid->kp = params->kp;
  pid->ki = params->ki;
  pid->kd = params->kd;
  pid->i_limit = params->i_limit;
  pid->output_limit = params->out_limit;
  pid->lpf_d.alpha = params->d_lpf_alpha;
}

void control_init(void) {
  control_loop_fifo_init();

  // vfs_fd_t fd = vfs_open(PID_FILE_PATH, VFS_O_RDONLY);
  // if (fd >= 0) {
  //   uint8_t buffer[sizeof(control_config_t) + sizeof(uint32_t)];
  //   int br = vfs_read(fd, buffer, sizeof(buffer));
  //   vfs_close(fd);

  //   if (br == sizeof(buffer)) {
  //     control_config_t temp_config;
  //     v_memcpy(&temp_config, buffer, sizeof(control_config_t));

  //     uint32_t computed_crc = utils_compute_crc32((const uint8_t
  //     *)&temp_config,
  //                                                 sizeof(control_config_t));

  //     uint32_t stored_crc;
  //     v_memcpy(&stored_crc, buffer + sizeof(control_config_t),
  //              sizeof(uint32_t));

  //     if (computed_crc == stored_crc) {
  //       g_control_config = temp_config;
  //     }
  //   }
  // } else {
  //   fd = vfs_open(PID_FILE_PATH, VFS_O_WRONLY | VFS_O_CREAT | VFS_O_TRUNC);
  //   if (fd >= 0) {
  //     uint32_t computed_crc = utils_compute_crc32(
  //         (const uint8_t *)&g_control_config, sizeof(control_config_t));

  //     vfs_write(fd, &g_control_config, sizeof(control_config_t));
  //     vfs_write(fd, &computed_crc, sizeof(uint32_t));
  //     vfs_close(fd);
  //   }
  // }

  // Angle Loops (Outer)
  pid_init(&pid_roll_angle, &g_control_config.roll_angle);
  pid_init(&pid_pitch_angle, &g_control_config.pitch_angle);
  pid_init(&pid_yaw_angle, &g_control_config.yaw_angle);

  // Rate Loops (Inner)
  pid_init(&pid_roll_rate, &g_control_config.roll_rate);
  pid_init(&pid_pitch_rate, &g_control_config.pitch_rate);
  pid_init(&pid_yaw_rate, &g_control_config.yaw_rate);
}

void control_task(void *args) {
  (void)args;

  control_init();

  // Setup ESCs (Mapping from motor_task.c)
  esc_init(&motors[0], TIM1, 1, GPIO_PA08); // Motor 1
  esc_init(&motors[1], TIM1, 2, GPIO_PA09); // Motor 2
  esc_init(&motors[2], TIM1, 3, GPIO_PA10); // Motor 3
  esc_init(&motors[3], TIM1, 4, GPIO_PA11); // Motor 4

  for (int i = 0; i < 4; i++) {
    esc_arm(&motors[i]);
    v_delay(4);
  }
  v_delay(100);

  static bmx160_all_reading_t imu_data = {0};
  static attitude_t attitude = {0};
  static attitude_t last_attitude = {0};
  static bmx160_all_reading_t last_valid = {0};
  static ibus_data_t rc_data;
  static uint16_t rc_channels[IBUS_MAX_CHANNELS];

  int count = 0;
  uint32_t last = dwt_get_cycles();
  static control_config_t new_cfg;

  while (1) {
    if (pid_config_t2c_pop(&new_cfg)) {
      g_control_config = new_cfg;
      pid_update_gains(&pid_roll_angle, &g_control_config.roll_angle);
      pid_update_gains(&pid_pitch_angle, &g_control_config.pitch_angle);
      pid_update_gains(&pid_yaw_angle, &g_control_config.yaw_angle);
      pid_update_gains(&pid_roll_rate, &g_control_config.roll_rate);
      pid_update_gains(&pid_pitch_rate, &g_control_config.pitch_rate);
      pid_update_gains(&pid_yaw_rate, &g_control_config.yaw_rate);

      pid_config_c2t_push(&g_control_config);
    }

    count++;
    uint32_t n = dwt_get_cycles();
    float dt = ((float)(n - last)) / (float)SYS_CLOCK_FREQ;
    last = n;
    if (dt <= 1e-6f || dt > 0.05f) { // reject anything > 50ms as bogus
      last = dwt_get_cycles();
      // vayu_log("Rejected control loop dt: %f\n", dt);
      v_delay(1);
      continue;
    }

    // 1. Get latest sensor data
    // Use averaged gyro data from the buffer if available

    if (!imu_queue_control_pop(&imu_data)) {
      // Fallback: use last valid data OR zero
      imu_data = last_valid;
    } else {
      // vayu_log("Failed to pop IMU data in control loop");
      // Save last good sample
      last_valid = imu_data;
    }
    for (int i = 0; i < 3; i++) {
      if (fabsf(imu_data.converted.gyr[i]) < 0.1f)
        imu_data.converted.gyr[i] = 0.0f;
    }
    //  Sanitize gyro (CRITICAL)
    for (int i = 0; i < 3; i++) {
      if (!isfinite(imu_data.converted.gyr[i]) ||
          fabsf(imu_data.converted.gyr[i]) > 2000.0f) {
        imu_data.converted.gyr[i] = 0.0f;
      }
    }
    // Get current attitude (updated by fusion task)
    if (attitude_queue_control_pop(&attitude)) {
      last_attitude = attitude;
    } else {
      attitude = last_attitude;
    }

    // 2. Get RC setpoints and map to physical units
    if (rc_queue_control_pop(&rc_data)) {
      v_memcpy(rc_channels, rc_data.channels,
               sizeof(rc_channels)); // Dangerous memcopy from struct
    }
    float throttle = ((float)rc_channels[2] - 1000.0f) / 1000.0f;

    if (system_state_get() == SYSTEM_STATE_ARMED &&
        throttle < MOTOR_MIN_THROTTLE) {
      throttle = MOTOR_MIN_THROTTLE;
    }

    // Clamp throttle
    if (throttle < 0.0f)
      throttle = 0.0f;
    if (throttle > 1.0f)
      throttle = 1.0f;

    // 3. Map RC sticks to Target Angles (Roll/Pitch) or Rate (Stick)
    float target_angle_roll = (expo(((float)rc_channels[0] - 1500.0f) / 500.0f,
                                    g_control_config.roll_rate.expo)) *
                              MAX_CONTROL_ANGLE;

    float target_angle_pitch = (expo(((float)rc_channels[1] - 1500.0f) / 500.0f,
                                     g_control_config.pitch_rate.expo)) *
                               MAX_CONTROL_ANGLE;

    // For Angle Mode, Yaw is typically still rate controlled by the pilot.
    float target_rate_yaw_stick =
        -(expo(((float)rc_channels[3] - 1500.0f) / 500.0f,
               g_control_config.yaw_rate.expo)) *
        MAX_CONTROL_RATE;

    // 4. Outer Loop (Angle Control)
    // Convert angle error to target rate
    float target_rate_roll = pid_calculate(&pid_roll_angle, target_angle_roll,
                                           attitude.roll, dt, NULL);
    float target_rate_pitch = pid_calculate(
        &pid_pitch_angle, target_angle_pitch, -attitude.pitch, dt, NULL);

    float target_rate_yaw = target_rate_yaw_stick;
    float max_rate = 60.0f; // CRITICAL

    if (target_rate_pitch > max_rate)
      target_rate_pitch = max_rate;
    if (target_rate_pitch < -max_rate)
      target_rate_pitch = -max_rate;

    if (target_rate_roll > max_rate)
      target_rate_roll = max_rate;
    if (target_rate_roll < -max_rate)
      target_rate_roll = -max_rate;

    // 5. Inner Loop (Rate Control)
    float derrivative = -imu_data.converted.gyr[0];
    float out_roll = pid_calculate(&pid_roll_rate, target_rate_roll,
                                   imu_data.converted.gyr[0], dt, &derrivative);
    derrivative = -imu_data.converted.gyr[1];
    float out_pitch =
        pid_calculate(&pid_pitch_rate, target_rate_pitch,
                      imu_data.converted.gyr[1], dt, &derrivative);
    derrivative = -imu_data.converted.gyr[2];
    float out_yaw = pid_calculate(&pid_yaw_rate, target_rate_yaw,
                                  imu_data.converted.gyr[2], dt, &derrivative);

    if (system_state_get() == SYSTEM_STATE_ARMED) {
      if (attitude.roll > MAX_ALLOWED_ANGLE ||
          attitude.roll < -MAX_ALLOWED_ANGLE ||
          attitude.pitch > MAX_ALLOWED_ANGLE ||
          attitude.pitch < -MAX_ALLOWED_ANGLE) {
        system_state_set(SYSTEM_STATE_FAILSAFE);
      }
    }
    // Your layout:
    // Front Left  = M4
    // Front Right = M1
    // Rear Left   = M3
    // Rear Right  = M2
    // M1 = Front Right
    if (throttle <= 0.06f) {
      out_roll = 0;
      out_pitch = 0;
      out_yaw = 0;
    }
    float authority = 0.4f + 0.4f * throttle;
    out_roll *= authority;
    out_pitch *= authority;
    out_yaw *= authority;
    float base = throttle;
    float limit = throttle * 0.6f;
    if (limit < 0.01f)
      limit = 0.01f;
    // out_roll = limit * tanhf(out_roll / limit);
    // out_pitch = limit * tanhf(out_pitch / limit);
    // out_yaw = limit * tanhf(out_yaw / limit);

    float m1 = base - (out_roll + out_pitch + out_yaw);
    float m2 = base - (out_roll - out_pitch - out_yaw);
    float m3 = base + (out_roll + out_pitch - out_yaw);
    float m4 = base + (out_roll - out_pitch + out_yaw);

    // --- DESATURATION ---
    float min_motor = fminf(fminf(m1, m2), fminf(m3, m4));
    float max_motor = fmaxf(fmaxf(m1, m2), fmaxf(m3, m4));

    // shift ALL motors together instead of clipping
    if (min_motor < 0.0f) {
      m1 -= min_motor;
      m2 -= min_motor;
      m3 -= min_motor;
      m4 -= min_motor;
    }

    if (max_motor > 1.0f) {
      float excess = max_motor - 1.0f;
      m1 -= excess;
      m2 -= excess;
      m3 -= excess;
      m4 -= excess;
    }

    // final clamp (should rarely trigger now)
    m1 = fminf(fmaxf(m1, 0.0f), 1.0f);
    m2 = fminf(fmaxf(m2, 0.0f), 1.0f);
    m3 = fminf(fmaxf(m3, 0.0f), 1.0f);
    m4 = fminf(fmaxf(m4, 0.0f), 1.0f);

    if (system_state_get() != SYSTEM_STATE_ARMED) {
      m1 = 0;
      m2 = 0;
      m3 = 0;
      m4 = 0;
    }
    float motor_cmds[4] = {m1, m2, m3, m4};
    for (int i = 0; i < 4; i++) {
      if (motor_cmds[i] < 0.0f)
        motor_cmds[i] = 0.0f;
      if (motor_cmds[i] > 1.0f)
        motor_cmds[i] = 1.0f;
      esc_set_throttle(&motors[i], motor_cmds[i]);
    }

    // Push motor PWMs and PID errors to SPSC queues for telemetry
    motor_pwm_data_t m_data;
    v_memcpy(m_data.motors, motor_cmds, sizeof(m_data.motors));
    motor_queue_push(&m_data);

    if (count % CONTROL_DATA_COUNTER_INTERVAL == 0) {
      control_loop_data_t c_data = {
          .roll_angle_error = pid_roll_angle.prev_error,
          .pitch_angle_error = pid_pitch_angle.prev_error,
          .yaw_angle_error = pid_yaw_angle.prev_error,
          .roll_rate_error = pid_roll_rate.prev_error,
          .pitch_rate_error = pid_pitch_rate.prev_error,
          .yaw_rate_error = pid_yaw_rate.prev_error,
          .dt = dt,
          .roll_angle_setpoint = target_angle_roll,
          .pitch_angle_setpoint = target_angle_pitch,
          .yaw_angle_setpoint = target_rate_yaw,
          .roll_rate_setpoint = target_rate_roll,
          .pitch_rate_setpoint = target_rate_pitch,
          .yaw_rate_setpoint = target_rate_yaw,
          .roll_output = out_roll,
          .pitch_output = out_pitch,
          .yaw_output = out_yaw,
          .throttle_output = throttle};
      control_loop_fifo_push(&c_data);
    }
    v_delay(1);
  }
}