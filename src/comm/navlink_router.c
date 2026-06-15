#include "comm/navlink_router.h"
#include "comm/channel.h"
#include "comm/comm_types.h"
#include "comm/serializer.h"  /* comm_rx_raw_drain */
#include "control/pid_config.h"
#include "control/flight_mode.h"           /* flight_mode_apply_command */
#include "control/angle_rate_controller.h" /* geometry apply */
#include "sys/state.h"                      /* system_state_get, SYSTEM_STATE_* */
#include "navhal.h"           /* hal_gpio_write, HAL_GPIO_HIGH/LOW */
#include "sys/sys_utils.h"    /* get_device_id */
#include "utils.h"            /* v_get_ticks, v_memcpy */
#include "variables.h"        /* _BLUE_LED_PIN */
#include "vayu_status.h"
#include "vayu_tasks.h"       /* comm_processor_dispatch */
#include "navlink_msgs.h"     /* the generated codec — included ONLY here */
#include <stdint.h>

extern channel_t g_telemetry_channel; /* defined in telemetry_task.c */

/* -------------------------------------------------------------------------- */
/* Default handler: blue-LED blink, 10 Hz for 1 s, non-reentrant.             */
/* -------------------------------------------------------------------------- */
#define BLINK_MS 1000u
#define BLINK_HALF_MS 50u /* 10 Hz blink => 50 ms half-period (toggle) */

static uint32_t s_blink_end;  /* 0 = idle, else v_get_ticks() at which to stop */
static uint32_t s_blink_last; /* last toggle time */
static uint8_t s_blink_on;

static void blink_start(void) {
  if (s_blink_end != 0u) {
    return; /* already blinking — a consecutive trigger just expires */
  }
  uint32_t now = v_get_ticks();
  s_blink_end = now + BLINK_MS;
  if (s_blink_end == 0u) {
    s_blink_end = 1u; /* keep the idle sentinel free across a tick wrap */
  }
  s_blink_last = now;
  s_blink_on = 1u;
  hal_gpio_write(_BLUE_LED_PIN, HAL_GPIO_HIGH);
}

static void blink_service(void) {
  if (s_blink_end == 0u) {
    return;
  }
  uint32_t now = v_get_ticks();
  if ((int32_t)(now - s_blink_end) >= 0) { /* window elapsed (wrap-safe) */
    s_blink_end = 0u;
    s_blink_on = 0u;
    hal_gpio_write(_BLUE_LED_PIN, HAL_GPIO_LOW);
    return;
  }
  if ((now - s_blink_last) >= BLINK_HALF_MS) {
    s_blink_last = now;
    s_blink_on = (uint8_t)!s_blink_on;
    hal_gpio_write(_BLUE_LED_PIN, s_blink_on ? HAL_GPIO_HIGH : HAL_GPIO_LOW);
  }
}

static void on_default(void *ctx, const navlink_frame_hdr_t *hdr, uint32_t msgid,
                       const uint8_t *payload, size_t len) {
  (void)ctx;
  (void)hdr;
  (void)msgid;
  (void)payload;
  (void)len;
  blink_start();
}

/* -------------------------------------------------------------------------- */
/* Real handlers (registered below; the rest of the comm layer never sees the */
/* codec types).                                                              */
/* -------------------------------------------------------------------------- */
/* Command handlers return their COMMAND_ACK result; the generated dispatch
 * builds and sends the ack (spec §12.1, enforced by codegen), so a handler can
 * never silently drop it — it only reports the result, or defers (returns
 * navlink_ack_deferred()) to send the ack itself once async work resolves. */
#define ACK_OK ((uint8_t)NAVLINK_COMMAND_RESULT_ACCEPTED)
#define ACK_BAD ((uint8_t)NAVLINK_COMMAND_RESULT_FAILED)
#define ACK_BUSY ((uint8_t)NAVLINK_COMMAND_RESULT_TEMPORARILY_REJECTED)

/* CMD_ARM defers its ack: setting the latch doesn't arm — the RC task evaluates
 * the arm gates on its next frame — so the ack is resolved in navlink_router_poll
 * from the flight-state machine (ARMED => ACCEPTED, else timeout => REJECTED). */
static uint8_t s_arm_ack_pending;
static uint8_t s_arm_ack_req_seq;
static uint32_t s_arm_ack_deadline;
#define ARM_ACK_TIMEOUT_MS 800u

/* Rebuild a v1 [cmd_id:2][argc:1][argc x f32] apply payload from typed args. */
static uint8_t build_cmd(uint8_t *p, uint16_t cmd_id, const float *args,
                         uint8_t argc) {
  v_memcpy(&p[0], &cmd_id, 2);
  p[2] = argc;
  if (argc > 0) {
    v_memcpy(&p[3], args, (unsigned)argc * 4u);
  }
  return (uint8_t)(3u + (unsigned)argc * 4u);
}

/* The side-effect-only commands (arm/disarm/calibrate) reuse the tested apply
 * engine via the internal packet_t; the v1 *wire* is gone (packet_t is just the
 * in-memory apply representation). TIME_SYNC / PERF_TASKNAME replies (non-ack)
 * also route here and go back out as v2 through navlink_tx. */
static void dispatch_v1(uint8_t packet_type, const uint8_t *payload,
                        uint8_t length) {
  packet_t pkt = {0};
  pkt.sync = 0x56;
  pkt.protocol_packet_type = (uint8_t)((packet_type << 4) | 0x1);
  pkt.length = length;
  pkt.device_id = get_device_id();
  if (length > 0) {
    v_memcpy(pkt.payload, payload, length);
  }
  comm_processor_dispatch(&pkt);
}

static navlink_ack_t on_cmd_set_pid(void *ctx, const navlink_frame_hdr_t *hdr,
                                    const navlink_cmd_set_pid_t *m) {
  (void)ctx; (void)hdr;
  uint8_t p[3 + 6 * 4];
  float args[6] = {(float)m->controller, (float)m->axis,
                   m->kp, m->ki, m->kd, m->kff};
  uint8_t len = build_cmd(p, (uint16_t)CMD_SET_PID, args, 6);
  return navlink_ack_result(pid_config_apply_command(p, len) == VAYU_OK ? ACK_OK : ACK_BAD);
}

static navlink_ack_t on_cmd_arm(void *ctx, const navlink_frame_hdr_t *hdr,
                                const navlink_cmd_arm_t *m) {
  (void)ctx; (void)hdr;
  uint8_t p[2] = {(uint8_t)CMD_ARM, 0x00}; /* bare cmd_id, no argc (len 2) */
  dispatch_v1(PACKET_TYPE_COMMAND, p, 2);  /* set the latch */
  /* Defer: arming only takes effect (or is gated out) when the RC task next
   * evaluates the latch. navlink_router_poll resolves the ack from the
   * flight-state machine, so the GCS hears the truthful outcome, not "received". */
  s_arm_ack_pending = 1;
  s_arm_ack_req_seq = m->req_seq;
  s_arm_ack_deadline = v_get_ticks() + ARM_ACK_TIMEOUT_MS;
  return navlink_ack_deferred();
}

static navlink_ack_t on_cmd_disarm(void *ctx, const navlink_frame_hdr_t *hdr,
                                   const navlink_cmd_disarm_t *m) {
  (void)ctx; (void)hdr; (void)m;
  uint8_t p[2] = {(uint8_t)CMD_DISARM, 0x00};
  dispatch_v1(PACKET_TYPE_COMMAND, p, 2);
  return navlink_ack_result(ACK_OK);
}

static navlink_ack_t on_cmd_calibrate_imu(void *ctx,
                                          const navlink_frame_hdr_t *hdr,
                                          const navlink_cmd_calibrate_imu_t *m) {
  (void)ctx; (void)hdr;
  if (m->which == 0xFFu) { /* sentinel: cancel calibration (v1 cmd 0x0009) */
    uint8_t p[2] = {0x09, 0x00};
    dispatch_v1(PACKET_TYPE_COMMAND, p, 2);
    return navlink_ack_result(ACK_OK);
  }
  /* Truthful start: reject if a calibration is already running, else kick it
   * off. (Progress/completion stream separately via CALIBRATION_STATUS.) */
  if (system_state_get() == SYSTEM_STATE_CALIBRATING) {
    return navlink_ack_result(ACK_BUSY);
  }
  /* Single IMU: imu_id 0; `which` selects the routine (the v1 `type` arg). */
  uint8_t p[3 + 2 * 4];
  float args[2] = {0.0f, (float)m->which};
  uint8_t len = build_cmd(p, (uint16_t)CMD_CALIBRATE_IMU, args, 2);
  dispatch_v1(PACKET_TYPE_COMMAND, p, len);
  return navlink_ack_result(ACK_OK);
}

static navlink_ack_t
on_cmd_set_gyro_lpf(void *ctx, const navlink_frame_hdr_t *hdr,
                    const navlink_cmd_set_gyro_lpf_t *m) {
  (void)ctx; (void)hdr;
  uint8_t p[3 + 2 * 4];
  float args[2] = {(float)m->axis, m->rc};
  uint8_t len = build_cmd(p, (uint16_t)CMD_SET_GYRO_LPF, args, 2);
  return navlink_ack_result(pid_config_apply_gyro_lpf_command(p, len) == VAYU_OK ? ACK_OK
                                                                     : ACK_BAD);
}

static navlink_ack_t
on_cmd_set_motor_geometry(void *ctx, const navlink_frame_hdr_t *hdr,
                          const navlink_cmd_set_motor_geometry_t *m) {
  (void)ctx; (void)hdr;
  uint8_t p[3 + 12 * 4];
  float args[12];
  for (int i = 0; i < 4; i++) {
    args[i] = m->pos_x[i];
    args[4 + i] = m->pos_y[i];
    args[8 + i] = (float)m->spin[i];
  }
  uint8_t len = build_cmd(p, (uint16_t)CMD_SET_MOTOR_GEOMETRY, args, 12);
  return navlink_ack_result(angle_rate_controller_apply_geometry_command(p, len) ? ACK_OK
                                                                     : ACK_BAD);
}

static navlink_ack_t
on_cmd_set_flight_mode(void *ctx, const navlink_frame_hdr_t *hdr,
                       const navlink_cmd_set_flight_mode_t *m) {
  (void)ctx; (void)hdr;
  uint8_t p[3 + 1 * 4];
  float args[1] = {(float)m->mode}; /* v1 carried mode only; source implied GCS */
  uint8_t len = build_cmd(p, (uint16_t)CMD_SET_FLIGHT_MODE, args, 1);
  return navlink_ack_result(flight_mode_apply_command(p, len) ? ACK_OK : ACK_BAD);
}

static void on_time_sync(void *ctx, const navlink_frame_hdr_t *hdr,
                         const navlink_time_sync_t *m) {
  (void)ctx; (void)hdr;
  time_sync_payload_t in = {0};
  in.role = m->role;
  in.seq = m->seq;
  in.t1_gcs_tx = m->t1_gcs_tx;
  in.t2_fc_rx = m->t2_fc_rx;
  in.t3_fc_tx = m->t3_fc_tx;
  in.commanded_offset_ms = m->commanded_offset_ms;
  dispatch_v1(PACKET_TYPE_TIME_SYNC, (const uint8_t *)&in, (uint8_t)sizeof(in));
}

static void on_perf_taskname_request(void *ctx, const navlink_frame_hdr_t *hdr,
                                     const navlink_perf_taskname_request_t *m) {
  (void)ctx; (void)hdr;
  uint8_t p[1] = {m->task_id};
  dispatch_v1(PACKET_TYPE_PERF_TASKNAME, p, 1);
}

/* -------------------------------------------------------------------------- */
/* The one handler table + parser.                                            */
/* -------------------------------------------------------------------------- */
static navlink_parser_t s_parser;
static navlink_handlers_t s_handlers;

/* Transport the dispatch uses to emit the COMMAND_ACK it builds for every
 * ack-requiring command (codegen-enforced). */
static void router_send(void *ctx, const uint8_t *frame, uint16_t len) {
  (void)ctx;
  write_channel(g_telemetry_channel, (uint8_t *)frame, len);
}

void navlink_router_init(void) {
  navlink_parser_init(&s_parser);
  s_handlers = (navlink_handlers_t){0};
  s_handlers.send = router_send; /* required: commands auto-ack via this */
  s_handlers.sysid = get_device_id();
  s_handlers.compid = 1;
  s_handlers.on_default = on_default; /* every unhandled leaf -> blink */
  s_handlers.on_cmd_set_pid = on_cmd_set_pid;
  s_handlers.on_cmd_arm = on_cmd_arm;
  s_handlers.on_cmd_disarm = on_cmd_disarm;
  s_handlers.on_cmd_calibrate_imu = on_cmd_calibrate_imu;
  s_handlers.on_cmd_set_gyro_lpf = on_cmd_set_gyro_lpf;
  s_handlers.on_cmd_set_motor_geometry = on_cmd_set_motor_geometry;
  s_handlers.on_cmd_set_flight_mode = on_cmd_set_flight_mode;
  s_handlers.on_time_sync = on_time_sync;
  s_handlers.on_perf_taskname_request = on_perf_taskname_request;
}

/* Resolve a deferred CMD_ARM ack from the flight-state machine: ACCEPTED once it
 * actually armed, TEMPORARILY_REJECTED if the gates kept it from arming in time. */
static void arm_ack_service(void) {
  if (!s_arm_ack_pending) {
    return;
  }
  if (system_state_get() == SYSTEM_STATE_ARMED) {
    navlink_command_ack_send(&s_handlers, NAVLINK_MSGID_CMD_ARM, s_arm_ack_req_seq,
                             navlink_ack_result(ACK_OK));
    s_arm_ack_pending = 0;
  } else if ((int32_t)(v_get_ticks() - s_arm_ack_deadline) >= 0) {
    navlink_command_ack_send(&s_handlers, NAVLINK_MSGID_CMD_ARM, s_arm_ack_req_seq,
                             navlink_ack_result(ACK_BUSY));
    s_arm_ack_pending = 0;
  }
}

void navlink_router_poll(void) {
  uint8_t buf[256];
  uint16_t n = comm_rx_raw_drain(buf, (uint16_t)sizeof(buf));
  if (n > 0) {
    navlink_parser_push(&s_parser, &s_handlers, buf, n);
  }
  arm_ack_service();
  blink_service();
}
