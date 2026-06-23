#include "comm/navlink_router.h"
#include "comm/channel.h"
#include "comm/comm_types.h"
#include "comm/serializer.h"  /* comm_rx_raw_drain */
#include "control/pid_config.h"
#include "control/flight_mode.h"           /* flight_mode_apply_command */
#include "control/angle_rate_controller.h" /* geometry apply */
#include "control/sysid.h"                  /* sysid_start/abort (CMD_SYSID_EXCITE) */
#include "sys/state.h"                      /* system_state_get, SYSTEM_STATE_* */
#include "navhal.h"           /* hal_gpio_write, HAL_GPIO_HIGH/LOW */
#include "sys/sys_utils.h"    /* get_device_id */
#include "utils.h"            /* v_get_ticks, v_memcpy */
#include "variables.h"        /* _BLUE_LED_PIN */
#include "vayu_status.h"
#include "vayu_tasks.h"       /* comm_processor_dispatch */
#include "comm/xfer/navlink_xfer.h" /* bulk-transfer substrate SM (codec-blind) */
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

static navlink_ack_t on_cmd_sysid_excite(void *ctx, const navlink_frame_hdr_t *hdr,
                                         const navlink_cmd_sysid_excite_t *m) {
  (void)ctx; (void)hdr;
  /* axis 0xFF is the abort sentinel; any other out-of-range axis is rejected.
   * Otherwise start a chirp (sysid_start clamps amp/freq/duration to hard caps).
   * Injection only reaches the motors through the controller's ARMED gate, so a
   * disarmed FC accepts this and shows the chirp in CONTROL_TRACE with props
   * still. The run self-aborts on angle/rate limits and at duration. */
  if (m->axis == 0xFFu) {
    sysid_abort();
    return navlink_ack_result(ACK_OK);
  }
  if (m->axis > 2u)
    return navlink_ack_result(ACK_BAD);
  sysid_request_t req = {.axis = m->axis,
                         .f0_hz = m->f0_hz,
                         .f1_hz = m->f1_hz,
                         .amp_dps = m->amp_dps,
                         .duration_s = m->duration_s};
  sysid_start(&req);
  return navlink_ack_result(ACK_OK);
}

static navlink_ack_t on_cmd_sysid_dump(void *ctx, const navlink_frame_hdr_t *hdr,
                                       const navlink_cmd_sysid_dump_t *m) {
  (void)ctx; (void)hdr; (void)m;
  /* Arm the dump cursor; the telemetry task streams SYSID_SAMPLE chunks. Reads
   * the capture buffer that the (now-finished) run filled, so no race. */
  sysid_dump_request();
  return navlink_ack_result(ACK_OK);
}

/* ---- xfer (FTP) substrate: map codec structs -> the codec-blind SM --------
 * on_xfer_open/close run here on the comm task (state only). A valid open is
 * deferred: the SM stashes it and xfer_service_task runs provider->open then
 * emits COMMAND_ACK + XFER_INFO (off the comm task — the C1->C3 invariant).
 * xfer_result_t mirrors command_result by value, so an immediate disposition
 * passes straight to navlink_ack_result(). */
static navlink_ack_t on_xfer_open(void *ctx, const navlink_frame_hdr_t *hdr,
                                  const navlink_xfer_open_t *m) {
  (void)ctx;
  xfer_open_args_t a = {0};
  a.session = m->session;
  a.dir = m->dir;
  a.mode = m->mode;
  a.req_seq = m->req_seq;
  a.gcs_sys = hdr->sysid;   /* stamp the reply target from the frame header */
  a.gcs_comp = hdr->compid;
  a.service_id = m->service_id;
  a.offset_start = m->offset_start;
  a.rate_hz = m->rate_hz;
  for (uint8_t i = 0; i < XFER_ARG_MAX; i++)
    a.arg[i] = m->arg[i];
  int d = xfer_on_open(&a);
  if (d == XFER_OPEN_DEFERRED)
    return navlink_ack_deferred();
  return navlink_ack_result((uint8_t)d);
}

static navlink_ack_t on_xfer_close(void *ctx, const navlink_frame_hdr_t *hdr,
                                   const navlink_xfer_close_t *m) {
  (void)ctx; (void)hdr;
  int d = xfer_on_close(m->session, m->req_seq, m->result);
  if (d == XFER_OPEN_DEFERRED)
    return navlink_ack_deferred();
  return navlink_ack_result((uint8_t)d);
}

static void on_xfer_data(void *ctx, const navlink_frame_hdr_t *hdr,
                         const navlink_xfer_data_t *m) {
  (void)ctx; (void)hdr;
  xfer_on_data(m->session, m->offset, m->data, m->len, m->flags);
}

static void on_xfer_ack(void *ctx, const navlink_frame_hdr_t *hdr,
                        const navlink_xfer_ack_t *m) {
  (void)ctx; (void)hdr;
  xfer_on_ack(m->session, m->next_offset, m->flags);
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
  /* Truthful start: calibration can only be entered from STANDBY or FAILSAFE
   * (see sys/state.c). Reject (temporarily) from anywhere else — ARMED/IN_AIR
   * or already CALIBRATING — instead of ACCEPTing a command that can't run.
   * (Progress/completion stream separately via CALIBRATION_STATUS.) */
  sys_state_t st = system_state_get();
  if (st != SYSTEM_STATE_STANDBY && st != SYSTEM_STATE_FAILSAFE) {
    return navlink_ack_result(ACK_BUSY);
  }
  /* `which` packs the sensor selector and mode into one byte: high nibble =
   * imu_id (1=accel, 2=gyro, 3=mag), low nibble = type (0=bias, 1=full). Unpack
   * both for the v1 dispatch, which keys calibration_task on (imu_id, type). */
  uint8_t imu = (uint8_t)((m->which >> 4) & 0x0Fu);
  uint8_t typ = (uint8_t)(m->which & 0x0Fu);
  uint8_t p[3 + 2 * 4];
  float args[2] = {(float)imu, (float)typ};
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
  in.commanded_offset_hi_ms = m->commanded_offset_hi_ms;
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

/* §10.5 security gate: an unsynchronised FC MUST reject every command. The
 * dispatch consults this before EVERY command handler (codegen-enforced), so the
 * rule holds for current and future commands without per-handler checks. Until
 * the GCS has disciplined our clock (time-sync handshake, §10), commands are
 * TEMPORARILY_REJECTED — a soft "retry after sync", not a hard failure — so the
 * GCS re-issues once synced rather than surfacing a permanent error. */
static navlink_ack_t router_command_gate(void *ctx, uint32_t command) {
  (void)ctx;
  (void)command;
  return navlink_ack_result(time_sync_is_synced() ? ACK_OK : ACK_BUSY);
}

void navlink_router_init(void) {
  navlink_parser_init(&s_parser);
  s_handlers = (navlink_handlers_t){0};
  s_handlers.send = router_send; /* required: commands auto-ack via this */
  s_handlers.sysid = get_device_id();
  s_handlers.compid = 1;
  s_handlers.command_gate = router_command_gate; /* §10.5: reject until synced */
  s_handlers.on_default = on_default; /* every unhandled leaf -> blink */
  s_handlers.on_cmd_set_pid = on_cmd_set_pid;
  s_handlers.on_cmd_sysid_excite = on_cmd_sysid_excite;
  s_handlers.on_cmd_sysid_dump = on_cmd_sysid_dump;
  s_handlers.on_cmd_arm = on_cmd_arm;
  s_handlers.on_cmd_disarm = on_cmd_disarm;
  s_handlers.on_cmd_calibrate_imu = on_cmd_calibrate_imu;
  s_handlers.on_cmd_set_gyro_lpf = on_cmd_set_gyro_lpf;
  s_handlers.on_cmd_set_motor_geometry = on_cmd_set_motor_geometry;
  s_handlers.on_cmd_set_flight_mode = on_cmd_set_flight_mode;
  s_handlers.on_time_sync = on_time_sync;
  s_handlers.on_perf_taskname_request = on_perf_taskname_request;
  s_handlers.on_xfer_open = on_xfer_open;
  s_handlers.on_xfer_close = on_xfer_close;
  s_handlers.on_xfer_data = on_xfer_data;
  s_handlers.on_xfer_ack = on_xfer_ack;
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
