#include "comm/navlink_router.h"
#include "comm/channel.h"
#include "comm/comm_types.h"
#include "comm/serializer.h"  /* comm_rx_raw_drain */
#include "control/pid_config.h"
#include "navhal.h"           /* hal_gpio_write, HAL_GPIO_HIGH/LOW */
#include "sys/sys_utils.h"    /* get_device_id */
#include "utils.h"            /* v_get_ticks, v_memcpy */
#include "variables.h"        /* _BLUE_LED_PIN */
#include "vayu_status.h"
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
static void on_cmd_set_pid(void *ctx, const navlink_frame_hdr_t *hdr,
                           const navlink_cmd_set_pid_t *m) {
  (void)ctx;
  (void)hdr;
  /* Reuse the tested v1 apply path: rebuild [cmd_id:2][argc:1][6 x f32]. */
  uint8_t payload[3 + 6 * 4];
  uint16_t cmd_id = (uint16_t)CMD_SET_PID;
  v_memcpy(&payload[0], &cmd_id, 2);
  payload[2] = 6;
  float args[6] = {(float)m->controller, (float)m->axis,
                   m->kp, m->ki, m->kd, m->kff};
  v_memcpy(&payload[3], args, sizeof(args));
  vayu_status_t st = pid_config_apply_command(payload, sizeof(payload));

  navlink_command_ack_t ack = {0};
  ack.command = NAVLINK_MSGID_CMD_SET_PID;
  ack.req_seq = m->req_seq;
  ack.result = (st == VAYU_OK) ? (uint8_t)NAVLINK_COMMAND_RESULT_ACCEPTED
                               : (uint8_t)NAVLINK_COMMAND_RESULT_FAILED;
  static uint8_t s_ack_seq = 0;
  uint8_t frame[NAVLINK_MAX_FRAME];
  size_t n = navlink_command_ack_encode(frame, &ack, s_ack_seq++,
                                        get_device_id(), 1);
  write_channel(g_telemetry_channel, frame, (uint16_t)n);
}

/* -------------------------------------------------------------------------- */
/* The one handler table + parser.                                            */
/* -------------------------------------------------------------------------- */
static navlink_parser_t s_parser;
static navlink_handlers_t s_handlers;

void navlink_router_init(void) {
  navlink_parser_init(&s_parser);
  s_handlers = (navlink_handlers_t){0};
  s_handlers.on_default = on_default;         /* every unhandled leaf -> blink */
  s_handlers.on_cmd_set_pid = on_cmd_set_pid; /* override: real handler */
}

void navlink_router_poll(void) {
  uint8_t buf[256];
  uint16_t n = comm_rx_raw_drain(buf, (uint16_t)sizeof(buf));
  if (n > 0) {
    navlink_parser_push(&s_parser, &s_handlers, buf, n);
  }
  blink_service();
}
