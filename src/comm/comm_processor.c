#include "comm/comm_types.h"
#include "comm/ibus.h"
#include "comm/perf_packet.h"
#include "comm/serializer.h"
#include "control/control.h"
#include "control/flight_mode.h"
#include "memory.h"
#include "sensor/sensor.h"
#include "sys/state.h"
#include "task.h"
#include "utils.h"
#include "sys/sys_utils.h"
#include "vaios.h"
#include "variables.h"
#include "vayu_status.h"
#include "vayu_tasks.h"
#include "vayu_tasks.h"
#include "control/pid_config.h"
#include "navlink_msgs.h"
#include <stdbool.h>
#include <stdint.h>
static uint32_t _calibration_task_handle = 0;

/* COMM-CMD-002: a command payload is laid out as
 *   [cmd_id:2][argc:1][arg0:4][arg1:4]...
 * so reading `argc` 4-byte args requires the framing layer to have
 * delivered at least argc*4 + 3 payload bytes. Validate both argc and the
 * payload length before any handler dereferences an argument. `len` is
 * pkt.length — the payload byte count produced by the deserializer.
 * @implements COMM-CMD-002 */
static bool command_payload_valid(uint16_t len, uint8_t argc,
                                  uint8_t expected_argc) {
  if (argc < expected_argc) {
    return false;
  }
  return len >= (uint16_t)argc * 4u + 3u;
}
/* Dispatch a single received packet to its handler. Extracted from
 * comm_processor_task's loop so the GCS -> FC command path can be exercised
 * end-to-end in SITL (deserialize a real GCS frame -> dispatch), independent
 * of the blocking RX queue. */
void comm_processor_dispatch(const packet_t *pkt) {
  uint8_t packet_type = (pkt->protocol_packet_type >> 4) & 0x0F;

  if (packet_type == PACKET_TYPE_HEARTBEAT) {
    /* Heartbeat is now liveness + device-id only; clock alignment moved to the
     * time-sync handshake below (no more one-shot offset jam). */
    set_device_id(pkt->device_id);
  } else if (packet_type == PACKET_TYPE_TIME_SYNC &&
             pkt->length >= sizeof(time_sync_payload_t)) {
    /* NTP-style sync (docs/telemetry/time_sync.md). Capture the receive time
     * first, reply echoing t1 with t2 (rx) and t3 (tx), and apply any
     * GCS-commanded clock correction via the slewed discipline. */
    time_sync_payload_t in;
    v_memcpy(&in, pkt->payload, sizeof(in));
    if (in.role == TIME_SYNC_REQUEST) {
      /* Apply the correction FIRST, then stamp t2/t3 from the corrected clock,
       * so this exchange's response already reflects the command. The GCS then
       * measures the *post-correction* residual and won't re-send a correction
       * it has already applied — without this the loop double-applies and
       * oscillates. t2/t3 are taken back-to-back so they're consistent (no
       * step landing between them). */
      if (in.commanded_offset_ms != INT32_MIN) {
        time_sync_set_offset(in.commanded_offset_ms);
      }
      time_sync_payload_t out = {0};
      out.role = TIME_SYNC_RESPONSE;
      out.seq = in.seq;
      out.t1_gcs_tx = in.t1_gcs_tx;
      out.t2_fc_rx = (uint64_t)get_timestamp_unix();
      out.commanded_offset_ms = INT32_MIN;
      out.t3_fc_tx = (uint64_t)get_timestamp_unix();
      send_packet(&g_telemetry_channel, PACKET_TYPE_TIME_SYNC, (uint8_t *)&out,
                  (uint8_t)sizeof(out));
    }
  } else if (packet_type == PACKET_TYPE_PERF_TASKNAME && pkt->length >= 1) {
    /* GCS asked for one task's name by id; reply [id][name\0]. Name is a
     * borrowed flash pointer in the TCB, copied bounded + NUL-terminated. */
    uint8_t id = pkt->payload[0];
    const char *nm = task_get_name_by_id(id);
    uint8_t buf[1 + PERF_TASKNAME_MAX];
    buf[0] = id;
    uint8_t n = 0;
    while (n < PERF_TASKNAME_MAX - 1 && nm[n]) {
      buf[1 + n] = (uint8_t)nm[n];
      n++;
    }
    buf[1 + n] = '\0';
    send_packet(&g_telemetry_channel, PACKET_TYPE_PERF_TASKNAME, buf,
                (uint8_t)(2 + n));
  } else if (packet_type == PACKET_TYPE_COMMAND && pkt->length >= 2) {
    uint16_t cmd_id;
    v_memcpy(&cmd_id, pkt->payload, 2);
    /* argc is payload[2]; only present when length >= 3. */
    uint8_t argc = (pkt->length >= 3) ? pkt->payload[2] : 0;

    if (cmd_id == CMD_CALIBRATE_IMU) {
      /* COMM-CMD-002: need 2 args (imu_id, type) -> argc>=2 and a
       * payload long enough to actually hold them. */
      if (system_state_get() != SYSTEM_STATE_CALIBRATING &&
          command_payload_valid(pkt->length, argc, 2)) {
        calibration_args_t *cal_args =
            (calibration_args_t *)v_malloc(sizeof(calibration_args_t));
        if (cal_args != NULL) {
          v_memcpy(&cal_args->imu_id, &pkt->payload[3], 4);
          v_memcpy(&cal_args->type, &pkt->payload[7], 4);
        }
        _calibration_task_handle =
            task_create(calibration_task, cal_args, 4096, 0);
      }
    } else if (cmd_id == 0x0009) { // CMD_CANCEL_CALIBRATION
      if (_calibration_task_handle != 0) {
        task_exit_request(_calibration_task_handle);
        _calibration_task_handle = 0;
      }
      VAYU_DISCARD(system_state_set(SYSTEM_STATE_STANDBY));
    } else if (cmd_id == CMD_ARM) {
      /* GCS software-arm: set the latch. The RC task evaluates it (OR'd
       * with the physical arm switch via rc_arm_engaged) against the arm
       * preconditions on its next frame, so throttle/link/estimator gates
       * still apply. Lets a 4-channel stick with no arm channel arm from
       * the GCS. */
      g_sw_arm_request = 1;
    } else if (cmd_id == CMD_DISARM) {
      /* Clear the latch; the RC task disarms to STANDBY on its next frame
       * (mirrors releasing the arm switch). */
      g_sw_arm_request = 0;
    } else if (cmd_id == CMD_SET_PID) {
      /* COMM-CMD-003: validate, apply to the live controller, persist. */
      VAYU_DISCARD(pid_config_apply_command(pkt->payload, pkt->length));
    } else if (cmd_id == CMD_SET_GYRO_LPF) {
      /* Live rate-loop gyro LPF update (co-tuned with the gains). */
      VAYU_DISCARD(pid_config_apply_gyro_lpf_command(pkt->payload, pkt->length));
    } else if (cmd_id == CMD_SET_MOTOR_GEOMETRY) {
      /* Set the mixer signs from the airframe motor layout (sim/vehicle). */
      VAYU_DISCARD(
          angle_rate_controller_apply_geometry_command(pkt->payload, pkt->length));
    } else if (cmd_id == CMD_SET_FLIGHT_MODE) {
      /* GCS stabilise/acro override (arg 0=angle, 1=acro, 2=release to RC). */
      VAYU_DISCARD(flight_mode_apply_command(pkt->payload, pkt->length));
    }
  }
}

/* ---- NavLink v2 uplink (GCS -> FC) ------------------------------------------
 * Phase 3 of the v2 migration (navlink/INTEGRATION.md). v2 command frames ride
 * the same telemetry UART as v1 (sync 0x56, byte1 == 0x02); the RX ISR mirrors
 * every byte into a raw ring (serializer.c) which we drain here, in task
 * context, through the generated parser. v1 frames in the stream are ignored by
 * the v2 parser and still handled by comm_processor_dispatch(); v2 frames are
 * dispatched to the handlers below. First migrated command: CMD_SET_PID. */
static navlink_parser_t g_v2_parser;
static navlink_handlers_t g_v2_handlers;

static void v2_on_cmd_set_pid(void *ctx, const navlink_frame_hdr_t *hdr,
                              const navlink_cmd_set_pid_t *m) {
  (void)ctx;
  (void)hdr;
  /* Reuse the tested v1 apply path: rebuild the v1 command payload
   *   [cmd_id:2][argc:1][controller, axis, kp, ki, kd, kff : f32 x6]
   * and call pid_config_apply_command (COMM-CMD-003). */
  uint8_t payload[3 + 6 * 4];
  uint16_t cmd_id = (uint16_t)CMD_SET_PID;
  v_memcpy(&payload[0], &cmd_id, 2);
  payload[2] = 6; /* argc */
  float args[6] = {(float)m->controller, (float)m->axis,
                   m->kp, m->ki, m->kd, m->kff};
  v_memcpy(&payload[3], args, sizeof(args));
  vayu_status_t st = pid_config_apply_command(payload, sizeof(payload));

  /* Reply with a v2 COMMAND_ACK — the observable for live validation. */
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

static void comm_v2_init(void) {
  navlink_parser_init(&g_v2_parser);
  g_v2_handlers = (navlink_handlers_t){0};
  g_v2_handlers.on_cmd_set_pid = v2_on_cmd_set_pid;
}

void comm_processor_task(void *args) {
  (void)args;
  packet_t pkt;
  static uint8_t v2_rx[256];

  comm_v2_init();

  while (1) {
    /* Drain raw RX bytes through the v2 parser before the v1 queue. */
    uint16_t got = comm_rx_raw_drain(v2_rx, (uint16_t)sizeof(v2_rx));
    if (got > 0) {
      navlink_parser_push(&g_v2_parser, &g_v2_handlers, v2_rx, got);
    }

    if (get_next_rx_packet(&pkt) == NONE) {
      comm_processor_dispatch(&pkt);
    } else {
      v_delay(4); // Wait for more packets
    }
  }
}
