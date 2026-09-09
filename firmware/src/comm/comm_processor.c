#include "comm/comm_types.h"
#include "comm/ibus.h"
#include "comm/navlink_tx.h"
#include "comm/perf_packet.h"
#include "comm/serializer.h"
#include "control/control.h"
#include "control/flight_mode.h"
#include "memory.h"
#include "sensor/sensor.h"
#include "storage/fs_owner.h" /* vayu_log */
#include "sys/state.h"
#include "task.h"
#include "utils.h"
#include "sys/sys_utils.h"
#include "vaios.h"
#include "variables.h"
#include "vayu_status.h"
#include "vayu_tasks.h"
#include "vayu_tasks.h"
#include "comm/navlink_router.h"
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
/** @implements COMM-CMD-001, COMM-CMD-003, COMM-HB-001 */
void comm_processor_dispatch(const packet_t *pkt) {
  uint8_t packet_type = (pkt->protocol_packet_type >> 4) & 0x0F;

  if (packet_type == PACKET_TYPE_HEARTBEAT) {
    /* Heartbeat carries liveness + device-id only; clock alignment is handled
     * by the time-sync handshake below. */
    set_device_id(pkt->device_id);
  } else if (packet_type == PACKET_TYPE_TIME_SYNC &&
             pkt->length >= sizeof(time_sync_payload_t)) {
    /* NTP-style sync (docs/telemetry/time_sync.md). Capture the receive time
     * first, reply echoing t1 with t2 (rx) and t3 (tx), and apply any
     * GCS-commanded clock correction via the slewed discipline. */
    time_sync_payload_t in;
    v_memcpy(&in, pkt->payload, sizeof(in));
    if (in.role == TIME_SYNC_REQUEST || in.role == TIME_SYNC_REQUEST_WIDE) {
      /* Apply the correction FIRST, then stamp t2/t3 from the corrected clock,
       * so this exchange's response already reflects the command. The GCS then
       * measures the *post-correction* residual and won't re-send a correction
       * it has already applied — without this the loop double-applies and
       * oscillates. t2/t3 are taken back-to-back so they're consistent (no
       * step landing between them). */
      if (in.role == TIME_SYNC_REQUEST_WIDE) {
        /* Full 64-bit correction split across (hi, lo) — used when the FC<->GCS
         * deviation exceeds int32 ms (e.g. uptime clock vs GCS epoch). */
        int64_t corr = ((int64_t)in.commanded_offset_hi_ms << 32) |
                       (int64_t)(uint32_t)in.commanded_offset_ms;
        time_sync_set_offset64(corr);
      } else if (in.commanded_offset_ms != INT32_MIN) {
        time_sync_set_offset(in.commanded_offset_ms);
      }
      time_sync_payload_t out = {0};
      out.role = TIME_SYNC_RESPONSE;
      out.seq = in.seq;
      out.t1_gcs_tx = in.t1_gcs_tx;
      out.t2_fc_rx = (uint64_t)get_timestamp_unix();
      out.commanded_offset_ms = INT32_MIN;
      out.t3_fc_tx = (uint64_t)get_timestamp_unix();
      navlink_tx_time_sync_response(&out);
    }
  } else if (packet_type == PACKET_TYPE_PERF_TASKNAME && pkt->length >= 1) {
    /* GCS asked for one task's name by id; reply [id][name\0]. */
    uint8_t id = pkt->payload[0];
    navlink_tx_perf_taskname(id, task_get_name_by_id(id));
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
        /* 
         * Stack: 3072, about 2x the measured requirement. The deepest chain is
         * calibration_task(400) -> calib_engine_run(104) -> run_ellipsoid(488)
         * -> calib_fit_ellipsoid(272) = 1264 B by -fstack-usage, plus ~200 B of
         * exception frame.*/
        calibration_args_t *cal_args =
            (calibration_args_t *)v_malloc(sizeof(calibration_args_t));
        if (cal_args == NULL) {
          vayu_log("[CALIB] out of heap for args; not starting");
        } else {
          v_memcpy(&cal_args->imu_id, &pkt->payload[3], 4);
          v_memcpy(&cal_args->type, &pkt->payload[7], 4);
          _calibration_task_handle =
              task_create(calibration_task, cal_args, 3072, 0);
          if (_calibration_task_handle == 0) {
            /* task_create returns 0 when the TCB alloc fails (a failed STACK
             * alloc panics inside the kernel). Nothing will ever free the arg
             * block, so do it here. */
            v_free(cal_args);
            vayu_log("[CALIB] out of heap for task; not starting");
          }
        }
      }
    } else if (cmd_id == 0x0009) { // CMD_CANCEL_CALIBRATION
      /* Cooperative cancel: raise the flag the calibration task polls at each
       * loop boundary so it tears down cleanly (restores STANDBY, frees args)
       * instead of being killed mid-run. The task restores state itself. */
      if (_calibration_task_handle != 0) {
        bmx160_calib_request_cancel();
        _calibration_task_handle = 0;
      } else {
        // No task running; restore state directly in case it was left stuck.
        VAYU_DISCARD(system_state_set(SYSTEM_STATE_STANDBY));
      }
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

/** @noreq comm RX task loop wrapper (drives navlink_router_poll) */
void comm_processor_task(void *args) {
  (void)args;
  packet_t pkt;

  /* Pure NavLink v2 uplink (GCS -> FC): the RX ISR mirrors every byte into a raw
   * ring (serializer.c) which navlink_router_poll() drains through the generated
   * parser + handler table in task context. comm_processor_dispatch() is the
   * in-memory command apply engine that navlink_router.c reuses. */
  (void)pkt;
  navlink_router_init();

  while (1) {
    navlink_router_poll();
    v_delay(4);
  }
}
