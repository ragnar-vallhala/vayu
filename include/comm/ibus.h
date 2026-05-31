#ifndef VAYU_COMM_IBUS_H
#define VAYU_COMM_IBUS_H

#include <stdbool.h>
#include <stdint.h>

#define IBUS_MAX_CHANNELS 14
#define IBUS_PACKET_SIZE 32
#define IBUS_START_BYTE 0x20
#define IBUS_CMD_CHANNELS 0x40

typedef struct {
  uint16_t channels[IBUS_MAX_CHANNELS];
  bool is_failsafe;
} ibus_data_t;

typedef enum {
  IBUS_STATE_WAIT_START,
  IBUS_STATE_WAIT_CMD,
  IBUS_STATE_PAYLOAD,
  IBUS_STATE_CHECKSUM_L,
  IBUS_STATE_CHECKSUM_H
} ibus_state_t;

void ibus_init(ibus_data_t *data);
bool ibus_parse_byte(uint8_t byte, ibus_data_t *data);

/* ----------------------------------------------------------------------------
 * RC link watchdog (SYS-SAFE-002 / COMM-RC-002)
 * --------------------------------------------------------------------------*/

/** Failsafe horizon (ms): RC loss this long drives FAILSAFE (SYS-SAFE-002). */
#define RC_LOSS_TIMEOUT_MS 1000U

/** Fast COMM-layer detect horizon (ms): rc_loss() trips this quickly so
 *  higher layers can react before the slower failsafe transition (COMM-RC-002). */
#define RC_LOSS_DETECT_MS 100U

/**
 * @brief Mark "right now" as the most recent valid RC frame.
 *
 * Called by the iBUS task whenever a complete frame parses cleanly.
 * Lock-free single 32-bit aligned store — safe from any context.
 *
 * @implements COMM-RC-002
 */
void rc_mark_frame_valid(void);

/**
 * @brief Predicate: was a valid RC frame received within
 *        RC_LOSS_TIMEOUT_MS (1.0 s) of now? Backs the failsafe gate.
 *
 * Lock-free; safe to call from the rate-loop hot path (R8.6).
 *
 * @implements SYS-SAFE-002
 */
bool rc_has_signal(void);

/**
 * @brief Predicate: has the RC link been lost for more than
 *        RC_LOSS_DETECT_MS (100 ms)? The fast COMM-layer rc_loss flag,
 *        distinct from (and tripping well before) the 1.0 s failsafe.
 *
 * Lock-free; safe from any context (R8.6).
 *
 * @implements COMM-RC-002
 */
bool rc_loss(void);

/**
 * @brief One RC-watchdog tick: if the link is lost in a flight-relevant
 *        state, request SYSTEM_STATE_FAILSAFE. No-op otherwise.
 *
 * Defined in src/comm/rc_safety.c.
 *
 * @implements SYS-SAFE-002
 */
void rc_watchdog_step(void);

/**
 * @brief STANDBY -> ARMED precondition gate: throttle at minimum, RC
 *        link healthy, estimator not degraded. Defined in
 *        src/comm/rc_safety.c.
 *
 * @implements SYS-SAFE-005, CTRL-ARM-001
 */
bool arm_preconditions_met(const ibus_data_t *rc);

/**
 * @brief GCS software-arm latch. Set by CMD_ARM, cleared by CMD_DISARM
 *        (src/comm/comm_processor.c). OR'd with the RC arm switch by
 *        rc_arm_engaged() so the vehicle can be armed from the GCS when no
 *        physical arm channel is available (e.g. a 4-channel USB-HID stick in
 *        SITL). Defined in src/comm/rc_safety.c.
 */
extern volatile uint8_t g_sw_arm_request;

/**
 * @brief Unified arm-request predicate: true when the RC arm switch
 *        (channel 5 > 1500) OR the GCS software-arm latch is engaged.
 *        Arm preconditions (throttle, link, estimator) are still enforced
 *        separately by the caller. Defined in src/comm/rc_safety.c.
 */
bool rc_arm_engaged(const ibus_data_t *rc);

#ifdef VAYU_SIM
/**
 * SITL-only fault-injection hook. When non-zero, the iBUS task
 * suppresses rc_mark_frame_valid() calls so a SITL scenario can
 * simulate RC loss without disabling the synth feed.
 */
extern volatile uint8_t sim_rc_force_loss;
#endif

#endif // VAYU_COMM_IBUS_H
