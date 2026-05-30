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

/** Time horizon (ms) within which a valid RC frame must be received. */
#define RC_LOSS_TIMEOUT_MS 1000U

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
 *        RC_LOSS_TIMEOUT_MS of now?
 *
 * Lock-free; safe to call from the rate-loop hot path (R8.6).
 *
 * @implements COMM-RC-002, SYS-SAFE-002
 */
bool rc_has_signal(void);

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

#ifdef VAYU_SIM
/**
 * SITL-only fault-injection hook. When non-zero, the iBUS task
 * suppresses rc_mark_frame_valid() calls so a SITL scenario can
 * simulate RC loss without disabling the synth feed.
 */
extern volatile uint8_t sim_rc_force_loss;
#endif

#endif // VAYU_COMM_IBUS_H
