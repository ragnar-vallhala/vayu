#ifndef VAYU_SIM_HOST_RC_FEEDER_H
#define VAYU_SIM_HOST_RC_FEEDER_H

/* Spawn the RC-feeder pthread.
 *
 * Reads CSV channel lines (us per channel) from a serial port written
 * by tools/sim_bridge/ (an Arduino decoding FS-iA6B PPM into the
 * standard FS-i6 channel order). If the port can't be opened or
 * disappears, falls back to a synthetic hover frame so the host SITL
 * stays alive standalone.
 *
 * Serial port path comes from the VAYU_UART_RC_PATH environment
 * variable; defaults to /dev/ttyUSB0. */
void host_rc_feeder_start(void);

#endif
