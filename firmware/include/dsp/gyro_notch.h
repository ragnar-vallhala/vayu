#ifndef VAYU_DSP_GYRO_NOTCH_H
#define VAYU_DSP_GYRO_NOTCH_H

/* gyro_notch.h — firmware glue that owns the per-axis dynamic notch banks.
 *
 * This is the layer between the pure DSP (dsp/notch_bank, dsp/notch_fft,
 * maths/biquad) and the rate loop. It v_malloc's every working buffer at init
 * so nothing lands in .bss (the F401 boot-HardFault invariant — see
 * f401-sram-heap-overflow), runs one notch bank per gyro axis, and bounds the
 * per-tick cost: notch_bank_filter runs on every sample, but the expensive FFT
 * analysis + retune is staggered to at most one axis per rate-loop tick.
 *
 * The biquads always run at INNER_LOOP_FREQ_HZ (the rate loop); the FFT analyzer
 * runs at a decimated rate D = floor(loop / ~1 kHz) so its Nyquist stays safely
 * above the analysis band no matter how fast the loop ticks. At the current 1 kHz
 * loop D == 1 and the two rates are equal; raising INNER_LOOP_FREQ_HZ above ~2 kHz
 * transparently engages decimation (observe every Dth sample, analyzer fs = loop/D).
 * Disabled by default: no effect on the gyro stream until
 * gyro_notch_set_enabled(true), mirroring the gyro-LPF's "off until tuned"
 * policy so flight behaviour is unchanged until validated.
 *
 * Not thread-safe by construction — apply/service must run on the SAME task (the
 * rate loop) so the biquad coefficients are never written by one task while read
 * by another. Design ref: reference-autopilots-comparison.md §9.3 / §10.
 */
#include <stdbool.h>
#include <stdint.h>

/* Allocate and initialise the per-axis notch banks on the heap. Idempotent:
 * a second call is a no-op. Returns true on success; false if allocation failed
 * (the module then stays inert — apply/service pass the gyro through untouched).
 * Starts DISABLED regardless of success. */
bool gyro_notch_init(void);

/* Runtime enable/disable of the filtering. Analysis keeps running while disabled
 * (so the spectrum is warm on enable); only the filter output is gated. */
void gyro_notch_set_enabled(bool enabled);
bool gyro_notch_enabled(void);

/* Feed the current throttle (0..1) each tick. The notch only engages (filters +
 * analyses) above an internal throttle threshold, where the prop vibration it
 * targets actually exists; below it the filter passes the gyro through and the
 * banks are reset on the disengage edge. */
void gyro_notch_set_throttle(float throttle01);

/* Hot path: call once per axis (0..NUM_AXES-1) per rate-loop tick with the raw
 * gyro rate. Feeds the analyzer and, when enabled, returns the notch-filtered
 * rate. Returns the input unchanged when disabled, uninitialised, or axis is out
 * of range. */
float gyro_notch_apply(uint8_t axis, float gyro);

/* Call once per rate-loop tick, after the per-axis gyro_notch_apply calls. Runs
 * at most one FFT analysis + retune (round-robin over axes) so the analysis cost
 * is amortised across ticks and never blocks the loop for more than one FFT. */
void gyro_notch_service(void);

/* Live-tune the detection parameters across all axes (applied at the next
 * retune): notch Q, the analysis band [fmin_hz, fmax_hz], and the peak
 * prominence gate min_ratio. A non-positive argument leaves that field
 * unchanged, so a caller can set one field at a time. No-op if uninitialised. */
void gyro_notch_set_params(float q, float fmin_hz, float fmax_hz,
                           float min_ratio);

/* Telemetry: the idx-th tuned notch center frequency [Hz] for an axis, or 0 if
 * that slot is currently bypassed / out of range. */
float gyro_notch_center_hz(uint8_t axis, uint8_t idx);

/* The analyzer decimation factor D actually in use (analyzer fs = loop rate / D),
 * or 0 if uninitialised. 1 at the current 1 kHz loop; observability / diagnostics. */
unsigned gyro_notch_decimation(void);

#endif /* VAYU_DSP_GYRO_NOTCH_H */
