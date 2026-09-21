# Battery voltage sensing

Nothing in the stack measures pack voltage: no ADC driver in `firmware/src/`, no
voltage field in `navlink/dialect.json`, and the only `battery` mentions in the
tree are comments about sag in `height_controller.h`, `rate_indi.h` and
`flight_phase.h`. The consequence is that **"low pack" and "real thrust deficit"
are indistinguishable in every recording we can make**, which is exactly the
ambiguity that blocked the 2026-09-17 flight analysis (see
[`journal/log-analysis/20260917-010314-stable-flights/analysis.md`](https://github.com/ragnar-vallhala/vayu-logs/blob/main/20260917-010314-stable-flights/analysis.md)
§1): hover throttle 0.58–0.69 against a ~0.38 design hover, packs swapped
mid-session, and no way to attribute any of it.

This plan closes that. Scope is **measurement and logging only** — no
voltage-based compensation, no low-battery failsafe. Those are separate
decisions and should not ride along with the sensing.

## Status

**Blocked on hardware.** The divider is a board modification that does not exist
yet. Firmware work is specified below and can land ahead of it (the read simply
returns a floor value with no divider fitted), but the numbers are meaningless
until the resistors are on.

---

## Hardware

PA0 is free: absent from the pin-ownership table
([`scratch/resource-ownership-map.md`](../scratch/resource-ownership-map.md) §3),
unreferenced anywhere in `firmware/src/`, and already the F401's designated
analog input in NavHAL's board header (`src/board/nucleo_f401re/board.h`:
`BOARD_ADC_PIN A0`, `BOARD_ADC_CHANNEL 0`, `BOARD_ADC HAL_ADC_1`). Using it
means no channel arithmetic to get wrong.

Of PA0–PA3, all four are ADC1 regular channels (IN0–IN3), but **PA2/PA3 are
taken** by USART2 iBus RC (`rc_task.c:112`). PA1 (IN1) is the spare if a second
sense line is ever wanted — pack current, or a second cell tap.

### Divider — 33 kΩ / 10 kΩ, for 3S

| | |
|---|---|
| Ratio | 4.3 (43k/10k) |
| 12.6 V full pack reads | 2.93 V — 89 % of the 3.3 V scale |
| Clipping at | ~14.1 V, so a hot-off-the-charger pack still fits |
| Thévenin impedance | 7.7 kΩ — inside the STM32 ADC source-impedance budget, **no buffer needed** |
| Resolution at the pack | 3.46 mV/LSB (0.806 mV/LSB × 4.3); sag needs ~10 mV, so ample |
| Quiescent draw | 293 µA — negligible in flight, matters only if a pack sits plugged in for weeks |

**100 nF across the low-side resistor.** With 7.7 kΩ source that is a ~200 Hz
corner: kills ESC switching pickup, still far faster than the few-Hz sampling
this needs.

**Protect the pin.** If the 10 kΩ goes open, PA0 sees the full pack and the MCU
dies. A BAT54 to 3V3, or a 3.3 V zener, across the low side.

---

## Firmware

NavHAL **already has the driver** — `include/common/hal_adc.h`,
`src/vendor/stm32/adc/`, F401 registers in
`family/stm32f4/include/family/adc_reg.h`. Blocking, polled, single-channel:
`hal_adc_read()` returns a raw right-aligned 12-bit code, and the caller must
put the pin in `HAL_GPIO_MODE_ANALOG` first. That is a fine fit for a few-Hz
voltage read. It is simply not enabled here.

1. **`firmware/navhal.config`** — add `CONFIG_DRV_ADC=y`. One line; the driver
   set is otherwise unchanged.

2. **New `firmware/src/sensor/battery.c`** — a small module, polled from an
   existing low-rate task. Do **not** give it its own task: `rate_ctl` stack
   headroom is finite and every added preemption is a boot-panic risk (see
   `memory/rate-ctl-stack-marginal.md`). Sampling at the `vrt` stream's 19 Hz,
   or slower, is plenty.

3. **Correct against VREFINT, not the nominal 3.3 V.** The F401 exposes its
   internal 1.21 V reference on **ADC1 channel 17**, no external parts. Read it
   alongside PA0 and scale by the ratio.

   This is not optional polish. VDDA on this board sags under load, so a
   reading referenced to a nominal 3.3 V would move *with throttle* and
   masquerade as pack sag — the one failure mode that would actively mislead
   the analysis this whole plan exists to serve.

4. **Keep the ratio a calibration knob, not a constant.** 1 % resistors give
   ~1.4 % worst-case ratio error. Relative accuracy across flights is what the
   analysis needs, but expose a trim factor calibrated once against a
   multimeter rather than hardcoding 4.3. Persist it next to the other tunes if
   it earns its keep; a compile-time constant in `variables.h` is an acceptable
   start.

---

## Telemetry and logging

5. **`navlink/dialect.json`** — a voltage field, then regenerate
   (`generate.py --lang all`). Range and `req_seq` rules per the v2 spec.

6. **A column on the HSL `act` stream.** This is the point of the exercise: the
   voltage has to land *next to throttle and the four motor commands*, in the
   same record, so a log can answer "was the mixer railed because the pack was
   down" without correlating across streams. `act` is 311 Hz and already carries
   `[m1,m2,m3,m4,thr,flags]`; voltage at that rate is wasteful but the record is
   the natural home, so prefer widening it over a new stream. Check the record
   size against `HSL_PREAMBLE_BYTES` and the FMT declaration bound.

   Bump the format so old logs still decode, and teach `tools/telemetry/hslog.py`
   the new column in the same change — it is the reference reader and the
   `--selftest` is what keeps it honest against the spec.

---

## Explicitly out of scope

- **Voltage-compensated thrust or gain scheduling.** A separate decision.
- **Low-battery failsafe or RTL.** Also separate, and higher-stakes than the
  sensing.
- **Cell-level monitoring / balance leads.** A single pack-level tap answers the
  question in front of us.
- **Current sensing and mAh integration.** PA1 is reserved for it if wanted, but
  it is not needed to disambiguate thrust from charge.

---

## Verification

- Bench: known bench-supply voltages across 9–13 V, compare against a
  multimeter, confirm the VREFINT correction holds while a supply is loaded.
- Confirm the reading does **not** move with throttle on a bench run with props
  off and the pack on a charger — if it does, the VREFINT correction is wrong.
- Then re-fly on a known-full pack and re-measure hover throttle. If it lands
  near 0.38 the thrust budget is fine; if it stays near 0.58 there is a real
  deficit to chase, and for the first time the logs will say which.
