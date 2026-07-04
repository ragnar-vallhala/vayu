# FFT-driven dynamic gyro notch

An adaptive band-stop filter on the gyro rate measurement that tracks and removes
the thrust-correlated prop vibration (~0.2–1.5 g) in real time. The craft runs
analog ESCs with **no RPM telemetry**, so an FFT of the live gyro spectrum is the
only honest source of *where* the vibration actually is — hence "dynamic notch"
(the center frequency follows the spectrum) rather than a fixed static notch.

Design reference: [`reference-autopilots-comparison.md`](../journal/log-analysis/20260625-233852-pitch-indi-campaign/reference-autopilots-comparison.md)
§10.5–§10.6 and `recommendations.md` P3 #7. Built bottom-up: FFT engine → biquad
evaluator → analysis front-end → notch bank → firmware glue → tuning surface.

**Status: the full chain is BUILT, host-tested, F401-cross-compiled, and wired
into the rate loop — plus SD persistence, a GCS tuning page, spectrum-derived
auto-band, and the >1 kHz decimation seam. Still OFF by default and never
flight-tested** — the flight test is the one remaining gap. It sits behind the
§9.1/§9.2 control-authority fixes in priority (P3).

---

## Architecture

```
gyro (1 kHz, post-LPF)
   │
   ├─► notch_bank_observe ─► notch_fft (ring→Hann→rfft→peak-pick) ─► center freqs
   │                                    │ (staggered: one axis / tick)
   │                          notch_bank_update ─► redesign biquads (bump-free)
   │
   └─► notch_bank_filter ─► 3× RBJ band-stop biquad cascade ─► filtered gyro ─► rate PID
```

- The **filter** runs every sample on the rate-loop hot path (cheap: ~5 mul + 4
  add per biquad section).
- The **analysis** (FFT + peak-pick + coeff redesign) is the expensive part; it is
  amortised to **at most one axis per tick** so the worst-case tick cost is a
  single N=128 transform.
- The biquads always run at the full loop rate; the analyzer runs at a decimated
  rate `D = floor(loop / ~1 kHz)` so its Nyquist stays above the 450 Hz band no
  matter how fast the loop ticks. At the current 1 kHz loop `D = 1` and the two
  rates coincide; raising `INNER_LOOP_FREQ_HZ` transparently engages decimation
  (observe every Dth sample). `gyro_notch_decimation()` reports the active D.

---

## What's done

All on `main` (commits `13edf95` → `2b81279`). Every module is portable float,
no CMSIS-DSP, no HAL; all working buffers are caller-owned or `v_malloc`'d so
`.bss` stays flat — the F401 boot-HardFault invariant (`_heap_start` + `HEAP_SIZE`
must stay under the top of SRAM; adding static BSS here would break boot).

| Layer | File(s) | Notes |
| --- | --- | --- |
| **FFT engine** | `src/maths/fft.c`, `include/maths/maths_interface.h` | Radix-2 DIT + real-FFT wrapper `m_rfft_forward`, `m_fft_bin_power`; one const-in-flash twiddle table (`fft_tables_N128.h`) via a stride trick. 0 bss. |
| **Biquad evaluator** | `src/maths/biquad.c` | RBJ band-stop designer `m_biquad_notch_design` + `_bypass`, DF2T `m_biquad_step` (bump-free coeff swap). libm only in the designer, off the per-sample path. Fail-safe bypass on bad params. |
| **Analysis front-end** | `src/dsp/notch_fft.c`, `include/dsp/notch_fft.h` | Overlapped Hann ring (hop N/2) → rfft → **whole-spectrum-mean threshold** → greedy peak-pick with parabolic sub-bin interpolation + guard-band skirt suppression. Caller-owned buffers, 0 bss. |
| **Notch bank** | `src/dsp/notch_bank.c`, `include/dsp/notch_bank.h` | Composes fft + biquad into a per-channel N-section cascade. `observe`/`update`/`filter`; records tuned `freqs[]`; `set_hold` (hold-last-good on a peak dropout), `reset`, `set_detection` (live Q/band/prominence). |
| **Firmware glue** | `src/dsp/gyro_notch.c`, `include/dsp/gyro_notch.h` | `v_malloc`s the per-axis banks + FFT scratch (~6.2 KB heap). Wired into `angle_rate_controller.c` after the gyro-LPF, before the rate PID. Staggers retune to one axis/tick. |
| **Gating** | `gyro_notch.c` + `CMakeLists.txt` | Compile gate `VAYU_FFT_NOTCH` (default **on**; off → passthrough stubs and `--gc-sections` reclaims **~56 KB flash**). Runtime **throttle gate** (engages >10 % throttle where prop vibration exists; resets the banks on the disengage edge so no stale notch at spool-up). |
| **Tuning surface** | `navlink/dialect.json`, `navlink_router.c`, `navlink_tx.c`, `telemetry_task.c` | `CMD_SET_GYRO_NOTCH` (msgid 8206): master enable + Q / fmin / fmax / min_ratio + `autoband`, applied live to all axes. `NOTCH_STATUS` (msgid 1047): 9 per-axis center freqs streamed at 5 Hz. |
| **SD persistence** | `pid_config.c` (`pid.bin` **PID4**), `fs_owner.c` | The tune (enable + band/Q, the *effective* set read back from the notch) persists across reboot; restored in the rate-controller init. Magic bumped PID3→PID4 (resets old files); `FS_SAVE_PAYLOAD_MAX` 160→192. |
| **Auto-band** | `gyro_notch.c` | One-shot spectrum-derived band: while engaged, watch the tracked peaks then tighten `[fmin,fmax]` around the observed min/max (±20 Hz, floored at 40 Hz, clamped below Nyquist). Armed by the `autoband` command flag; live-only. |
| **Decimation seam** | `gyro_notch.c` | Analyzer decimates to a ~1 kHz effective rate above a 1 kHz loop (`D = floor(loop/1 kHz)`); biquads stay at the full rate. `D = 1` today (no change), real for `D > 1`. |
| **GCS page** | `navigator/…/GyroNotchWidget`, `CommandCodec`, `NavlinkRouter`, `DroneProtocol` | Tools-menu page (Ctrl+9): enable + auto-band + Q/band/min_ratio tuning form emitting `CMD_SET_GYRO_NOTCH`, over a live 3×3 `NOTCH_STATUS` center-frequency readout. |

### Defaults (compile-time, in `gyro_notch.c`)

| Param | Value | Meaning |
| --- | --- | --- |
| N | 128 | FFT length (reuses the const flash tables → zero-heap window/twiddles) |
| notches/axis | 3 | fundamental + 2 harmonics |
| Q | 8.0 | band-stop width |
| band | 60–450 Hz | analysis band (< 500 Hz Nyquist at 1 kHz) |
| min_ratio | 4.0 | peak must exceed 4× the whole-spectrum mean power |
| throttle gate | 0.10 | notch engages above 10 % throttle |

### Verification

- Host unit tests (ctest): `firmware/tests/host/{biquad,notch_fft,notch_bank}_unit_test.c`
  — sub-bin tone recovery, multi-peak ranking, band gating, hold/reset/params,
  fail-safe bypass. **All pass.**
- SITL integration (`sim/host/tests/test_phase3_comm.c`, COMM-CMD-004): the notch
  command applies + persists (save→reload round-trip), the filter tracks a 200 Hz
  tone end-to-end, and the auto-band pass narrows the band to bracket it.
- ARM cross-compile clean on cortex-m4F; **`.bss` stays flat**, boot invariant
  holds (`_heap_start` + 0xE000 ≤ 0x20018000, ~360 B margin with PID4).
- `dsp` coverage floor in `tools/coverage_gate.py` retuned **95.0→92.0** now that
  `gyro_notch.c` glue is measured (SITL-driven; its `v_malloc`-failure aborts are
  not host-coverable). Gate green (dsp ~93 %).
- NavLink codec regenerates from `dialect.json` at build; generated C compiles
  `-Werror` clean and the wire round-trips (firmware + navigator parity tests).

---

## What's left

1. **Flight test** — the whole point, and the only substantive gap. Enable with
   `CMD_SET_GYRO_NOTCH` (or the GCS Gyro-Notch page), arm, and fly at throttle.
   Confirm the notches track a real prop line and that gyro noise / motor heat
   drops without adding rate-loop phase lag. Nothing downstream is validated until
   this happens. Auto-band should be exercised in the same session (arm it, fly to
   a hover, confirm the learned band is sane).
2. **Persist the learned auto-band** — an auto-band pass sets the band live but
   does **not** auto-persist it (no synchronous save mid-flight); the operator must
   re-issue an explicit `CMD_SET_GYRO_NOTCH` with the discovered values to save.
   A "save current" path (or reporting the live band in `NOTCH_STATUS`) would close
   this.
3. **Decimation for factor > 1** — the seam is real and unit-reasoned but only
   exercised at `D = 1` (the 1 kHz loop); a loop rate > ~2 kHz would engage `D > 1`
   for the first time and wants a dedicated test.

### Known, unrelated

The `navlink test_registry_covers_dialect` check fails on the `prod` profile
(the registry omits the `HW_TEST_*` msgids that full-dialect lists). This is
**pre-existing** and not caused by this work — confirmed by reverting the schema.

---

## Non-goals

No CMSIS-DSP, no DShot/bidirectional-DShot RPM telemetry (the ESCs are analog —
this filter exists *because* there's no RPM feedback). All new state on the heap,
never static.
