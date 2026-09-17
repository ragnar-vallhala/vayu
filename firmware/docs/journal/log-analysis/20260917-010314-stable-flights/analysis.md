# 2026-09-17 01:03 — the stable-flight session

Source: `data/imuhs-20260917-010314.bin`, the HSL1 ring pulled off the card on
2026-09-17 17:23.

**Scope: sessions `[16]`–`[25]` only** — the night's block, opened by the one
session carrying a synced clock (`[16]`, 2026-09-17 01:03:14); everything after
it is the same block on uptime alone. Earlier sessions in the ring are older
hardware states and are deliberately excluded.

Ten sessions in scope: five flights (`[17]`–`[21]`) and five ground arms
(`[16]`, `[22]`–`[25]`).

All four streams are present and healthy throughout: `imu` 1827 Hz, `ctl` 1000 Hz,
`act` 311 Hz, `vrt` 19 Hz. The 1827 Hz confirms the 1600 Hz ODR config is holding
(the 8.2 % deficit vs 2000 Hz is the known 1-in-13 ancillary DMA chain).

## The flights

| # | aloft | ended | thr med | mixer railed | headroom | gyro rms | u rms (r/p/y) |
|---|---|---|---|---|---|---|---|
| [17] | 5.5 s | FAILSAFE | 0.743 | **51.9 %** | 0.002 | 144 °/s | .191/.193/.185 |
| [18] | 18.0 s | FAILSAFE | 0.979 | **88.4 %** | 0.000 | 33 °/s | .139/.089/.060 |
| [19] | 25.1 s | FAILSAFE | 0.688 | 10.9 % | 0.124 | 114 °/s | .090/.097/.094 |
| [20] | 14.2 s | landed | 0.669 | 12.5 % | 0.153 | 69 °/s | .065/.082/.048 |
| [21] | 11.1 s | landed | 0.582 | 10.1 % | 0.166 | 70 °/s | .118/.128/.071 |

"mixer railed" = fraction of the flight with at least one motor above 0.98.
"headroom" = 1 − median of the highest motor.

`[20]` and `[21]` are the two that ended in a real landing (`IN_AIR -> STANDBY`)
and they are the calmest by every measure. `[19]` is the longest but ended in
failsafe. `[17]` and `[18]` never had any control authority at all.

The ground arms are short and uneventful except `[16]`, which reached 0.83
throttle with motors at the rail for 2.9 s without ever declaring `IN_AIR` —
worth a look as a failed or aborted takeoff. `[22]`/`[23]`/`[25]` peak at
0.30–0.37 throttle and `[24]` never spins up past 0.15.

## What the data says

### 1. Hover throttle is high, and the pack is the likely reason

Hover sits at 0.58–0.69 against a design hover of ~0.38, and even the two good
flights leave only 0.12–0.17 of headroom on the highest motor. The pilot reports
the packs were not fully charged, and the data is consistent with that:

`[17]` and `[18]` are the signature. Throttle ramps to 0.979 and pins there —
quartile medians 0.33/0.71/0.98/0.98 and 0.80/0.98/0.98/0.98 — with the mixer
railed 52 % and 88 % of the time and headroom at literally zero. Commanding full
throttle and still not holding altitude, then failsafe, is what a depleted pack
looks like.

`[19]`–`[21]` are a different story: hover *falls* in chronological order
(0.688 → 0.669 → 0.582), which is not one pack draining across three flights.
The pilot confirms packs were swapped during the block, which is exactly this.
Within `[19]` throttle still creeps (+0.8 %/s, 0.663 → 0.734 across the flight),
which is sag inside one pack.

**Consequence for this archive: no cross-flight comparison here can measure
thrust.** Each flight sat on a different, unrecorded pack state, so hover
throttle, headroom, and the mixer-railed percentage are not comparable between
sessions — they rank pack charge as much as anything about the airframe. The
findings below survive only because none of them depend on that: the trims are
normalised demands, and the clipping and notch numbers are per-flight.

**Caveat, and the reason this can only be "likely":** nothing in the stack
measures battery voltage — no ADC driver in `firmware/src/`, no voltage field in
`navlink/dialect.json`. There is no way to separate a low pack from a genuine
thrust deficit in any recording we can make today. Even `[21]`'s 0.582 is well
above design, and whether that is a tired pack or missing thrust is currently
unanswerable from the logs.

This does not soften §3 — it sharpens it. A sagging pack drives the mixer into
saturation *more* often, which is exactly the condition the old anti-windup
could not see.

### 2. Three constant trims are eating that headroom

Derived two independent ways — the mixer inverse of the motor commands, and the
PID's own logged output `u` — which agree to within 0.02 on every axis and every
flight:

| flight | roll | pitch | yaw |
|---|---|---|---|
| [19] | −0.093 | +0.033 | −0.100 |
| [20] | −0.103 | +0.018 | −0.089 |
| [21] | −0.107 | +0.063 | −0.118 |

The controller holds roll ≈ −0.10 and yaw ≈ −0.10 for the entire duration of
every flight. That is ~10 % of roll authority and ~10 % of yaw authority spent
before the pilot asks for anything, and it is what pins motor 2 at a median 0.86
while motor 3 sits at 0.45.

The yaw trim costs 75–235° of uncommanded heading per flight. Both trims are
mechanical, not a tuning problem: a CoG offset and a motor-mount or prop drag
asymmetry.

### 3. These flights confirm the anti-windup fix, and were flown without it

The mixer was saturated 10–12 % of the time, but the per-axis PID outputs never
came close to their own rails (`|u| > 0.95` for 0.0–0.5 % of samples; `u` rms is
only ~0.09). The old anti-windup keyed on the per-axis rail, so **it never fired
once in any of these flights** while the actuators were clipping for seconds at
a time. The integrators charged straight through every saturation episode.

That is exactly the failure the mixer-saturation anti-windup (commit `887a464`)
addresses, measured on real flights. It is committed but **not yet flashed** —
these logs pre-date it.

### 4. The accelerometer is clipping at ±16 g

`[19]` and `[20]` hit the ±16 g rail (156.9 m/s²) on all three axes — 0.08 % of
samples on x, 0.03–0.04 % on y, 0.19–0.29 % on z. Small, but it means the range
increase did not fully buy out the vibration, and every clipped sample is a lie
told to the vertical estimator. `[21]` does not clip.

### 5. The dynamic notch is working — this is not the problem

Per-axis, comparing the raw `imu` spectrum against the post-filter `ctl` stream:

| flight | raw pitch peak | notch centre | offset |
|---|---|---|---|
| [19] | 125.6 Hz | 123.0 Hz | −2.6 Hz |
| [20] | 126.5 Hz | 124.0 Hz | −2.5 Hz |
| [21] | 117.5 Hz | 119.0 Hz | +1.5 Hz |

It tracks the dominant peak within ~3 Hz and removes 49–58 % of the 80–200 Hz
band on pitch (the axis carrying 4–5× the energy of the other two). For one
Q=8 biquad chasing a peak that wanders 95–164 Hz, that is about right. Notch
tune in `pid.bin`: Q 8, 60–450 Hz, min_ratio 4, enabled.

## Not a finding

The persisted motor geometry in `pid.bin` (`geom_valid = 1`) has spin signs
`{+1,−1,+1,−1}`, the inverse of the compiled default `s_mix_yaw`. This is **not**
a bug: the firmware builds both `B` and `Bpinv` from the persisted table, the
mixer inverse agrees with the PID output on all three axes, and yaw converges
rather than diverging. The persisted table is simply the one in force.

## What to do next, in order

1. **Add battery voltage sensing** (ADC + a telemetry field + an `act`-stream
   column). Without it, "low pack" and "thrust deficit" are indistinguishable in
   every log, and that ambiguity is currently sitting on top of the whole
   analysis.
2. **Re-fly on a known-full pack and re-measure hover throttle.** If it lands
   near 0.38 the thrust budget is fine; if it stays near 0.58 there is a real
   deficit to chase.
3. **Trim out the roll and yaw biases mechanically** — they are worth ~20 % of
   combined authority and cost 75–235° of heading per flight.
4. **Flash `887a464`** (mixer anti-windup). The condition it fixes is present in
   10–12 % of every good flight here.
5. Re-run the accel calibration at ±16 g, then the board-level trim (`imu_id = 4`),
   in that order.
