# Kernel / RTOS analysis — `export-20260617-124210.bin`

Health of the **vaios** RTOS from the PERF telemetry: `PerfGlobal` (1034, 372),
`PerfTask` (1035, 3,173), `PerfFifo` (1036, 1,062), and `EstPerf` (1033, 284).
Companion to [`session-analysis.md`](session-analysis.md).

## Headline

The kernel is **healthy and not the bottleneck**: ~63 % CPU load with stable
headroom, estimator using 22 % of its loop budget, no heap OOM, no IPC timeouts,
all stacks within margin, and — critically — **zero drops on every control-path
FIFO**. The only saturation is on the **telemetry** FIFOs and the TX buffer,
i.e. the *downlink* can't keep up, which is by-design lossy (overwrite) and does
**not** affect flight control.

## CPU & scheduler

- **CPU load ≈ 63 %** (mean 63.3 %, range 57.8–65.9 % over 312 inter-sample
  deltas). Steady — no creep, no spikes. ~37 % idle headroom.
  > Computed from `cpu_cycles_lo`/`idle_cycles_lo` **deltas** between consecutive
  > reports. These are the *low 32 bits* of 64-bit counters, so a single sample's
  > `1 − idle/cpu` is meaningless (gives −239 % etc.); deltas <~51 s apart are
  > valid. **Reporting caveat:** ship the high words (or a precomputed load) so
  > consumers don't have to reconstruct this.
- **Scheduler:** 6,238,647 context switches over 465 s = **13,414 /s**.
  `systick_max_cyc` = 1000 (longest tick handler), `systick_preemptions` 348,875.
- **Uptime:** first report at 91 s, last at 465 s — the FC had been running **~91 s
  before logging started** (explains `tx_overflow` already = 612 at log start).

## Estimator timing (`EstPerf`)

- Runs at a rock-steady **250 Hz**, IMU **decimation 8** (2000 Hz → 250 Hz).
- mean **709 µs**, peak **891 µs** → **18 % mean / 22 % peak** of the 4000 µs
  budget. Comfortable. This is the EKF (see
  [`sensor-analysis.md`](sensor-analysis.md) §Fusion).

## Memory

- **Heap:** 38 allocs, 2 frees, **0 OOM**, peak **21,232 / 57,344 B (37 %)** —
  essentially all-at-startup static allocation, no churn, no leak. Healthy.
- **Stacks** (worst-case high-water over the whole session):

  | task_id | peak / size | % |
  |---:|---|---:|
  | **4** | **1508 / 2048** | **74 %** ← highest |
  | 13 | 860 / 2048 | 42 % |
  | 11 | 788 / 2048 | 38 % |
  | 6 | 684 / 2048 | 33 % |
  | 5 | 420 / 1536 | 27 % |
  | 10 | 252 / 1024 | 25 % |
  | 9 | 492 / 2048 | 24 % |
  | 1 | 100 / 512 | 20 % |
  | 8 | 396 / 2048 | 19 % |
  | 7 | 156 / 1024 | 15 % |
  | 2 / 12 | 124 / 1024 | 12 % |

  All within margin. **task_id 4 at 74 % is the one to watch** (26 % headroom) —
  fine for now, but worth keeping an eye on if its workload grows.

  > **Task names were not captured** — `PerfTaskname` (1038) requires a
  > `PerfTasknameRequest` (1037) exchange the GCS never sent, so the log carries
  > only numeric ids. To label these, request task names on connect. (Firmware
  > `src/main.c` creates comm/imu/attitude/rc/angle/rate/motor/telemetry/flush/
  > perf tasks; mapping ids needs the live exchange to be certain.)

## IPC

- `ipc_takes` 3,642,812, `ipc_gives` 3,809,036, **`ipc_timeouts` = 0** (clean).
- `ipc_takes_blocked` 2,687,752 (**74 %**) — high but **expected**: worker tasks
  block waiting on their input FIFOs/semaphores; blocking ≠ a problem here given
  zero timeouts.

## FIFOs — the one real signal

SPSC FIFOs, `OVERWRITE` policy (newest wins). Worst-case over the session:

| fifo | role | peak fill | drops |
|---|---|---:|---:|
| IMU_CONTROL (2) | IMU → rate loop | 33 % | **0** |
| ATTITUDE_CONTROL (6) | attitude → angle loop | 22 % | **0** |
| RC_CONTROL (8) | RC → angle loop | 25 % | **0** |
| IMU_TELEMETRY (1) | IMU → downlink | 100 % | **65535** ⚠ |
| ATTITUDE_TELEMETRY (5) | attitude → downlink | 100 % | **65535** ⚠ |
| RC_TELEMETRY (7) | RC → downlink | 100 % | 54921 |
| TELEMETRY (9) | control → downlink | 100 % | **65535** ⚠ |
| IMU_CALIB (3) / CALIB_TELEM (4) | calibration | 0 % | 0 |

**The split is the whole story:** every **control-path** FIFO never dropped a
sample and stayed ≤33 % full — the flight loops are fed cleanly. Every
**telemetry-path** FIFO is pinned at 100 % and dropping continuously. The downlink
simply can't drain them at the offered rate (corroborated by `SystemHealth.tx_overflow`
612→2680 and the 33.5 kbps link in the session report). Because the policy is
`OVERWRITE`, telemetry always shows the *freshest* sample but loses the ones in
between — acceptable for monitoring, and it does **not** starve control.

**Reporting caveats:** the drop counter is `u16` and **saturated at 65535** — true
drop counts are higher and unknowable; widen to `u32` or report a rate. The
calibration FIFOs being idle is consistent with no calibration this session.

## Takeaways

1. Kernel is healthy: 63 % CPU, 22 % estimator budget, 0 OOM, 0 IPC timeouts,
   stacks in margin — **not** limiting flight.
2. **Control is never starved** (0 drops on all control FIFOs); the saturation is
   purely downlink/telemetry — fix at the link/rate level (decimate high-rate
   streams or apply the proposed stream-rate control packet `0x2007`), not in the
   kernel.
3. Telemetry-side reporting fixes: send PerfGlobal cycle high-words (or a computed
   CPU load), widen FIFO `drops` past `u16`, and support a `PerfTaskname` exchange
   so task ids resolve to names.
4. Watch **task_id 4** stack (74 %).
