# Vayu FC — Memory Footprint Analysis (RAM/FLASH, stacks, heap, .data/.bss)

**What this is:** a deep accounting of where the firmware's memory goes — FLASH,
static `.data`/`.bss`, the kernel heap (which holds every task stack + TCB), and
the MSP/main stack — with **general (steady-state) vs worst-case** figures and a
per-task table. Numbers are from the real target ELF `build_flash/main`
(built 2026-06-21) via `arm-none-eabi-size`/`nm`, plus source constants.
Target: **STM32F401RE — 96 KB RAM, 512 KB FLASH, Cortex-M4**.

> Method note: `.data`/`.bss`/`.text` are *measured* from the ELF. Heap and stack
> figures are *computed* from `task_create` sizes (bytes) + `sizeof(TCB)` + the
> 16-byte per-block heap header; runtime high-water is measurable on-device via
> the `V_PERF_STACK_FILL` (0xC5C5C5C5) sentinel scan (`v_perf_task_stats`).

---

## 1. Top-level RAM map (96 KB, fully partitioned)

| Region | Size | % of 96 KB | Source |
|---|---:|---:|---|
| `.data` (init globals, RAM copy) | 1,896 B | 1.9% | ELF `-A` |
| `.bss` (zero-init statics) | 31,520 B | 32.1% | ELF `-A` |
| **Static subtotal** | **33,416 B (32.6 KB)** | **34.0%** | |
| **Heap** (`HEAP_SIZE`, fixed) | 57,344 B (56 KB) | 58.3% | `vaios_app_config.h:46` = `0xE000` |
| **MSP / handler stack** | ~7,544 B (7.4 KB) | 7.7% | top-of-RAM − heap_end |
| **Total** | 98,304 B (96 KB) | 100% | |

**Key structural fact:** RAM is *fully committed at link/boot time*. The heap is a
**fixed 56 KB** block placed at `_heap_start` (immediately after `.bss`,
`memory.c:94` `heap_mem_head = (Heap_Mem_Block*)&_heap_start`), and `_estack` is
the top of RAM (`linker.ld:54`). So the MSP (ISR + pre-scheduler stack) is just
the gap between heap-end and top-of-RAM ≈ **7.4 KB**. There is **no free RAM
outside these three** — all slack lives *inside* the 56 KB heap (largely unused,
see §4) and *inside* over-provisioned task stacks (§3).

FLASH: text+rodata+isr+`.data`(LMA) ≈ **~120 KB of 512 KB (~24%)** — not a constraint.

---

## 2. Where the 31.5 KB `.bss` goes (largest static consumers)

From `nm --print-size --size-sort` on the target ELF:

| Symbol | Size | What |
|---|---:|---|
| `_serial_handlers` | **12,336 B** | comm `channel.c` serial channels — each has `buffers[2][2048]` ping-pong (4 KB) + state; ~3 slots. **39% of all `.bss`.** |
| `open_files` | 2,208 B | VFS/FatFS open-file table |
| `_imu_*_buffer` ×4 | 4,048 B | IMU SPSC queues (control/telemetry/attitude/calibration), 1,012 B each |
| `_telemetry_buffer` | 720 B | telemetry staging |
| `fs_obj` | 560 B | FatFS filesystem object |
| `_rx_raw_buf` | 512 B | UART6 RX raw ring |
| `irq_callbacks` | 512 B | IRQ dispatch table |
| `__atexit0` | 400 B | libc atexit |
| `_attitude_*` / `_vert_input_buffer` ×3 | 1,188 B | control/estimator SPSC queues, 396 B each |
| EKF matrices (`Phi`,`PhiP`,`Pnew`,`KH`,`IKH`,`tmp`,`E`) | ~2.5 KB | estimator scratch, ~324–372 B each |

The single biggest lever on static RAM is **`_serial_handlers` (12.3 KB)** — the
2×2048 B telemetry ping-pong buffers (sized up from 512 B to absorb the 1 Hz perf
burst, see `channel.c:9-14`), multiplied across channel slots.

---

## 3. Per-task stacks (the heap's main tenants)

`task_create(stack_size)` takes **bytes**; the stack is `v_malloc`'d from the heap
(`task.c:197`), min 128 B, 4-byte aligned, painted with the high-water sentinel.
Each task also costs one TCB (`sizeof(TCB)` ≈ **112 B** = 72 base + 32 perf + magic)
+ two 16-byte heap headers (stack block + TCB block).

| Task | Prio | Stack (alloc = **worst**) | Measured peak (**general**) | Lifetime |
|---|:--:|---:|---:|---|
| `comm_processor` | 0 | 2,048 | — (deep: cmd dispatch + VFS) | perpetual |
| `bmx160_initiate_read` | 2 | 1,536 | — | perpetual |
| `attitude` (EKF) | 1 | 2,048 | — (deep: matrix math) | perpetual |
| `vertical_estimator` | 1 | 2,048 | — | perpetual |
| `rc_ibus` | 0 | 1,024 | **~120** (12%) | perpetual |
| `angle_controller` | 1 | 2,048 | — | perpetual |
| `angle_rate_controller` | 1 | 2,048 | — | perpetual |
| `motor` | 1 | 1,024 | **~252** (25%) | perpetual |
| `imu_telemetry` | 0 | 2,048 | — | perpetual |
| `bme280_read` | 0 | 1,024 | — | perpetual |
| `flush` | 0 | 1,024 | **~124** (12%) | perpetual |
| `perf_telemetry` | 0 | 2,048 | — | perpetual |
| `heartbeat` | 0 | 1,024 | **~124** (12%) | perpetual |
| `idle` | idle | 512 | — | perpetual |
| `boot` | 0 | 1,024 | — | **exits** (`boot.c:53` `task_exit()`) → reclaimable |
| `calibration` | 0 | **8,192** | — | **on-demand** (`comm_processor.c:103`) |

Observations:
- The 1,024 B stacks are **heavily over-provisioned** — measured peaks are
  120–252 B (12–25%). They could be halved with margin; the sentinel scan exists
  precisely to right-size them.
- The 2,048 B stacks (estimators, controllers, comm, telemetry) carry no recorded
  peak in source — treat **worst-case = full 2,048 B** until measured on-device.
- `calibration` (8,192 B) is the single largest stack and the dominant worst-case
  swing; it is created only when a calibration command arrives (disarmed bench).

---

## 4. Heap budget (56 KB) — general vs worst case

Per-task heap cost = stack + 16 B header + TCB (112 B) + 16 B header.

**General (steady-state): all perpetual tasks running, `boot` already exited, no
calibration — 14 tasks (incl. idle):**

| Component | Bytes |
|---|---:|
| Task stacks (Σ perpetual, incl. idle 512) | 21,504 |
| Stack block headers (14 × 16) | 224 |
| TCBs (14 × 112) | 1,568 |
| TCB block headers (14 × 16) | 224 |
| Semaphores / mutexes / misc init allocs (est.) | ~1,200 |
| **Steady-state heap in use** | **~24.7 KB / 56 KB (≈44%)** |
| **Free heap** | **~31 KB** |

**Worst case: steady-state + `calibration_task` (8,192 + 112 TCB + 32 headers):**

| | Bytes |
|---|---:|
| Steady-state | ~24,700 |
| + `calibration` (stack+TCB+headers) | +8,336 |
| **Worst-case heap in use** | **~33 KB / 56 KB (≈59%)** |
| **Free heap** | **~23 KB** |

`boot` (1,024) and `calibration` don't overlap (boot exits during init;
calibration is commanded post-link), so ~33 KB is the realistic peak. The heap
**watermark warning** trips at `HEAP_SIZE − HEAP_WATERMARK_THRESHOLD` =
57,344 − 1,024 = **56,320 B** (`memory.c:146`) — we never approach it.

Heap allocator: segregated free lists (8 size classes), 16 B header/block,
8 B min payload, split-on-malloc / coalesce-on-free (`memory.c`), so the ~15
long-lived large blocks fragment little.

---

## 5. MSP / main stack (~7.4 KB)

The MSP is the region between heap-end (`_heap_start + 0xE000`) and `_estack`
(top of RAM). With static 32.6 KB + heap 56 KB, that leaves **~7,544 B**. After
the scheduler starts, tasks run on PSP (their heap stacks); the MSP carries only
**ISR nesting + pre-scheduler `main()`/init**. 7.4 KB is ample for the IRQ depth
here (UART/I2C/DMA/timer handlers), but note:

- The linker reserves **no separate stack section and no guard** between heap-top
  and MSP-bottom. They don't collide only because the allocator caps the heap at
  `HEAP_SIZE`. **Raising `HEAP_SIZE` directly eats the MSP margin** — at 56 KB the
  MSP floor is fixed at 7.4 KB.
- `TASK_STACK_OVERFLOW_THRESHOLD = 256 B` guards *task* (PSP) stacks
  (`task.c:301-305`), not the MSP.

---

## 6. Build-variant divergences

- **SITL legacy (pthread)** bumps each task's stack to a 64 KB minimum
  (`host_vaios.c` `if (stack_size < 65536)`) — host libc/pthread overhead, not
  representative of target.
- **SITL RTOS** overrides `HEAP_SIZE = 0x180000` (1.5 MB) so the real scheduler's
  8 KB stacks fit on the host (`sim/host/CMakeLists.txt:225`).
- The figures in §1–§5 are the **real flash target** (`HEAP_SIZE = 0xE000`).

---

## 7. Findings / levers

1. **Static RAM is dominated by `_serial_handlers` (12.3 KB).** If RAM ever gets
   tight, the telemetry ping-pong sizing / channel-slot count is the first lever.
2. **The 1,024 B stacks waste ~0.8 KB each** (measured peaks ≤252 B). Right-sizing
   the five 1,024 B tasks could free ~4 KB of heap — but heap is already 41% free,
   so it's optional.
3. **Worst-case (calibration active) still leaves ~23 KB heap free** — healthy.
4. **MSP is the tightest region (7.4 KB) and is unguarded** against heap growth;
   keep `HEAP_SIZE` ≤ 0xE000 unless the static footprint shrinks, and treat the
   MSP floor as a hard constraint when adding deep/recursive ISR work.
5. **2,048 B stacks have no measured peak** — run `v_perf_task_stats` on-device
   under load (EKF + full telemetry) to confirm headroom before trusting them.

## References
- `build_flash/main` (target ELF) — `arm-none-eabi-size -A`, `nm --size-sort`
- `linker.ld` (RAM/FLASH regions, `_heap_start`, `_estack`)
- `include/vaios_app_config.h` (`HEAP_SIZE`, `IDLE_TASK_STACK_SIZE`, thresholds)
- `extern/vaios/kernel/{task.c,memory.c,perf.c}` (stack alloc, heap, high-water)
- `extern/vaios/include/{task.h,perf.h}` (TCB + perf struct)
- `src/main.c` (task_create sites + recorded peaks), `src/sys/boot.c` (boot exits)
- Siblings: `resource-ownership-map.md`, `sitl-fc-stub-inventory.md`
