# sim_renode — vayu firmware in Renode

Boots the same ARM ELF that flashes to the Nucleo-F401RE (`build/main`) on an emulated STM32F401RE in Renode. Phase 1 of the Renode ↔ Gazebo SITL plan.

## Status

| Stage | Status |
|---|---|
| ELF loads, CPU starts at the reset handler, SRAM/flash sized correctly | ✓ |
| `clock_setup`, `hal_cycle_counter_init`, FPU, NVIC, SysTick | ✓ |
| `v_system_init` → vaios kernel init, UART2 brought up | ✓ |
| Full SDIO command protocol (CMD0/CMD8/ACMD41/CMD2/CMD3/CMD7/CMD16/CMD13) | ✓ |
| FatFS `f_mount` + `check_fs` + `disk_read` of boot sector | ✓ |
| **DMA-driven** SD reads & writes via real NavHAL code path | ✓ |
| `logger_init` → `vfs_preallocate` allocates ~4000 sectors on the card | ✓ |
| Kernel-level UART output (panic banner observed via `-DVAYU_SIM=ON`) | ✓ |

## SITL fidelity

The firmware in sim is bit-identical to what flashes to the board — no polling fallbacks, no `#ifdef VAYU_SIM` carve-outs in the SDIO path. NavHAL's DMA-driven `hal_sdio_read_block_async` + `hal_sdio_wait_sync` runs exactly the same code, and the same `_sdio_dma_rx_irq_handler` ISR runs from the same `DMA2_Stream3_IRQn` vector.

This required custom work because Renode's stock SD model couldn't drive vayu's DMA — see "Custom SDIO + SD mock" below.

## Files

- **`nucleo_f401re.repl`** — F401RE platform on top of `platforms/cpus/stm32f4.repl`: trims SRAM to 96 KB, flash to 512 KB, adds USART6 (iBus RC) and DWT (`hal_cycle_counter_*`). Registers our custom Python SDIO peripheral at `0x40012C00`.
- **`sdmmc_mock.py`** — the Python SDIO + SD mock (next section).
- **`vayu.resc`** — boot script: machine + platform + USART2 analyzer + file backend (`/tmp/vayu_uart2.log`) + `LoadELF build/main`.

## Custom SDIO + SD mock

Renode's stock `SD.STM32FSDMMC` leaves DMA's `PFCTRL`, `PBURST`, `MBURST` as "unhandled bits" and never raises DMA TC. NavHAL's `hal_sdio_wait_sync` polls a `dma_done` flag set by the DMA ISR, so the firmware hangs forever.

`sdmmc_mock.py` replaces it. It:
1. Implements the SDIO command state machine — enough to satisfy vaios's init (`CMD0` → `CMD8` → `CMD55+ACMD41` → `CMD2` → `CMD3` → `CMD7` → `CMD16`, plus `CMD13` status polling).
2. On `CMD17/CMD24` with `DCTRL.DTEN+DMAEN`, copies the block(s) between the backing image file and RAM at the M0AR the firmware programmed into DMA2 — **stream 3 for reads (`0x40026464`), stream 6 for writes (`0x400264AC`)**.
3. Pulses **both** the DMA TC IRQ (`DMA2_Stream3_IRQn=59` for reads, `DMA2_Stream6_IRQn=69` for writes) **and** the SDIO STATUS IRQ (`SDIO_IRQn=49`) via the NVIC ISPR. Both are needed: NavHAL's `_sdio_dma_*_irq_handler` only clears `sd_busy` when both `dma_done` and `sdio_done` are set.

### The deferred-IRQ trick

The most surprising piece of plumbing. Firing the IRQs inline causes a race: the IRQ handler runs *before* NavHAL's calling function reaches `sd_busy = 1` two instructions later. The handler clears `sd_busy = 0`, then `sd_busy = 1` overwrites it, then `wait_sync` enters its `while (sd_busy) WFI` loop with `sd_busy = 1` and stays there forever.

Fix: schedule the NVIC pending write to the next time-source sync point.

```python
m.LocalTimeSource.ExecuteInNearestSyncedState(_deferred_irq_pulse)
```

The CPU's current instruction block completes (including the `sd_busy = 1` write), *then* the deferred action runs, NVIC fires the IRQ, the ISR runs with `sd_busy=1` already set, and clears it. `wait_sync` exits cleanly.

## Build the firmware

```
cmake -B build -DNAVHAL=ON -DEXTERNAL_LINKER=ON
cmake --build build
```

The `-DVAYU_SIM=ON` CMake option exists for future sim-only carve-outs (BMX160 mock etc.) but isn't required for the SD path — the SDIO mock handles that without firmware changes.

## Prepare a backing SD card image

```
truncate -s 32M /tmp/vayu_sd.img
mkfs.vfat -F 16 -n VAYU /tmp/vayu_sd.img
```

The `.repl`'s mock opens this directly. Override with `VAYU_SD_PATH=/path/to/other.img renode …`.

## Run it

```
renode --disable-gui --console \
  -e 'include @tools/sim_renode/vayu.resc' \
  -e 'start' \
  -e 'sleep 10' \
  -e 'pause'
```

Or just `renode tools/sim_renode/vayu.resc` for the GUI (opens a USART2 analyzer window).

UART2 output also lands in `/tmp/vayu_uart2.log`.

After a 10 s run on a clean card you should see ~4000 sectors written — vfs preallocating the navlink/system/general log files — and the card image's hash will have changed.

## Diagnostics

Function-name tracing for "where is vayu?":

```
... \
  -e 'logFile @/tmp/vayu_trace.log' \
  -e 'cpu LogFunctionNames true' \
  ...
# then:
grep -oE 'Entering function [a-zA-Z_][a-zA-Z0-9_]+' /tmp/vayu_trace.log | tail -20
```

SDIO command log (per command vayu issues):

```
grep -E 'sdmmc-mock' /tmp/renode_all.log
```

## Out-of-tree changes (vaios submodule)

The sim depends on one edit inside the vaios submodule that should be upstreamed:

- **`extern/vaios/navhal.config`** — enables `CONFIG_DRV_I2C`, `CONFIG_DRV_I2C_DMA`, `CONFIG_DRV_PWM`, `CONFIG_DRV_CRC` so NavHAL actually builds the drivers vayu uses. (Stock vaios config didn't, after the NavHAL M4 Kconfig-driven build.)

The previous diskio.c `#if 0` patch is **reverted** — the DMA path works now, polling fallback isn't needed.

## Phase 2 — iBus injection from sim_bridge

Done. End-to-end pipeline:

```
FS-i6 transmitter (or synthetic) -> sim_bridge (Arduino, optional)
       -> ibus_inject.py  --(host FIFO /tmp/vayu_usart6.fifo)-->
       usart6_mock.py (Python peripheral)  --(RAM write)-->
       ibus_dma_buf  --(rc_ibus_task polls)-->  ibus_parse_byte
       -> rc_queue_control_push / rc_queue_telemetry_push
```

### Components

- **`tools/sim_renode/ibus_inject.py`** — encodes FlySky iBus frames at 50 Hz (`0x20 0x40 [14 LE-u16 channels] [LE16 checksum]`). Two sources: `--source=test` (synthetic stick sweep) or `--source=/dev/ttyUSB0` (the Arduino `sim_bridge`'s CSV). Auto-detects FIFO vs PTY targets.
- **`tools/sim_renode/usart6_mock.py`** — `Python.PythonPeripheral` at `0x40011400`. Owns USART6 registers (we just shadow them; bits don't matter for our path) AND fakes the RX DMA: a self-rescheduling pump (`LocalTimeSource.ExecuteInNearestSyncedState`) drains the host FIFO and writes bytes directly into `ibus_dma_buf` in RAM, updates `DMA2->STREAM[2].NDTR` so vayu's reader sees forward progress.

### Run it

```
# Terminal 1
renode --disable-gui --console \
  -e 'include @tools/sim_renode/vayu.resc' \
  -e 'start'

# Terminal 2 (or before terminal 1's start, since the inject side blocks
# on FIFO writer until the peripheral opens the read end)
python3 tools/sim_renode/ibus_inject.py --source=test
# or
python3 tools/sim_renode/ibus_inject.py --source=/dev/ttyUSB0
```

### What we learned

**Renode's `STM32_UART` doesn't emit DMA requests** — no `ReceiveDMA` GPIO; that's why the stock model can't drive vayu's DMA-RX code path. Same family of gap as the SDIO/DMA story; same shape of fix (a Python peripheral that does the host I/O and "fakes the DMA" by writing RAM directly).

**Renode's `STM32DMA` doesn't persist `M0AR` for reads.** NavHAL writes `M0AR` during `hal_dma_init`; reading back returns 0. Workaround: hardcode `ibus_dma_buf`'s link-time address (`0x20001974`). Verify with `arm-none-eabi-readelf -s build/main | grep ibus_dma_buf` when the firmware layout changes.

**Latent vayu firmware bug (surfaced by sim).** `src/comm/rc_task.c` reads `DMA2->STREAM[2].NDTR`, but NavHAL's `uart.c` puts USART6 RX on **stream 1**. On real hardware it works by coincidence: stream 2's reset-NDTR = 0 makes `write_ptr = 128`, so `rc_ibus_task` cycles over the buffer stream 1 IS filling. The peripheral mock follows the firmware's actual read address so vayu sees forward progress.

### Tuning that helped (under `VAYU_SIM`)

- `include/variables.h` — log file preallocate dropped from 10 MB to 64 KB (the SDMMC mock runs at ~2 ms/sector; 30 MB would block boot for ~2 min).
- `src/sensor/bmx160.c` — `bmx160_init` returns OK immediately; no I²C polling.
- `src/main.c` — `bmx160_initiate_read` task not created (Phase 5 will replace with a Python I²C BMX160 mock).

## Phase 5 Light — IMU sample injection

Infrastructure in place; full end-to-end verification deferred (Renode + cached models pushed this dev machine into swap; needs an unloaded host to confirm).

- **`tools/sim_renode/imu_inject_mock.py`** — Python peripheral at `sysbus 0x40002400` (unused APB1 slot between TIM14 and RTC). Exposes the latest `bmx160_all_converted_reading_t` (76 bytes) from the host FIFO `/tmp/vayu_imu.fifo`. Default sample is level + stationary so vayu sees sane defaults even before Phase 4's Gazebo bridge wires real IMU in.
- **`src/sensor/bmx160_sim.c`** — under `VAYU_SIM`, replaces the real `bmx160_initiate_read` task. Every millisecond it memcpys from `0x40002400` into a `bmx160_all_reading_t` and pushes via the firmware's normal `imu_buffer_push` / `imu_queue_control_push` / `imu_queue_telemetry_push` APIs. The real task in `src/sensor/bmx160.c` is gated by `#ifndef VAYU_SIM`.
- **`vayu.resc`** force-IsInits the peripheral pre-emulation (`sysbus ReadDoubleWord 0x40002400`) so the host FIFO is created before vayu's first read.

This is "Phase 5 Light" — bypasses the I²C/BMX160 register stack and feeds samples in at the imu_buffer layer. The full I²C + BMX160 + BMM150 register model ("Phase 5 Heavy") is a multi-day follow-up tracked separately; the right path once the rest of the SITL is proven.

## A footgun to remember

## A footgun to remember

vayu's `text+bss = 97 536 / 98 304 bytes — 99.2 % of SRAM`. Flipping `LOGGING_ENABLED` to `1` in `include/vaios_app_config.h` adds enough log-buffer BSS to overflow, and the firmware hangs in the C-runtime `zero_bss` loop before `main()` ever runs. If you need verbose logging for sim debugging, shrink `LOG_BUFFER_SIZE` / `IMU_BUFFER_SIZE` first.
