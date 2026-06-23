# NavLink xfer (FTP) substrate — memory budget

**Status:** budget locked before implementation. Companion to
`docs/plans/navlink-xfer-substrate.md` (feature plan), the deep
`docs/scratch/memory-footprint-analysis.md` (full RAM accounting), and the
`f401-sram-heap-overflow` memory. Numbers below are from the **current**
`feat/centralised-fs-owner` build (`arm-none-eabi-nm/size build/main`).

## The decision in one line

**Allocate the xfer state from the half-empty 56 KB heap (`v_malloc` at init),
NOT from static `.bss`.** The heap has **~23–31 KB free**; `.bss` growth instead
comes straight out of the **~1.7 KB MSP/interrupt-stack margin**, which is the
genuinely scarce region. So xfer adds **~0 B to `.bss`** and **~3 KB to the heap**
— BSS stays flat, the MSP margin is untouched.

## Why `.bss` is the wrong place (the RAM map)

RAM is fully partitioned at link/boot into three regions (see
`memory-footprint-analysis.md` §1, §5):

```
0x20000000  ┌────────────────────────────┐  ORIGIN(RAM)
            │ .data (1896 B) + .bss       │
            │   .bss = 37352 B (current)  │  ← grows with every static/global
0x20009958  ├────────────────────────────┤  _heap_start = _ebss (ALIGN 8)
            │ kernel heap  HEAP_SIZE       │  56 KB fixed. ~23–31 KB of it FREE.
            │   = 0xE000 (56 KiB)         │  task stacks + TCBs + v_malloc live here
0x20017958  ├────────────────────────────┤  _heap_start + HEAP_SIZE
            │ MSP / interrupt stack       │  ← only 1704 B (0x6A8). grows DOWN
0x20018000  └────────────────────────────┘  _estack = top of RAM
```

The three regions are **coupled and sum to 96 KB**:

| Region | Now | How it grows | Headroom |
|---|---:|---|---|
| `.bss` (static) | 37352 B | every new static/global | — |
| heap `HEAP_SIZE` | 57344 B (fixed) | `v_malloc` (stacks, queues) draws from it | **~23–31 KB free** |
| **MSP margin** (heap-top→`_estack`) | **1704 B** | **shrinks 1:1 with `.bss`** | **~1.7 KB — scarce** |

**Two ways `.bss` growth bites:**
1. It pushes `_heap_start` up so the boot `v_memset(_heap_start, 0, HEAP_SIZE)`
   (`memory.c:94`) runs nearer the top of RAM; if `_heap_start + 0xE000 >
   0x20018000` the memset runs off RAM → **boot HardFault** (this already cost us
   one HardFault — fs_owner log queue 32→4, commit `796412f`).
2. The gap `[heap_top, _estack)` **is** the MSP (ISR + pre-scheduler) stack — not
   spare RAM. Spending it on `.bss` directly shrinks the interrupt stack. With the
   FPU active an FP-touching ISR lazy-stacks ~104 B/frame.

**Reconciliation with the footprint analysis (it shows MSP ≈ 7.4 KB):** that doc
was built 2026-06-21 at `.bss` = 31,520 B. Since then `.bss` grew to 37,352 B
(+5,832 B: fs_owner + recent telemetry/world work), so the MSP margin shrank
**7.4 KB → 1.7 KB**. That margin is *already mostly spent*; the lesson is to stop
spending it. **The heap, by contrast, is still ~half empty** (footprint §4:
~24.7 KB used steady-state of 56 KB; ~33 KB worst-case with calibration active).

## xfer allocation plan — heap, not BSS

All sized from `XFER_CHUNK_MAX` = 247, `XFER_MAX_SESSIONS` = 2.

| Allocation | Where | Sizing | Bytes |
|---|---|---|---|
| `xfer_session_t[2]` | **heap** (`v_malloc` in `xfer_init`) | 2 × ~96 B (SM fields + `arg[32]`) | ~192 |
| fs_owner write-at lane backing buffer | **heap** (`v_malloc` in `fs_owner_init`) | `FS_WRITEAT_QUEUE_CAP(2)` × ~296 B (`path[40]`+`offset`+`len`+`data[247]`) | ~592 |
| `xfer_service_task` stack | **heap** (`task_create`) | prio 0, 2048 B | 2048 |
| provider registry + counters | `.bss` (a few pointers/words) | 3 ptr slots | ~64 |
| **Heap added** | | ~192 + 592 + 2048 + TCB/headers(~160) | **~3.0 KB** |
| **`.bss` added** | | registry only | **~64 B** |

Result: heap usage **~24.7 KB → ~27.7 KB** (still **~29 KB free** steady-state,
~20 KB worst-case). MSP margin stays **1704 B**. `_heap_start` barely moves
(+~64 B) → boot memset stays well clear of the top of RAM.

**Deliberately NOT static and NOT per-session:**
- **Download read scratch** (247 B): a local on the `xfer_service_task` stack —
  `fs_owner_read_at(path,off,buf,n)` fills it, `xfer_tx_data` copies it into the
  channel. Costs stack the task already has; 0 BSS, 0 extra heap.
- **Upload staging:** `on_xfer_data` copies straight from the codec-unpacked
  struct (comm-task stack) into the write-at slot — no intermediate buffer.
- **File fds:** uploads/downloads reuse the existing VFS `open_files` table
  (already 2208 B `.bss`); no new fd storage.

`v_malloc` is the same allocator `task_create` already uses for every stack
(`memory.c:130`); these are long-lived blocks never freed, which the segregated
free-list allocator handles with negligible fragmentation (footprint §4).

## Knobs (all `#define`)

| Knob | Default | Effect |
|---|---|---|
| `XFER_MAX_SESSIONS` | 2 | +~96 B **heap** per extra session (plan allows up to 4). |
| `FS_WRITEAT_QUEUE_CAP` | 2 | upload pipelining depth; +~296 B **heap**/slot. CAP=1 if heap ever tightens. |
| `XFER_CHUNK_MAX` / `FS_WRITEAT_PAYLOAD_MAX` | 247 | `XFER_DATA` wire payload (254 B frame ≤ 255). |
| `XFER_ARG_MAX` | 32 | path/name arg; 32 B × sessions. |

## Optional lever — reclaim MSP margin (only if needed)

If we ever want the MSP margin back above ~1.7 KB, the cleanest move is to
**heap-allocate the *existing* fs_owner static buffers too** (`s_log_buf` 1040 B +
`s_save_buf` 656 B = ~1.7 KB `.bss`): converting them to `v_malloc` would roughly
**double the MSP margin** at no heap-pressure cost (the heap has the room). Bigger
levers if static RAM ever gets truly tight, from the footprint analysis §7:
`_serial_handlers` (12.3 KB — the 2×2048 telemetry ping-pong) and the
over-provisioned 1024 B task stacks (measured peaks ≤252 B). None needed for xfer.

## Verification gates (every xfer build)

1. **`.bss` stays flat** — `arm-none-eabi-size build/main`: `.bss` must stay
   ~37.3 KB (only the ~64 B registry). Equivalently
   `nm build/main | grep ' _heap_start'` must stay ≈ 0x20009958 (NOT climb). A
   climbing `_heap_start` means something landed in `.bss` that should be on the
   heap — fix it before flashing.
2. **Heap fits** — boot log must NOT print the `HEAP_WATERMARK` warning
   (`v_get_heap_allocation_size() > HEAP_SIZE − 1024 = 56320`). With ~28 KB used
   we have ~28 KB of margin; this only trips if a sizing knob is wildly off.
3. **MSP under load** — on hardware, run a full-rate download + telemetry; no
   HardFault. (Unchanged from today since `.bss` didn't grow — this is a
   regression check, not a new risk.)
4. **SITL is blind to all three** — host has gigabytes; the wall is target-only.

## Bottom line

xfer costs **~3 KB heap** (of ~23–31 KB free) and **~64 B `.bss`** — because the
state lives in the half-empty heap, not the scarce 1.7 KB MSP margin. The
HardFault class from the `f401-sram-heap-overflow` memory simply can't recur as
long as gate 1 holds (`_heap_start` doesn't climb). Re-run gate 1 after every
build.
