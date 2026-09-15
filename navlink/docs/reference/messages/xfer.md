# Bulk Transfer & Streaming Substrate (`XFER_*`)

> Authoritative wire spec: `navlink/dialect.json` + `../navlink-v2-spec.md`.
> Design + rationale: `../../../../docs/plans/navlink-xfer-substrate.md`;
> RAM budget: `../../../../docs/plans/xfer-memory-budget.md`.

A generic, reusable "FTP-like" sublayer over NavLink v2: reliable, offset-addressed
**chunked transfer** (download FC→GCS, upload GCS→FC) plus open-ended **streams**.
Future apps — file transfer, blackbox-log download, config/tune export, live data
streams — plug in as **providers** without rebuilding the transport.

## Messages

| Message | msgid | dir | wire | purpose |
| ------- | ----- | --- | ---- | ------- |
| `XFER_OPEN`  | 8201 (cmd) | GCS→FC | 46 B | open a session: dir, mode, `service_id`, `offset_start`, `rate_hz`, `arg` |
| `XFER_INFO`  | 1042 (tlm) | FC→GCS | 12 B | open reply: `result`, negotiated `chunk_size`, `total_size` (0xFFFFFFFF = stream) |
| `XFER_DATA`  | 1043 (tlm) | both   | 254 B| one chunk: `offset`, `len`, `flags` (EOF), `data[247]` |
| `XFER_ACK`   | 1044 (tlm) | both   | 7 B  | cumulative flow-control: `next_offset` (lowest missing byte), `flags` (NAK/DONE/ABORT) |
| `XFER_SACK`  | 1049 (tlm) | GCS→FC | 54 B | selective ack: cumulative `next_offset` plus up to 8 `(miss_off, miss_len)` gaps above it |
| `XFER_CLOSE` | 8202 (cmd) | GCS→FC | 5 B  | close/abort a session |

`XFER_OPEN`/`XFER_CLOSE` are in the command range `0x2000–0x2FFF`, so they are
**§10.5 time-sync-gated** and auto-acknowledged with `COMMAND_ACK`. The data-plane
messages carry no `COMMAND_ACK`; the session state machine owns reliability.

`enum xfer_flags { NONE=0, DONE=1, NAK=2, ABORT=4, EOF=8 }`. `XFER_DATA` is 254 B —
one byte under the 255 payload cap (`data[247]`). The receiver **trusts `len`, never
the unpacked array tail**: the final short chunk trailing-zero-truncates on the wire
(like `SYSID_SAMPLE`).

## Reliability model — cumulative-ack + resume-by-rewind

Deliberately **not** a sliding window. It formalises the FC's proven bulk pattern
(near-stateless FC, self-describing chunks, GCS-driven idempotent re-request,
FC-side pacing against channel backpressure):

- **Download (FC→GCS):** the FC streams `XFER_DATA` from a cursor, paced by a shared
  per-tick chunk budget and `channel_tx_overflow_count()`. An `XFER_ACK{next_offset
  < cursor}` (or `flags=NAK`) **rewinds** the cursor — an idempotent re-read that
  resumes/refills dropped chunks. `EOF` marks the last chunk; the GCS replies
  `XFER_ACK{DONE}`.
- **Upload (GCS→FC):** the FC ingests **contiguous-only** chunks (`offset == cursor`);
  a dup (`<cursor`) or gap (`>cursor`) is ignored. It periodically emits
  `XFER_ACK{next_offset = cursor}`, and the GCS retransmits from there. The final
  chunk carries `EOF`.
- **Stream:** `total_size = 0xFFFFFFFF`; the provider is polled at `rate_hz`; never
  rewinds (best-effort), and rising `tx_overflow` skips stream emission first.

A plain cumulative ack does **not** rewind. Its `next_offset` is behind the
sender's cursor for as long as anything is in flight — that is the normal state
of a sender that is ahead, not a request to resend. Only `flags=NAK` rewinds.
(Treating every ack as a rewind collapsed downloads to stop-and-wait: measured
on hardware at 266 useful chunks against 911 frames emitted.)

## Reliability model — selective repeat (`XFER_SACK`)

The sibling of the above, and **which one a transfer uses is the receiver's
choice**. No negotiation, no capability bit: send `XFER_ACK{NAK}` and you get
go-back-N, send `XFER_SACK` and you get selective repeat, send neither and you
get neither. A receiver that has never heard of `XFER_SACK` is unaffected.

`XFER_SACK` carries the same cumulative `next_offset` plus up to 8 explicit gaps
above it. The FC refills exactly those and **leaves its cursor alone**, so
streaming continues and nothing already in flight is re-sent.

Three properties the FC side must hold, each learned by getting it wrong:

- **A report is authoritative.** A SACK is a complete statement of what the
  receiver is missing, so the repair backlog is rebuilt from each report rather
  than accumulated. Otherwise, when the gaps above the lowest one are filled but
  the cumulative ack cannot advance past that lowest one, the queue stays full of
  entries that are no longer missing and newly reported gaps are never admitted.
- **An emitted repair stays queued.** The receiver keeps naming a hole until it is
  filled, and a repair takes a round trip to land, so it will be re-reported many
  times while already in flight. Re-sending on each of those is the duplication
  selective repeat exists to remove.
- **The retry timer counts reports, not milliseconds** (the handler runs on the
  comm task, which has no clock), so it must track how fast reports arrive: at the
  data-frame rate while streaming (`XFER_SACK_REPEAT_REPORTS`), and at the
  receiver's keepalive cadence once the stream is finished
  (`XFER_SACK_REPEAT_REPORTS_IDLE`), which is hundreds of times slower.

**The receiver must keep reporting when nothing is arriving.** With the stream
finished and a repair lost, the FC is waiting on a report and a receiver that only
reports on arriving data is waiting on the repair — neither moves. A periodic
re-report in the transfer's own protocol is required, not optional.

Measured on hardware, 64 KiB over the ESP8266 bridge, same client otherwise:

| recovery | throughput | duplicate frames |
| -------- | ---------- | ---------------- |
| go-back-N (`NAK`) | 18.1 KiB/s | ~50% |
| selective repeat (`XFER_SACK`) | 21.7–26.0 KiB/s | 22–33% |

## Two-phase open

1. GCS sends `XFER_OPEN`. The FC validates on the comm task (session slot,
   provider, dir/mode capability) — an immediate reject returns the result in
   `COMMAND_ACK` (`TEMPORARILY_REJECTED` slot busy/out-of-range, `UNSUPPORTED`
   unknown `service_id`, `DENIED` bad dir/mode).
2. A valid open is **deferred**: the `xfer_service_task` runs the (possibly
   blocking) `provider->open` off the comm task, then emits `COMMAND_ACK`
   (`ACCEPTED`/`FAILED`) **+** `XFER_INFO`. This upholds the "no blocking SD on the
   comm task" invariant (the centralised FS owner / C1→C3 fix).

## Providers

| `service_id` | name | dir | `arg` | backing |
| ------------ | ---- | --- | ----- | ------- |
| 0 | `file`   | up + down      | SD path        | `fs_owner_read_at` / `fs_owner_enqueue_write_at` |
| 1 | `log`    | download only  | `navlink`/`system`/`general` | blackbox circular file (raw dump; GCS de-wraps). Upload → `DENIED` |
| 2 | `stream` | download stream| source name (e.g. `imu`) | a named live telemetry source @ `rate_hz` |

All storage I/O is mediated by the **fs_owner** gateway: uploads ride a dedicated
async write-at lane (drained after PID/calib saves, before logs); downloads use the
one sanctioned synchronous reader (`fs_owner_read_at`, `vfs_mutex`-serialised,
xfer-task-only).

## Notes

- `chunk_size` is advertised in `XFER_INFO` so neither side hard-codes it.
- Concurrent sessions (default 2) share one channel budget, round-robined per tick.
- A big file download sets `xfer_download_active()`, which lets the telemetry task
  suppress the heaviest tuning streams to hand the link to the transfer (mirrors the
  system-ID dump).
- `XFER_OPEN` folds the stream rate into `rate_hz`, **superseding** the never-shipped
  `CMD_SET_STREAM_RATE` proposal (see `stream_rates.md`).
