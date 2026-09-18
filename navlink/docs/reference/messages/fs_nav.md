# Filesystem Navigation (`FS_LIST` / `FS_INFO` / `FS_DELETE`)

> Authoritative wire spec: `navlink/dialect.json` + `../navlink-v2-spec.md`.
> FC implementation: `src/comm/xfer/fs_query.c` on the fs_owner gateway.

Browse the FC's SD card from the GCS: list a directory, stat any single path,
and delete a file.
The companion to the bulk-transfer substrate ([`xfer.md`](xfer.md)) — that moves
bytes, this navigates. A path that **does not exist** is reported distinctly
(`result = DENIED`), never as an empty success.

## Messages

| Message | msgid | dir | wire | purpose |
| ------- | ----- | --- | ---- | ------- |
| `FS_LIST`        | 8203 (cmd) | GCS→FC | 53 B | list a directory; `path` char[48], `start_index` to resume |
| `FS_ENTRY`       | 1045 (tlm) | FC→GCS | 27 B | one directory entry: `index`, `type` (0=file/1=dir), `size`, `name` char[16] |
| `FS_INFO`        | 8204 (cmd) | GCS→FC | 51 B | stat one path; `path` char[48] |
| `FS_INFO_REPLY`  | 1046 (tlm) | FC→GCS | 12 B | `result`, `type`, `size`, `mtime` (ext) |
| `FS_DELETE`      | 8207 (cmd) | GCS→FC | 51 B | delete one file; `path` char[48]. Answered by `COMMAND_ACK` alone |

`FS_LIST`/`FS_INFO`/`FS_DELETE` are in the command range `0x2000–0x2FFF`, so they are
§10.5 time-sync-gated and `COMMAND_ACK`-acknowledged. The replies are telemetry
correlated by `req_seq`.

## Listing protocol

1. GCS sends `FS_LIST{path, start_index}`.
2. The FC replies `COMMAND_ACK` (deferred — the directory walk runs off the comm
   task on the xfer service task), then streams one `FS_ENTRY` per directory
   entry (`index` = 0,1,2,…, `result = ACCEPTED`), paced by the shared chunk
   budget.
3. After the last entry the FC emits a **terminal `FS_ENTRY`** with an **empty
   `name`** and `count` = the total number of entries — the GCS knows the listing
   is complete.
4. If `path` does not exist (or is not a directory), the FC emits a **single
   `FS_ENTRY` with `result = DENIED`** and stops.

`start_index` lets the GCS resume/refill a long listing from a given entry
(the FC re-opens the directory and skips to that index).

## Stat protocol

GCS sends `FS_INFO{path}`; the FC replies `COMMAND_ACK` then one
`FS_INFO_REPLY`: `result = ACCEPTED` with `type`/`size`/`mtime` if the path
exists, or `result = DENIED` if it does not.

## Delete protocol

GCS sends `FS_DELETE{path}`; the FC answers with `COMMAND_ACK` and nothing
else — there is no outcome to report but the result code:

| result | meaning |
| ------ | ------- |
| `ACCEPTED` | the file is gone |
| `DENIED` | the path is absent, is a directory, or is **protected** |
| `TEMPORARILY_REJECTED` | the file is in use right now; retry later |
| `FAILED` | allowed, but the unlink itself errored |

Two classes of path are refused, and the result code is what separates them —
`DENIED` means stop asking, `TEMPORARILY_REJECTED` means ask again later:

- **`cal.bin` and `pid.bin`, always.** The calibration and tune stores. Losing
  either costs a full recalibration or retune, and neither is ever the file
  someone meant to reclaim space with. Matched on the basename,
  case-insensitively, because FatFS is case-insensitive and a guard that only
  caught one spelling of `PID.BIN` would be no guard at all.
- **`v_nav.bin`, `v_sys.bin`, `v_gen.bin`, always.** The blackbox ring files.
  Deleting one is not the small thing its name suggests: `fs_owner_boot_init`
  preallocates all three at the next boot and `vfs_preallocate` zero-fills
  512 B at a time, so 30 MB of re-creation runs before the scheduler reaches
  `timer_callback_init` — the aircraft looks hung for minutes. They are ring
  files besides, so deleting one reclaims nothing: the space is already fixed.
- **`imuhs.bin` while a session is recording.** The high-speed recorder holds it
  open and is writing into it. This one is state-gated rather than permanent,
  so it succeeds once disarmed. The gate is re-checked at the unlink itself, not
  only when the request arrives, because a session can start in between.

Policy lives in `fs_query.c`, not in the fs_owner gateway: the owner moves
bytes, the service decides what may be moved. `fs_owner_unlink` enforces no
policy of its own, and drops any cached write/read handle on the path first so
no later flush can write into clusters the unlink has freed.

## Notes

- Names are FatFS 8.3 (`FF_USE_LFN = 0`), so `FS_ENTRY.name` is char[16].
- All VFS access is mediated by the fs_owner gateway
  (`fs_owner_stat`/`fs_owner_opendir`/`fs_owner_readdir`/`fs_owner_closedir`):
  synchronous, `vfs_mutex`-serialised, xfer-service-task only — the SD stays
  single-transaction, and the comm/control tasks never block on it.
- The underlying VFS directory/stat primitives live in vaios
  (`vfs_stat`/`vfs_opendir`/…) over NavHAL `v_stat`/`v_opendir`/`v_readdir`.
