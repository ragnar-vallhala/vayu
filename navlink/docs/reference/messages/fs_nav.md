# Filesystem Navigation (`FS_LIST` / `FS_INFO`)

> Authoritative wire spec: `navlink/dialect.json` + `../navlink-v2-spec.md`.
> FC implementation: `src/comm/xfer/fs_query.c` on the fs_owner gateway.

Browse the FC's SD card from the GCS: list a directory, and stat any single path.
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

`FS_LIST`/`FS_INFO` are in the command range `0x2000–0x2FFF`, so they are
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

## Notes

- Names are FatFS 8.3 (`FF_USE_LFN = 0`), so `FS_ENTRY.name` is char[16].
- All VFS access is mediated by the fs_owner gateway
  (`fs_owner_stat`/`fs_owner_opendir`/`fs_owner_readdir`/`fs_owner_closedir`):
  synchronous, `vfs_mutex`-serialised, xfer-service-task only — the SD stays
  single-transaction, and the comm/control tasks never block on it.
- The underlying VFS directory/stat primitives live in vaios
  (`vfs_stat`/`vfs_opendir`/…) over NavHAL `v_stat`/`v_opendir`/`v_readdir`.
