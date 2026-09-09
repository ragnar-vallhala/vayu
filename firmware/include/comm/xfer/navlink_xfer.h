/**
 * @file include/comm/xfer/navlink_xfer.h
 * @brief Generic NavLink bulk-transfer / streaming substrate — core state machine.
 *
 * A reusable "FTP-like" sublayer over NavLink v2: reliable, offset-addressed
 * chunked transfer (download FC->GCS, upload GCS->FC) plus open-ended streams,
 * with a provider registry so future apps (file transfer, blackbox download,
 * config/tune export, live data streams) plug in without rebuilding transport.
 * Design: docs/plans/navlink-xfer-substrate.md; RAM: firmware/docs/plans/xfer-memory-budget.md.
 *
 * Reliability model (NOT a sliding window): cumulative-ack + resume-by-rewind.
 *   - download: FC streams XFER_DATA by offset, paced against channel backpressure;
 *     an XFER_ACK{next_offset < cursor} rewinds the cursor (idempotent re-read).
 *   - upload: FC writes contiguous chunks (off == cursor only) and periodically
 *     emits XFER_ACK{next_offset = cursor}; the GCS retransmits from there.
 *
 * Codec-blindness: this core includes NO generated navlink_msgs.h. All emission
 * goes through an injected xfer_tx_ops_t (firmware wires it to navlink_xfer_tx.c;
 * the host SM test wires a capturing fake). The constants below mirror the
 * dialect (command_result, xfer_flags, the XFER_* msgids) by value — kept in
 * lockstep with navlink/dialect.json.
 *
 * Threading contract (the C1->C3 invariant — see the centralised FS owner):
 *   - xfer_on_open/on_data/on_ack/on_close run on the COMM task: state only,
 *     never blocking SD I/O. Uploads enqueue via the provider (async fs_owner).
 *   - xfer_tick runs on the dedicated xfer_service_task: provider->open/read
 *     (blocking SD) and all emission happen here, off the comm + control tasks.
 */
#ifndef NAVLINK_XFER_H
#define NAVLINK_XFER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---- tunables (RAM budget: firmware/docs/plans/xfer-memory-budget.md) ------------- */
#ifndef XFER_MAX_SESSIONS
#define XFER_MAX_SESSIONS 2u
#endif
#ifndef XFER_MAX_PROVIDERS
#define XFER_MAX_PROVIDERS 4u
#endif
#define XFER_CHUNK_MAX 247u /* = XFER_DATA wire payload (254 B frame <= 255) */
#define XFER_ARG_MAX 32u    /* = XFER_OPEN.arg char[32] */

/* ---- wire constants mirrored from navlink/dialect.json ------------------- */
#define XFER_WIRE_MSGID_OPEN 8201u
#define XFER_WIRE_MSGID_CLOSE 8202u
#define XFER_SIZE_STREAM 0xFFFFFFFFu /* total_size sentinel: stream/unknown */

/* command_result (dialect enum) — COMMAND_ACK.result + XFER_INFO/ACK.result */
typedef enum {
  XFER_RES_ACCEPTED = 0,
  XFER_RES_TEMPORARILY_REJECTED = 1,
  XFER_RES_DENIED = 2,
  XFER_RES_UNSUPPORTED = 3,
  XFER_RES_FAILED = 4,
  XFER_RES_IN_PROGRESS = 5
} xfer_result_t;

/* xfer_flags (dialect enum) — bitmask on XFER_DATA / XFER_ACK */
enum {
  XFER_F_NONE = 0,
  XFER_F_DONE = 1,
  XFER_F_NAK = 2,
  XFER_F_ABORT = 4,
  XFER_F_EOF = 8
};

typedef enum { XFER_DIR_DOWNLOAD = 0, XFER_DIR_UPLOAD = 1 } xfer_dir_t;
typedef enum { XFER_MODE_FILE = 0, XFER_MODE_STREAM = 1 } xfer_mode_t;

/* xfer_on_open() sentinel: provider->open is deferred to xfer_tick (it may block
 * on SD), so a valid open returns this and the router defers the COMMAND_ACK.
 * Any other return is an immediate xfer_result_t the router acks right away. */
#define XFER_OPEN_DEFERRED 255

/* ---- session ------------------------------------------------------------- */
typedef enum {
  XFER_ST_FREE = 0,
  XFER_ST_PENDING_OPEN, /* args stashed on comm task; tick runs provider->open */
  XFER_ST_ACTIVE,
  XFER_ST_DONE_LINGER /* terminal chunk/ack sent; hold briefly for late acks */
} xfer_state_t;

struct xfer_provider; /* fwd */

typedef struct xfer_session {
  xfer_state_t state;
  uint8_t session; /* slot id (== array index) */
  uint8_t dir;     /* xfer_dir_t */
  uint8_t mode;    /* xfer_mode_t */
  uint8_t gcs_sys; /* reply target, stamped from the XFER_OPEN frame hdr */
  uint8_t gcs_comp;
  uint8_t open_req_seq; /* correlates the deferred COMMAND_ACK */
  uint8_t close_req_seq;
  bool close_pending; /* xfer_on_close seen; tick emits the close COMMAND_ACK */
  bool info_acked; /* first XFER_ACK/peer response seen -> stop retrying INFO */
  bool rx_activity; /* a chunk arrived since the last tick (upload liveness) */
  bool upload_eof;  /* EOF chunk received; tick polls provider->flush (all
                        * bytes persisted) before the terminal DONE/FAILED ack */
  uint16_t service_id;
  uint16_t chunk_size; /* negotiated emit size (<= XFER_CHUNK_MAX) */
  uint32_t total_size; /* XFER_SIZE_STREAM for streams */
  uint32_t cursor; /* download: next offset to emit; upload: next expected */
  uint32_t offset_start;
  uint16_t rate_hz;      /* stream cadence */
  uint32_t next_due_ms;  /* stream: next poll; upload: next periodic ACK */
  uint32_t last_rx_ms;   /* idle-timeout base */
  uint32_t last_emit_ms; /* INFO/ACK retransmit base */
  const struct xfer_provider *provider;
  void *user; /* provider scratch (e.g. open fd / file size) */
  char arg[XFER_ARG_MAX];
} xfer_session_t;

/* Open arguments handed from XFER_OPEN (router maps the codec struct to this). */
typedef struct {
  uint8_t session;
  uint8_t dir;
  uint8_t mode;
  uint8_t req_seq;
  uint8_t gcs_sys;
  uint8_t gcs_comp;
  uint16_t service_id;
  uint32_t offset_start;
  uint16_t rate_hz;
  char arg[XFER_ARG_MAX];
} xfer_open_args_t;

/* ---- provider vtable ----------------------------------------------------- */
/* Return conventions: open 0=ok / <0=error (sub-code negated into result_param2);
 * read >=0 bytes / <0 error; write =len accepted / 0 not-now (backpressure) /
 * <0 error; poll >0 bytes / 0 nothing-due. NULL read/write/poll => that mode is
 * unsupported (open is rejected DENIED).
 *
 * open() is called from xfer_tick (deferred off the comm task) with `a == NULL`:
 * every field it needs is already on the session (s->arg, s->dir, s->mode,
 * s->service_id, s->offset_start, s->rate_hz). For a download it sets *total_out
 * to the byte length (ignored for uploads/streams). */
typedef struct xfer_provider {
  uint16_t service_id;
  const char *name;
  int (*open)(xfer_session_t *s, const xfer_open_args_t *a,
              uint32_t *total_out);
  int (*read)(xfer_session_t *s, uint32_t off, uint8_t *buf, uint16_t max);
  int (*write)(xfer_session_t *s, uint32_t off, const uint8_t *buf,
               uint16_t len);
  int (*poll)(xfer_session_t *s, uint8_t *buf, uint16_t max);
  void (*close)(xfer_session_t *s, int result);
  /* Upload completion (optional). After the EOF chunk, tick polls this until the
   * provider confirms every byte is durably persisted, so the GCS's terminal ack
   * means "on disk", not merely "enqueued". Returns 1 = all flushed (-> DONE),
   * 0 = still pending (keep waiting), <0 = a write permanently failed (-> FAILED,
   * reported to the GCS). NULL => no async write-back, treated as 1 (e.g. the RAM
   * / stream providers). */
  int (*flush)(xfer_session_t *s);
} xfer_provider_t;

/* ---- injected emitter (keeps the core codec-blind) ----------------------- */
typedef struct {
  void (*command_ack)(const xfer_session_t *s, uint32_t acked_msgid,
                      uint8_t req_seq, uint8_t result, int32_t param2);
  void (*info)(const xfer_session_t *s, uint8_t result, uint16_t chunk_size,
               uint32_t total_size, uint32_t mtime);
  /* Returns 1 if the frame was accepted by the channel (will be transmitted),
   * 0 if the TX ring was full and it was dropped. The download SM uses this as
   * real backpressure: the cursor only advances on an accepted write, so it
   * self-paces to the link's true throughput instead of racing ahead and losing
   * chunks. (Must NOT key off the channel-wide overflow *counter* — telemetry
   * shares the channel and moves that counter independently, which would stall
   * the download whenever telemetry congests the ring.) */
  int (*data)(const xfer_session_t *s, uint8_t flags, uint8_t len,
              uint32_t offset, const uint8_t *buf);
  void (*ack)(const xfer_session_t *s, uint8_t flags, uint8_t result,
              uint32_t next_offset);
} xfer_tx_ops_t;

/* ---- API ----------------------------------------------------------------- */
/* Allocate the session table (heap, per the RAM budget) and bind the emitter.
 * Idempotent. `tx` must outlive the substrate (a static in firmware). */
void xfer_init(const xfer_tx_ops_t *tx);

/* Register a provider (by service_id). Returns 0 on success, <0 if the table is
 * full or the id is already registered. Call at boot before any session opens. */
int xfer_register_provider(const xfer_provider_t *p);

/* COMM task. Validate + stash; returns XFER_OPEN_DEFERRED for a valid open (the
 * router defers the COMMAND_ACK; tick runs provider->open) or an immediate
 * xfer_result_t to ack now. Re-open with the same (session, req_seq) is idempotent. */
int xfer_on_open(const xfer_open_args_t *a);

/* COMM task. Ingest one uploaded chunk; advances only on contiguous offset. */
void xfer_on_data(uint8_t session, uint32_t offset, const uint8_t *buf,
                  uint8_t len, uint8_t flags);

/* COMM task. Flow-control / resume ack for a download (rewinds on NAK/regress). */
void xfer_on_ack(uint8_t session, uint32_t next_offset, uint8_t flags);

/* COMM task. Request close/abort; tick emits the COMMAND_ACK and tears down. */
int xfer_on_close(uint8_t session, uint8_t req_seq, uint8_t result);

/* XFER task. Drive deferred opens, paced emission, periodic acks, timeouts.
 * `tx_overflow` is channel_tx_overflow_count() (rising => back off); `chunk_budget`
 * caps XFER_DATA emissions this call (shared round-robin across sessions).
 * Returns the number of chunks emitted (so the task can pace its delay). */
int xfer_tick(uint32_t now_ms, uint32_t tx_overflow, int chunk_budget);

/* True while any file-mode DOWNLOAD is ACTIVE — lets imu_telemetry_task suppress
 * heavy streams to hand the link to a big transfer (mirrors sysid_dump_active()).
 * Streams are best-effort, so this never suppresses a stream-mode xfer. */
bool xfer_download_active(void);

/* True while ANY session is non-FREE (pending-open, active up/down, done-linger).
 * The xfer service task uses this to suppress best-effort blackbox logging for the
 * whole transfer window (fs_owner_suppress_logs), so concurrent multi-file SD
 * access can't corrupt the transfer (the only clean pattern is sequential). */
bool xfer_active(void);

/* Test/inspection helpers. */
bool xfer_session_active(uint8_t session);
const xfer_session_t *xfer_session_get(uint8_t session);
void xfer_reset_all(void); /* host tests: drop all sessions (keeps providers) */

#endif /* NAVLINK_XFER_H */
