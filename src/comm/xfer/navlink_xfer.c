/**
 * @file src/comm/xfer/navlink_xfer.c
 * @brief Generic NavLink bulk-transfer / streaming substrate — core SM.
 *
 * Codec-blind dual-direction state machine + provider registry. See the header
 * and docs/plans/navlink-xfer-substrate.md. Emission is via the injected
 * xfer_tx_ops_t so this file links + tests with no generated codec.
 */
#include "comm/xfer/navlink_xfer.h"

#include "memory.h" /* v_malloc */

/* ---- module state -------------------------------------------------------- */
static const xfer_tx_ops_t *s_tx;
static xfer_session_t *s_sessions; /* heap: XFER_MAX_SESSIONS (RAM budget) */
static const xfer_provider_t *s_providers[XFER_MAX_PROVIDERS];
static uint8_t s_provider_count;
static bool s_ready;

/* Timeouts / cadences (ms) — docs/plans/navlink-xfer-substrate.md. */
#define XFER_INFO_RETRY_MS 500u
#define XFER_ACK_PERIOD_MS 200u
#define XFER_IDLE_MS 5000u
#define XFER_DONE_LINGER_MS 1000u

/* ---- helpers ------------------------------------------------------------- */
static const xfer_provider_t *find_provider(uint16_t service_id) {
  for (uint8_t i = 0; i < s_provider_count; i++)
    if (s_providers[i]->service_id == service_id)
      return s_providers[i];
  return NULL;
}

static void session_free(xfer_session_t *s) {
  uint8_t id = s->session;
  for (uint32_t i = 0; i < sizeof(*s); i++)
    ((uint8_t *)s)[i] = 0;
  s->session = id;
  s->state = XFER_ST_FREE;
}

static bool is_stream(const xfer_session_t *s) {
  return s->total_size == XFER_SIZE_STREAM;
}

/* ---- lifecycle ----------------------------------------------------------- */
void xfer_init(const xfer_tx_ops_t *tx) {
  s_tx = tx;
  if (!s_sessions) {
    s_sessions = (xfer_session_t *)v_malloc(sizeof(xfer_session_t) *
                                            XFER_MAX_SESSIONS);
  }
  if (s_sessions) {
    for (uint8_t i = 0; i < XFER_MAX_SESSIONS; i++) {
      session_free(&s_sessions[i]);
      s_sessions[i].session = i;
    }
  }
  s_ready = (s_sessions != NULL);
}

int xfer_register_provider(const xfer_provider_t *p) {
  if (p == NULL || s_provider_count >= XFER_MAX_PROVIDERS)
    return -1;
  if (find_provider(p->service_id) != NULL)
    return -2;
  s_providers[s_provider_count++] = p;
  return 0;
}

void xfer_reset_all(void) {
  if (!s_sessions)
    return;
  for (uint8_t i = 0; i < XFER_MAX_SESSIONS; i++) {
    session_free(&s_sessions[i]);
    s_sessions[i].session = i;
  }
}

bool xfer_download_active(void) {
  if (!s_ready)
    return false;
  for (uint8_t i = 0; i < XFER_MAX_SESSIONS; i++) {
    const xfer_session_t *s = &s_sessions[i];
    if (s->state == XFER_ST_ACTIVE && s->dir == XFER_DIR_DOWNLOAD &&
        s->mode == XFER_MODE_FILE)
      return true;
  }
  return false;
}

bool xfer_session_active(uint8_t session) {
  if (!s_ready || session >= XFER_MAX_SESSIONS)
    return false;
  return s_sessions[session].state != XFER_ST_FREE;
}

const xfer_session_t *xfer_session_get(uint8_t session) {
  if (!s_ready || session >= XFER_MAX_SESSIONS)
    return NULL;
  return &s_sessions[session];
}

/* ---- COMM-task handlers (state only; no blocking I/O) -------------------- */
int xfer_on_open(const xfer_open_args_t *a) {
  if (!s_ready || a == NULL)
    return XFER_RES_TEMPORARILY_REJECTED;
  if (a->session >= XFER_MAX_SESSIONS)
    return XFER_RES_TEMPORARILY_REJECTED;

  xfer_session_t *s = &s_sessions[a->session];

  /* Idempotent retransmit: same slot + same req_seq already in flight. */
  if (s->state != XFER_ST_FREE) {
    if (s->open_req_seq == a->req_seq && s->state == XFER_ST_PENDING_OPEN)
      return XFER_OPEN_DEFERRED;
    return XFER_RES_TEMPORARILY_REJECTED; /* slot busy with another transfer */
  }

  const xfer_provider_t *p = find_provider(a->service_id);
  if (p == NULL)
    return XFER_RES_UNSUPPORTED;

  /* Capability gate (decidable without touching SD). */
  if (a->dir == XFER_DIR_UPLOAD && p->write == NULL)
    return XFER_RES_DENIED;
  if (a->dir == XFER_DIR_DOWNLOAD && a->mode == XFER_MODE_FILE && p->read == NULL)
    return XFER_RES_DENIED;
  if (a->mode == XFER_MODE_STREAM && p->poll == NULL)
    return XFER_RES_DENIED;

  /* Stash; the (possibly blocking) provider->open runs in xfer_tick. */
  session_free(s);
  s->session = a->session;
  s->state = XFER_ST_PENDING_OPEN;
  s->dir = a->dir;
  s->mode = a->mode;
  s->service_id = a->service_id;
  s->offset_start = a->offset_start;
  s->cursor = a->offset_start;
  s->rate_hz = a->rate_hz;
  s->gcs_sys = a->gcs_sys;
  s->gcs_comp = a->gcs_comp;
  s->open_req_seq = a->req_seq;
  s->provider = p;
  s->chunk_size = XFER_CHUNK_MAX;
  for (uint32_t i = 0; i < XFER_ARG_MAX; i++)
    s->arg[i] = a->arg[i];
  return XFER_OPEN_DEFERRED;
}

void xfer_on_data(uint8_t session, uint32_t offset, const uint8_t *buf,
                  uint8_t len, uint8_t flags) {
  if (!s_ready || session >= XFER_MAX_SESSIONS)
    return;
  xfer_session_t *s = &s_sessions[session];
  if (s->state != XFER_ST_ACTIVE || s->dir != XFER_DIR_UPLOAD)
    return;

  /* Contiguous-only: a dup (offset < cursor) or a gap (offset > cursor) is
   * ignored; the next periodic XFER_ACK{cursor} tells the GCS where to resume. */
  s->rx_activity = true; /* any chunk (dup/gap/accepted) is GCS liveness */
  if (offset == s->cursor && len > 0 && s->provider && s->provider->write) {
    int w = s->provider->write(s, offset, buf, len);
    if (w > 0)
      s->cursor += (uint32_t)w; /* accepted (enqueued); 0 = backpressure, hold */
  }
  if (flags & XFER_F_EOF) {
    /* Final chunk seen; once the contiguous cursor has reached it, finish. */
    if (offset + len <= s->cursor) {
      s->total_size = s->cursor;
      if (s_tx && s_tx->ack)
        s_tx->ack(s, XFER_F_DONE, XFER_RES_ACCEPTED, s->cursor);
      s->state = XFER_ST_DONE_LINGER;
      s->last_emit_ms = s->last_rx_ms; /* linger from now */
    }
  }
  if (flags & XFER_F_ABORT) {
    if (s->provider && s->provider->close)
      s->provider->close(s, XFER_RES_FAILED);
    session_free(s);
  }
}

void xfer_on_ack(uint8_t session, uint32_t next_offset, uint8_t flags) {
  if (!s_ready || session >= XFER_MAX_SESSIONS)
    return;
  xfer_session_t *s = &s_sessions[session];
  if (s->dir != XFER_DIR_DOWNLOAD)
    return;
  if (s->state != XFER_ST_ACTIVE && s->state != XFER_ST_DONE_LINGER)
    return;

  s->info_acked = true;
  if (flags & XFER_F_DONE) {
    if (s->provider && s->provider->close)
      s->provider->close(s, XFER_RES_ACCEPTED);
    session_free(s);
    return;
  }
  if (flags & XFER_F_ABORT) {
    if (s->provider && s->provider->close)
      s->provider->close(s, XFER_RES_FAILED);
    session_free(s);
    return;
  }
  /* Resume/refill: rewind to the GCS's lowest-missing byte (idempotent re-read).
   * NAK forces it even if next_offset == cursor. */
  if (next_offset < s->cursor || (flags & XFER_F_NAK)) {
    if (!is_stream(s))
      s->cursor = next_offset;
    if (s->state == XFER_ST_DONE_LINGER)
      s->state = XFER_ST_ACTIVE; /* reopened the tail */
  }
}

int xfer_on_close(uint8_t session, uint8_t req_seq, uint8_t result) {
  if (!s_ready || session >= XFER_MAX_SESSIONS)
    return XFER_RES_TEMPORARILY_REJECTED;
  xfer_session_t *s = &s_sessions[session];
  if (s->state == XFER_ST_FREE)
    return XFER_RES_ACCEPTED; /* already closed — idempotent */
  s->close_pending = true;
  s->close_req_seq = req_seq;
  (void)result;
  return XFER_OPEN_DEFERRED; /* tick emits the COMMAND_ACK + tears down */
}

/* ---- XFER-task tick ------------------------------------------------------ */
/* Run the deferred provider->open and emit COMMAND_ACK + XFER_INFO. */
static void tick_pending_open(xfer_session_t *s, uint32_t now_ms) {
  uint32_t total = 0;
  int rc = s->provider->open ? s->provider->open(s, NULL, &total) : -1;
  /* provider->open is given a NULL args here (everything it needs is already on
   * the session); providers that need the raw args keep them at open-stash time. */
  if (rc < 0) {
    if (s_tx && s_tx->command_ack)
      s_tx->command_ack(s, XFER_WIRE_MSGID_OPEN, s->open_req_seq,
                        XFER_RES_FAILED, (int32_t)rc);
    session_free(s);
    return;
  }
  if (s->dir == XFER_DIR_DOWNLOAD)
    s->total_size = (s->mode == XFER_MODE_STREAM) ? XFER_SIZE_STREAM : total;
  else
    s->total_size = XFER_SIZE_STREAM; /* upload length unknown until EOF */

  s->state = XFER_ST_ACTIVE;
  s->last_rx_ms = now_ms;
  s->last_emit_ms = now_ms;
  s->next_due_ms = now_ms;
  if (s_tx && s_tx->command_ack)
    s_tx->command_ack(s, XFER_WIRE_MSGID_OPEN, s->open_req_seq,
                      XFER_RES_ACCEPTED, 0);
  if (s_tx && s_tx->info)
    s_tx->info(s, XFER_RES_ACCEPTED, s->chunk_size, s->total_size, 0);
}

/* Download: emit up to `budget` chunks from the cursor; returns chunks emitted. */
static int tick_download(xfer_session_t *s, uint32_t now_ms, int budget) {
  int emitted = 0;
  uint8_t buf[XFER_CHUNK_MAX];
  while (budget > 0 && s->state == XFER_ST_ACTIVE) {
    if (s->cursor >= s->total_size) {
      /* Nothing left: send a zero-length EOF marker and linger for the ack. */
      if (s_tx && s_tx->data)
        s_tx->data(s, XFER_F_EOF, 0, s->cursor, buf);
      s->state = XFER_ST_DONE_LINGER;
      s->last_emit_ms = now_ms;
      emitted++;
      break;
    }
    uint16_t want = s->chunk_size;
    if ((uint32_t)want > s->total_size - s->cursor)
      want = (uint16_t)(s->total_size - s->cursor);
    int n = s->provider->read ? s->provider->read(s, s->cursor, buf, want) : -1;
    if (n <= 0)
      break; /* transient read shortfall; retry next tick */
    bool last = (s->cursor + (uint32_t)n) >= s->total_size;
    uint8_t flags = last ? XFER_F_EOF : XFER_F_NONE;
    if (s_tx && s_tx->data)
      s_tx->data(s, flags, (uint8_t)n, s->cursor, buf);
    s->cursor += (uint32_t)n;
    s->last_emit_ms = now_ms;
    s->last_rx_ms = now_ms; /* progress == liveness (idle-timeout base) */
    emitted++;
    budget--;
    if (last) {
      s->state = XFER_ST_DONE_LINGER;
      break;
    }
  }
  return emitted;
}

/* Stream: emit one due sample (best-effort, never rewinds). */
static int tick_stream(xfer_session_t *s, uint32_t now_ms, uint32_t tx_overflow) {
  static uint32_t s_last_overflow;
  if (s->rate_hz == 0)
    return 0;
  if ((int32_t)(now_ms - s->next_due_ms) < 0)
    return 0;
  uint32_t period = 1000u / s->rate_hz;
  if (period == 0)
    period = 1;
  s->next_due_ms = now_ms + period;
  if (tx_overflow != s_last_overflow) { /* backpressure rising -> skip stream */
    s_last_overflow = tx_overflow;
    return 0;
  }
  uint8_t buf[XFER_CHUNK_MAX];
  int n = s->provider->poll ? s->provider->poll(s, buf, s->chunk_size) : 0;
  if (n <= 0)
    return 0;
  if (s_tx && s_tx->data)
    s_tx->data(s, XFER_F_NONE, (uint8_t)n, s->cursor, buf);
  s->cursor += (uint32_t)n;
  s->last_emit_ms = now_ms;
  return 1;
}

/* Upload: periodic cumulative XFER_ACK so the GCS knows where to resume. */
static void tick_upload(xfer_session_t *s, uint32_t now_ms) {
  if (s->rx_activity) { /* refresh liveness only when chunks are still arriving,
                         * so a vanished GCS is reaped by the idle timeout */
    s->last_rx_ms = now_ms;
    s->rx_activity = false;
  }
  if ((uint32_t)(now_ms - s->last_emit_ms) >= XFER_ACK_PERIOD_MS) {
    if (s_tx && s_tx->ack)
      s_tx->ack(s, XFER_F_NONE, XFER_RES_ACCEPTED, s->cursor);
    s->last_emit_ms = now_ms;
  }
}

int xfer_tick(uint32_t now_ms, uint32_t tx_overflow, int chunk_budget) {
  if (!s_ready)
    return 0;
  /* Rotate which session gets first claim on the shared chunk_budget each tick,
   * so two concurrent downloads share the link fairly instead of the lower slot
   * always starving the higher one. */
  static uint8_t s_rr;
  int emitted = 0;
  for (uint8_t k = 0; k < XFER_MAX_SESSIONS; k++) {
    uint8_t i = (uint8_t)((s_rr + k) % XFER_MAX_SESSIONS);
    xfer_session_t *s = &s_sessions[i];

    /* Deferred close: emit the COMMAND_ACK and tear down. */
    if (s->close_pending) {
      if (s->provider && s->provider->close)
        s->provider->close(s, XFER_RES_ACCEPTED);
      if (s_tx && s_tx->command_ack)
        s_tx->command_ack(s, XFER_WIRE_MSGID_CLOSE, s->close_req_seq,
                          XFER_RES_ACCEPTED, 0);
      session_free(s);
      continue;
    }

    /* Run a deferred open first; if it goes ACTIVE, fall straight into emission
     * this same tick (open + first chunk together — lower latency, and an
     * offset-past-EOF open finishes immediately). */
    if (s->state == XFER_ST_PENDING_OPEN)
      tick_pending_open(s, now_ms);

    switch (s->state) {
    case XFER_ST_FREE:
    case XFER_ST_PENDING_OPEN: /* unreachable (handled above) */
      break;
    case XFER_ST_ACTIVE:
      if (s->dir == XFER_DIR_DOWNLOAD) {
        if (s->mode == XFER_MODE_STREAM)
          emitted += tick_stream(s, now_ms, tx_overflow);
        else
          emitted += tick_download(s, now_ms, chunk_budget - emitted);
      } else {
        tick_upload(s, now_ms);
      }
      /* INFO retransmit until the peer first responds. */
      if (!s->info_acked &&
          (uint32_t)(now_ms - s->last_emit_ms) >= XFER_INFO_RETRY_MS) {
        if (s_tx && s_tx->info)
          s_tx->info(s, XFER_RES_ACCEPTED, s->chunk_size, s->total_size, 0);
        s->last_emit_ms = now_ms;
      }
      /* Idle timeout. */
      if ((uint32_t)(now_ms - s->last_rx_ms) >= XFER_IDLE_MS) {
        if (s->provider && s->provider->close)
          s->provider->close(s, XFER_RES_FAILED);
        session_free(s);
      }
      break;
    case XFER_ST_DONE_LINGER:
      if ((uint32_t)(now_ms - s->last_emit_ms) >= XFER_DONE_LINGER_MS) {
        if (s->provider && s->provider->close)
          s->provider->close(s, XFER_RES_ACCEPTED);
        session_free(s);
      }
      break;
    }
  }
  s_rr = (uint8_t)((s_rr + 1u) % XFER_MAX_SESSIONS);
  return emitted;
}
