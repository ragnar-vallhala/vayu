/**
 * @file src/comm/xfer/providers/stream_provider.c
 * @brief STREAM xfer provider — open-ended download of a named live source.
 *
 * Generic and storage-free: a "source" is a named poll function (registered by
 * firmware/tests) that fills the next due sample. XFER_OPEN selects one by name
 * (`arg`) and rate (`rate_hz`); the SM's stream path (tick_stream) calls poll at
 * that cadence, never rewinds (best-effort), and total_size is the stream
 * sentinel. Proves the substrate carries live data, not just files. Design:
 * docs/plans/navlink-xfer-substrate.md.
 */
#include "comm/xfer/xfer_providers.h"

#ifndef XFER_STREAM_MAX_SOURCES
#define XFER_STREAM_MAX_SOURCES 4u
#endif

static struct {
  const char *name;
  xfer_stream_source_fn poll;
} s_sources[XFER_STREAM_MAX_SOURCES];
static uint8_t s_source_count;

/* Per-session bound source (indexed by session id) — avoids stashing a function
 * pointer in s->user (which is a void*; that cast is non-portable). */
static xfer_stream_source_fn s_bound[XFER_MAX_SESSIONS];

static bool name_eq(const char *a, const char *b) {
  for (uint16_t i = 0; i < XFER_ARG_MAX; i++) {
    if (a[i] != b[i])
      return false;
    if (a[i] == '\0')
      return true;
  }
  return true;
}

int xfer_stream_register_source(const char *name, xfer_stream_source_fn poll) {
  if (name == NULL || poll == NULL || s_source_count >= XFER_STREAM_MAX_SOURCES)
    return -1;
  s_sources[s_source_count].name = name;
  s_sources[s_source_count].poll = poll;
  s_source_count++;
  return 0;
}

static int stream_open(xfer_session_t *s, const xfer_open_args_t *a,
                       uint32_t *total_out) {
  (void)a;
  if (s->session >= XFER_MAX_SESSIONS)
    return -1;
  for (uint8_t i = 0; i < s_source_count; i++) {
    if (name_eq(s->arg, s_sources[i].name)) {
      s_bound[s->session] = s_sources[i].poll;
      if (total_out)
        *total_out = XFER_SIZE_STREAM; /* SM also forces this for stream mode */
      return 0;
    }
  }
  return -2; /* unknown stream name -> FAILED */
}

static int stream_poll(xfer_session_t *s, uint8_t *buf, uint16_t max) {
  if (s->session >= XFER_MAX_SESSIONS || s_bound[s->session] == NULL)
    return 0;
  return s_bound[s->session](buf, max);
}

static void stream_close(xfer_session_t *s, int result) {
  (void)result;
  if (s->session < XFER_MAX_SESSIONS)
    s_bound[s->session] = NULL;
}

static const xfer_provider_t STREAM_PROVIDER = {
    .service_id = XFER_SVC_STREAM,
    .name = "stream",
    .open = stream_open,
    .read = NULL,
    .write = NULL,
    .poll = stream_poll,
    .close = stream_close,
};

int stream_provider_register(void) {
  return xfer_register_provider(&STREAM_PROVIDER);
}
