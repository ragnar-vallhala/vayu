/**
 * @file src/storage/imu_hs_log.c
 * @brief High-speed IMU-to-SD stream (HSL). Format spec in the header.
 *
 * Threading: a textbook SPSC ring of whole 512 B sectors.
 *   producer = the IMU sample task, owns s_head and s_fill
 *   consumer = the FS task, owns s_tail, the ring cursor and every vfs_* call
 * Nothing is shared but those two indices and one flag, so there is no lock.
 * `seq` is stamped by the CONSUMER at write time, not by the producer: it
 * numbers the order sectors reach the card, which is the only order a reader
 * can reconstruct.
 */
#include "storage/imu_hs_log.h"

#include "storage/fs_owner.h" /* vayu_log, fs_owner_logs_suppressed */
#include "control/flight_mode.h"
#include "dsp/gyro_notch.h"
#include "sys/state.h"
#include "sys/sys_utils.h" /* get_timestamp_unix, time_sync_is_synced */
#include "utils.h"         /* v_memcpy */
#include "variables.h"
#include "vfs.h"

#define HSL_RING_SECTORS ((uint32_t)(HSL_FILE_SIZE / HSL_SECTOR_BYTES) - 1u)

/* ===========================================================================
 * Streams
 *
 * One ring per stream: each has a different producer task, and a single shared
 * ring would have three writers, which is not an SPSC queue any more. Per
 * stream the producer owns head+fill and the FS task owns tail; nothing else
 * is shared, so there are no locks.
 * =========================================================================== */
typedef struct {
  uint8_t *bufs; /* n_bufs * HSL_SECTOR_BYTES                          */
  uint8_t n_bufs;
  uint8_t stream_id;
  uint8_t rec_bytes;
  uint16_t cap;       /* records per sector                                 */
  uint32_t decim_cyc; /* min cycles between records (0 = take everything)   */
  /* producer-owned */
  volatile uint8_t head;
  uint16_t fill;
  uint32_t last_cyc;
  /* consumer-owned */
  volatile uint8_t tail;
} hsl_stream_t;

static uint8_t s_imu_bufs[HSL_IMU_BUFFERS][HSL_SECTOR_BYTES];
static uint8_t s_act_bufs[HSL_ACT_BUFFERS][HSL_SECTOR_BYTES];
static uint8_t s_vrt_bufs[HSL_VRT_BUFFERS][HSL_SECTOR_BYTES];

enum { HSL_S_IMU = 0, HSL_S_ACT, HSL_S_VRT, HSL_N_STREAMS };

static hsl_stream_t s_streams[HSL_N_STREAMS] = {
    [HSL_S_IMU] = {.bufs = &s_imu_bufs[0][0],
                   .n_bufs = HSL_IMU_BUFFERS,
                   .stream_id = HSL_STREAM_IMU,
                   .rec_bytes = HSL_IMU_REC_BYTES,
                   .cap = HSL_BLOCK_PAYLOAD_BYTES / HSL_IMU_REC_BYTES,
                   .decim_cyc = 0u},
    [HSL_S_ACT] = {.bufs = &s_act_bufs[0][0],
                   .n_bufs = HSL_ACT_BUFFERS,
                   .stream_id = HSL_STREAM_ACT,
                   .rec_bytes = HSL_ACT_REC_BYTES,
                   .cap = HSL_BLOCK_PAYLOAD_BYTES / HSL_ACT_REC_BYTES,
                   .decim_cyc = (uint32_t)SYS_CLOCK_FREQ / HSL_ACT_RATE_HZ},
    [HSL_S_VRT] = {.bufs = &s_vrt_bufs[0][0],
                   .n_bufs = HSL_VRT_BUFFERS,
                   .stream_id = HSL_STREAM_VRT,
                   .rec_bytes = HSL_VRT_REC_BYTES,
                   .cap = HSL_BLOCK_PAYLOAD_BYTES / HSL_VRT_REC_BYTES,
                   .decim_cyc = (uint32_t)SYS_CLOCK_FREQ / HSL_VRT_RATE_HZ},
};

/* Consumer-owned. */
static vfs_fd_t s_fd = -1;
static uint32_t s_slot = 0;    /* ring slot the next frame goes to         */
static uint32_t s_seq = 1;     /* seq the next frame will carry            */
static uint32_t s_wraps = 0;   /* times round the ring                     */
static uint32_t s_session = 0; /* armed-period counter                     */
static uint32_t s_since_hdr = 0;

/* Last values the FS task has already emitted an EVENT for. Single writer
 * (the FS task), so comparing them needs no synchronisation. */
static uint32_t s_ev_state = 0;
static uint32_t s_ev_mode = 0xFFFFFFFFu;
static uint32_t s_ev_accel_bad = 0xFFFFFFFFu;
static uint32_t s_ev_notch = 0xFFFFFFFFu;
/* Written by the VERT producer, read by the FS task: one word, one writer. */
static volatile uint32_t s_accel_unhealthy = 0;

/* count -> SI, published by the driver (imu_hs_log_set_scale). */
static float s_gyr_scale = 1.0f;
static float s_acc_scale = 1.0f;

/* Shared: set by the consumer, read by the producers. One flag, one writer. */
static volatile bool s_active = false;
static volatile uint32_t s_dropped = 0;

/* ===========================================================================
 * Little-endian stores. The buffers are byte-addressed, which is also what
 * keeps these alignment-safe.
 * =========================================================================== */
static void put_u16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void put_f32(uint8_t *p, float v) {
  uint32_t bits;
  v_memcpy(&bits, &v, sizeof(bits));
  put_u32(p, bits);
}

static uint16_t get_u16(const uint8_t *p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get_u32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

/* 0..1 -> full-scale u16, clamped. */
static uint16_t unit_to_u16(float v) {
  if (!(v > 0.0f)) { /* also catches NaN */
    return 0u;
  }
  if (v >= 1.0f) {
    return 0xFFFFu;
  }
  return (uint16_t)(v * 65535.0f);
}

/* ===========================================================================
 * Producers. Each runs in its own task; all of this is RAM-only and must stay
 * cheap enough for the 2 kHz caller.
 * =========================================================================== */
/* Claim room for one record, or NULL if the stream is closed, decimated away,
 * or its ring is full. Fills in the frame/block header on a fresh sector. */
static uint8_t *stream_claim(hsl_stream_t *st, uint32_t t_cyc) {
  if (!s_active) {
    return NULL;
  }
  /* Decimate on the cycle stamp, so the rate holds whatever rate the caller
   * happens to run at. Wrap-safe unsigned delta. */
  if (st->decim_cyc != 0u && (uint32_t)(t_cyc - st->last_cyc) < st->decim_cyc) {
    return NULL;
  }

  uint8_t *b = st->bufs + (uint32_t)st->head * HSL_SECTOR_BYTES;
  if (st->fill == 0u) {
    b[0] = (uint8_t)HSL_TYPE_BLOCK;
    b[1] = (uint8_t)HSL_RING_SENTINEL;
    put_u16(&b[2], (uint16_t)(HSL_SECTOR_BYTES - HSL_FRAME_HDR_BYTES));
    b[4] = st->stream_id;
    /* b[5] is the session tag and b[8..11] the seq, both stamped by the
     * consumer in ring_write -- only it knows the write order. */
    put_u32(&b[12], t_cyc); /* t_first */
  }
  return b + HSL_FRAME_HDR_BYTES + HSL_BLOCK_HDR_BYTES +
         (uint32_t)st->fill * st->rec_bytes;
}

/* Count the record just written and publish the sector if it is full. */
static void stream_commit(hsl_stream_t *st, uint32_t t_cyc) {
  uint8_t *b = st->bufs + (uint32_t)st->head * HSL_SECTOR_BYTES;
  st->last_cyc = t_cyc;
  st->fill++;
  put_u16(&b[6], st->fill); /* n      */
  put_u32(&b[16], t_cyc);   /* t_last */

  if (st->fill < st->cap) {
    return;
  }
  st->fill = 0u;

  /* One buffer always stays empty so head==tail means empty. */
  uint8_t next = (uint8_t)((st->head + 1u) % st->n_bufs);
  if (next == st->tail) {
    s_dropped++; /* SD is not keeping up; this sector is refilled instead */
    return;
  }
  st->head = next;
}

/** @noreq high-rate sample capture; RAM-only sector fill */
void imu_hs_log_sample(const int16_t gyr[3], const int16_t acc[3],
                       uint32_t t_cyc) {
  hsl_stream_t *st = &s_streams[HSL_S_IMU];
  uint8_t *rec = stream_claim(st, t_cyc);
  if (rec == NULL) {
    return;
  }
  v_memcpy(rec, gyr, 6u);
  v_memcpy(rec + 6, acc, 6u);
  stream_commit(st, t_cyc);
}

/** @noreq actuator capture; RAM-only, decimated to HSL_ACT_RATE_HZ */
void imu_hs_log_act(const float motors[4], float throttle, uint16_t flags,
                    uint32_t t_cyc) {
  hsl_stream_t *st = &s_streams[HSL_S_ACT];
  uint8_t *rec = stream_claim(st, t_cyc);
  if (rec == NULL) {
    return;
  }
  for (uint32_t i = 0; i < 4u; i++) {
    put_u16(&rec[i * 2u], unit_to_u16(motors[i]));
  }
  put_u16(&rec[8], unit_to_u16(throttle));
  put_u16(&rec[10], flags);
  stream_commit(st, t_cyc);
}

/** @noreq vertical-estimator capture; RAM-only, decimated to HSL_VRT_RATE_HZ */
void imu_hs_log_vert(const hsl_vert_sample_t *v, uint32_t t_cyc) {
  if (v == NULL) {
    return;
  }
  /* Published even when the record itself is decimated away: the FS task
   * watches this for the accel-health EVENT, and a latch that lasted less than
   * a 20 Hz sample is exactly the one worth annotating. */
  s_accel_unhealthy = (v->flags & HSL_VRT_F_ACCEL_UNHEALTHY) ? 1u : 0u;

  hsl_stream_t *st = &s_streams[HSL_S_VRT];
  uint8_t *rec = stream_claim(st, t_cyc);
  if (rec == NULL) {
    return;
  }
  put_f32(&rec[0], v->baro_altitude);
  put_f32(&rec[4], v->agl);
  put_f32(&rec[8], v->agl_tof);
  put_f32(&rec[12], v->altitude);
  put_f32(&rec[16], v->climb_rate);
  put_f32(&rec[20], v->accel_bias);
  put_u16(&rec[24], v->flags);
  put_u16(&rec[26], 0u);
  stream_commit(st, t_cyc);
}

/* ===========================================================================
 * Consumer -- FS task.
 * =========================================================================== */
/** @noreq lays one FMT field descriptor */
static void put_field(uint8_t *p, const char *name, uint8_t ftype,
                      float scale) {
  uint32_t i = 0;
  for (; i < 8u && name[i] != '\0'; i++) {
    p[i] = (uint8_t)name[i];
  }
  for (; i < 8u; i++) {
    p[i] = 0u;
  }
  p[8] = ftype;
  p[9] = 0u;
  p[10] = 0u;
  p[11] = 0u;
  uint32_t bits;
  v_memcpy(&bits, &scale, sizeof(bits));
  put_u32(&p[12], bits);
}

typedef struct {
  const char *name;
  uint8_t ftype;
  float scale;
} hsl_field_t;

/* Write one FMT frame at `f`; returns the byte after it. */
static uint8_t *emit_fmt(uint8_t *f, uint8_t stream_id, uint8_t rec_bytes,
                         uint16_t rate_hz, const hsl_field_t *fields,
                         uint8_t n) {
  const uint16_t pay = (uint16_t)(8u + (uint32_t)n * HSL_FMT_FIELD_BYTES);
  f[0] = (uint8_t)HSL_TYPE_FMT;
  f[1] = 0u;
  put_u16(&f[2], pay);
  f[4] = stream_id;
  f[5] = rec_bytes;
  put_u16(&f[6], rate_hz);
  f[8] = n;
  f[9] = 0u;
  f[10] = 0u;
  f[11] = 0u;
  uint8_t *fld = &f[HSL_FRAME_HDR_BYTES + 8u];
  for (uint32_t i = 0; i < n; i++) {
    put_field(fld + i * HSL_FMT_FIELD_BYTES, fields[i].name, fields[i].ftype,
              fields[i].scale);
  }
  return f + HSL_FRAME_HDR_BYTES + pay;
}

/* Sector 0: file header + FMT + PAD. Rewritten whenever the cursor hint is
 * updated, which also keeps FMT's scales current if the IMU range changed.
 * Static, not a local: this runs in the FS task, whose stack budget on the
 * F401 will not absorb 512 B for a buffer used a few times a second. */
static uint8_t s_preamble[HSL_SECTOR_BYTES];

/** @noreq builds sector 0 from the current cursor */
static void build_preamble(void) {
  for (uint32_t i = 0; i < HSL_SECTOR_BYTES; i++) {
    s_preamble[i] = 0u;
  }
  uint8_t *h = s_preamble;
  put_u32(&h[0], HSL_MAGIC);
  put_u16(&h[4], (uint16_t)HSL_VERSION);
  put_u16(&h[6], (uint16_t)HSL_FILE_HDR_BYTES);
  put_u32(&h[8], (uint32_t)SYS_CLOCK_FREQ);
  put_u32(&h[12], HSL_SECTOR_BYTES); /* ring_start   */
  put_u32(&h[16], HSL_RING_SECTORS); /* ring_sectors */
  put_u32(&h[20], s_slot);           /* head_slot HINT */
  put_u32(&h[24], s_seq);            /* next_seq  HINT */
  put_u32(&h[28], s_wraps);

  /* One FMT per stream, so the file describes every stream it contains with
   * no external schema. Scales come from the driver, not a constant here, so
   * an IMU range change cannot silently mis-scale a recording. The gyro/accel
   * sign carries the sensor -> body axis map (-x, +y, -z); a magnitude
   * spectrum ignores it, an axis-resolved plot does not. */
  const float gs = s_gyr_scale;
  const float as = s_acc_scale;
  const float u16fs = 1.0f / 65535.0f; /* u16 full scale -> 0..1 */
  uint8_t *f = &s_preamble[HSL_FILE_HDR_BYTES];

  f = emit_fmt(f, HSL_STREAM_IMU, HSL_IMU_REC_BYTES,
               (uint16_t)IMU_SAMPLE_FREQ_HZ,
               (const hsl_field_t[]){{"gx", HSL_FTYPE_I16, -gs},
                                     {"gy", HSL_FTYPE_I16, gs},
                                     {"gz", HSL_FTYPE_I16, -gs},
                                     {"ax", HSL_FTYPE_I16, -as},
                                     {"ay", HSL_FTYPE_I16, as},
                                     {"az", HSL_FTYPE_I16, -as}},
               6u);
  f = emit_fmt(f, HSL_STREAM_ACT, HSL_ACT_REC_BYTES, (uint16_t)HSL_ACT_RATE_HZ,
               (const hsl_field_t[]){{"m1", HSL_FTYPE_U16, u16fs},
                                     {"m2", HSL_FTYPE_U16, u16fs},
                                     {"m3", HSL_FTYPE_U16, u16fs},
                                     {"m4", HSL_FTYPE_U16, u16fs},
                                     {"thr", HSL_FTYPE_U16, u16fs},
                                     {"flags", HSL_FTYPE_U16, 1.0f}},
               6u);
  f = emit_fmt(f, HSL_STREAM_VRT, HSL_VRT_REC_BYTES, (uint16_t)HSL_VRT_RATE_HZ,
               (const hsl_field_t[]){{"baro", HSL_FTYPE_F32, 1.0f},
                                     {"agl", HSL_FTYPE_F32, 1.0f},
                                     {"agltof", HSL_FTYPE_F32, 1.0f},
                                     {"alt", HSL_FTYPE_F32, 1.0f},
                                     {"climb", HSL_FTYPE_F32, 1.0f},
                                     {"abias", HSL_FTYPE_F32, 1.0f},
                                     {"flags", HSL_FTYPE_U16, 1.0f},
                                     {"pad", HSL_FTYPE_U16, 1.0f}},
               8u);

  /* PAD out to the sector. Skipped by the generic `len` rule, so no decoder
   * needs to know it exists. */
  uint8_t *pad = f;
  uint32_t used = (uint32_t)(pad - s_preamble);
  pad[0] = (uint8_t)HSL_TYPE_PAD;
  pad[1] = 0u;
  put_u16(&pad[2], (uint16_t)(HSL_SECTOR_BYTES - used - HSL_FRAME_HDR_BYTES));
}

/** @noreq writes sector 0 and forces it to the card */
static bool flush_preamble(void) {
  build_preamble();
  if (vfs_lseek(s_fd, 0, VFS_SEEK_SET) < 0) {
    return false;
  }
  if (vfs_write(s_fd, s_preamble, HSL_SECTOR_BYTES) != (int)HSL_SECTOR_BYTES) {
    return false;
  }
  s_since_hdr = 0;
  return vfs_sync(s_fd) >= 0;
}

/* Write one prepared sector into the ring at the current slot and advance.
 * No sync: the write is 512 B and sector-aligned, so FatFS hands it straight
 * to the card -- it is durable on return. Only the header hint needs syncing. */
static bool ring_write(uint8_t *sec) {
  sec[1] = (uint8_t)HSL_RING_SENTINEL;
  /* Session membership goes in EVERY frame, not just the SESSION frame: the
   * ring can overwrite a session's header while its blocks are still live, and
   * those blocks would otherwise read as a continuation of the session before
   * them. Adjacent sessions cannot alias, so a reader splits on the tag
   * changing. */
  sec[5] = (uint8_t)(s_session & 0xFFu);
  put_u32(&sec[8], s_seq);

  const uint32_t off = HSL_SECTOR_BYTES * (1u + s_slot);
  if (vfs_lseek(s_fd, (long)off, VFS_SEEK_SET) < 0) {
    return false;
  }
  if (vfs_write(s_fd, sec, HSL_SECTOR_BYTES) != (int)HSL_SECTOR_BYTES) {
    return false;
  }
  s_seq++;
  s_slot++;
  if (s_slot >= HSL_RING_SECTORS) {
    s_slot = 0;
    s_wraps++;
  }
  return (++s_since_hdr < HSL_HDR_SYNC_FRAMES) || flush_preamble();
}

/* One sector, reused for the SESSION and EVENT frames. Both are built only in
 * the FS task, and neither outlives its own ring_write, so they can share it --
 * which keeps them off a stack the F401 cannot spare 512 B from. */
static uint8_t s_ctrl[HSL_SECTOR_BYTES];

/* Emit one EVENT frame. FS task only. */
static bool emit_event(uint8_t kind, uint32_t a, uint32_t b) {
  uint8_t *e = s_ctrl;
  for (uint32_t i = 0; i < HSL_SECTOR_BYTES; i++) {
    e[i] = 0u;
  }
  e[0] = (uint8_t)HSL_TYPE_EVENT;
  put_u16(&e[2], (uint16_t)(HSL_SECTOR_BYTES - HSL_FRAME_HDR_BYTES));
  /* e[8..11] seq, stamped by ring_write */
  put_u32(&e[12], hal_cycle_counter_get());
  e[16] = kind;
  put_u32(&e[20], a);
  put_u32(&e[24], b);
  return ring_write(e);
}

/* Emit an EVENT for anything watched that has changed since the last pass.
 * Polled rather than pushed: every one of these is readable state, so nothing
 * outside this module has to know the recorder exists, and there is no
 * cross-task queue to get wrong. The FS task loops every <= 5 ms, which is far
 * finer than anything being annotated. */
static void emit_changes(void) {
  const uint32_t st = (uint32_t)system_state_get();
  if (st != s_ev_state) {
    if (!emit_event(HSL_EV_STATE, st, s_ev_state)) {
      return;
    }
    s_ev_state = st;
  }
  const uint32_t md = (uint32_t)flight_mode_get();
  if (md != s_ev_mode) {
    if (!emit_event(HSL_EV_MODE, md, s_ev_mode)) {
      return;
    }
    s_ev_mode = md;
  }
  const uint32_t bad = s_accel_unhealthy;
  if (bad != s_ev_accel_bad) {
    if (!emit_event(HSL_EV_ACCEL_HEALTH, bad, s_ev_accel_bad)) {
      return;
    }
    s_ev_accel_bad = bad;
  }
  const uint32_t nt = gyro_notch_enabled() ? 1u : 0u;
  if (nt != s_ev_notch) {
    if (!emit_event(HSL_EV_NOTCH, nt, s_ev_notch)) {
      return;
    }
    s_ev_notch = nt;
  }
}

/** @noreq opens the file and emits this armed period's SESSION frame */
static void session_start(void) {
  s_fd = vfs_open(HSL_FILENAME, VFS_O_RDWR);
  if (s_fd < 0) {
    s_fd = -1;
    return;
  }
  s_session++;

  uint8_t *sf = s_ctrl;
  for (uint32_t i = 0; i < HSL_SECTOR_BYTES; i++) {
    sf[i] = 0u;
  }
  sf[0] = (uint8_t)HSL_TYPE_SESSION;
  put_u16(&sf[2], (uint16_t)(HSL_SECTOR_BYTES - HSL_FRAME_HDR_BYTES));
  /* sf[8..11] is seq, stamped by ring_write. */
  put_u32(&sf[12], s_session);
  const uint64_t unix_ms = get_timestamp_unix();
  put_u32(&sf[16], (uint32_t)(unix_ms & 0xFFFFFFFFu));
  put_u32(&sf[20], (uint32_t)(unix_ms >> 32));
  put_u32(&sf[24], hal_cycle_counter_get());
  sf[28] = time_sync_is_synced() ? 1u : 0u;

  if (!ring_write(sf) || !flush_preamble()) {
    vfs_close(s_fd);
    s_fd = -1;
    return;
  }
  /* Discard whatever the producers left queued from before this session. Tail
   * is ours to move, which is why no lock is needed here. */
  for (uint32_t i = 0; i < HSL_N_STREAMS; i++) {
    s_streams[i].tail = s_streams[i].head;
  }
  s_active = true;
}

/** @noreq commits the cursor hint and closes */
static void session_stop(void) {
  s_active = false; /* set FIRST: the producers now cannot touch a stream, so
                     * their partial sectors are ours to finish. */
  if (s_fd >= 0) {
    /* Flush partly-filled sectors. `n` already says how many records they
     * hold, so a short sector is a perfectly ordinary block -- without this a
     * VRT sector would lose up to ~850 ms at the end of every flight. */
    for (uint32_t i = 0; i < HSL_N_STREAMS; i++) {
      hsl_stream_t *st = &s_streams[i];
      if (st->fill > 0u) {
        (void)ring_write(st->bufs + (uint32_t)st->head * HSL_SECTOR_BYTES);
        st->fill = 0u;
      }
    }
    (void)flush_preamble();
    vfs_close(s_fd);
    s_fd = -1;
    vayu_log("hsl: session %u closed, slot %u, %u wraps, %u dropped",
             (unsigned)s_session, (unsigned)s_slot, (unsigned)s_wraps,
             (unsigned)s_dropped);
  }
}

/** @noreq per-loop service: arm-state gating + sector writes */
void imu_hs_log_drain(void) {
  /* Record whenever the props can be spinning. Nothing else needs to know this
   * module exists -- no command and no GCS work; each arm simply appends
   * another session to the ring. */
  const sys_state_t st = system_state_get();
  const bool want = (st == SYSTEM_STATE_ARMED) || (st == SYSTEM_STATE_IN_AIR);

  if (!want || fs_owner_logs_suppressed()) {
    /* Also stop for a bulk transfer: a held fd would occupy one of FatFS's four
     * slots for the whole download, and interleaving two files corrupts the
     * read-back (same reason the blackbox quiesces). */
    if (s_fd >= 0) {
      /* Record the transition that is ENDING this session before closing, or
       * nothing in the file would mark where the flight stopped -- only that
       * the blocks ran out, which is also what a card failure looks like. */
      emit_changes();
      session_stop();
    }
    return;
  }

  if (s_fd < 0) {
    session_start();
    if (s_fd < 0) {
      return;
    }
  }

  emit_changes();

  for (uint32_t i = 0; i < HSL_N_STREAMS; i++) {
    hsl_stream_t *st = &s_streams[i];
    while (st->tail != st->head) {
      if (!ring_write(st->bufs + (uint32_t)st->tail * HSL_SECTOR_BYTES)) {
        session_stop(); /* the card is unhappy; do not spin on it */
        return;
      }
      st->tail = (uint8_t)((st->tail + 1u) % st->n_bufs);
    }
  }
}

/* ===========================================================================
 * Observability
 * =========================================================================== */
/** @noreq trivial setter; driver publishes its scales for the FMT frame */
void imu_hs_log_set_scale(float gyr_dps_per_count, float acc_mps2_per_count) {
  if (gyr_dps_per_count > 0.0f) {
    s_gyr_scale = gyr_dps_per_count;
  }
  if (acc_mps2_per_count > 0.0f) {
    s_acc_scale = acc_mps2_per_count;
  }
}

/** @noreq trivial counter accessor */
uint32_t imu_hs_log_dropped(void) { return s_dropped; }
/** @noreq trivial cursor accessor */
uint32_t imu_hs_log_head_slot(void) { return s_slot; }
/** @noreq trivial geometry accessor */
uint32_t imu_hs_log_ring_sectors(void) { return HSL_RING_SECTORS; }
/** @noreq trivial counter accessor */
uint32_t imu_hs_log_session(void) { return s_session; }
/** @noreq trivial counter accessor */
uint32_t imu_hs_log_seq(void) { return s_seq; }
/** @noreq trivial counter accessor */
uint32_t imu_hs_log_wraps(void) { return s_wraps; }
/** @noreq trivial state accessor */
bool imu_hs_log_active(void) { return s_active; }

/**
 * Commit the file's full cluster chain, and recover where the ring left off.
 *
 * Why commit up front: allocating as we go would run create_chain() -- a FAT
 * scan plus a FAT sector write -- at every cluster boundary, i.e. every ~1.3 s
 * at 24 KB/s with 32 KB clusters, against a buffer ring that absorbs ~82 ms.
 * And f_sync on a GROWING file must also rewrite the directory entry's size,
 * so a power cut loses everything logged since the last sync -- which is
 * exactly the flight whose vibration you wanted to look at. With the size
 * committed here, every sector we wrote is readable no matter when the power
 * went away.
 *
 * Why not vfs_preallocate(): it zero-fills the whole file 512 B at a time.
 * f_lseek past EOF in write mode allocates the identical chain via
 * create_chain's forced stretch (ff.c, "Cluster following loop") for ~8 FAT
 * sector writes instead of 65536 data writes. The clusters then hold whatever
 * was on the card before, which is what HSL_RING_SENTINEL is for.
 *
 * Recovery: the hint in sector 0 is up to HSL_HDR_SYNC_FRAMES sectors stale
 * after a power cut, so resuming exactly at it would re-use seq values that
 * orphaned sectors already carry, and a reader sorting by seq could not tell
 * them apart. Resuming HSL_SEQ_RESUME_MARGIN further on cannot: every sector
 * this boot writes outranks every orphan, so an orphan shows up as what it is
 * -- older data, correctly ordered, with a gap in front of it.
 *
 * @implements LOG-SD-001
 */
void imu_hs_log_boot_init(void) {
  vfs_fd_t fd = vfs_open(HSL_FILENAME, VFS_O_RDWR | VFS_O_CREAT);
  if (fd < 0) {
    return; /* not a PANIC: the aircraft flies fine without a recording */
  }

  const long cur = vfs_lseek(fd, 0, VFS_SEEK_END);
  bool fresh = (cur < (long)HSL_FILE_SIZE);
  if (fresh) {
    vfs_lseek(fd, (long)(HSL_FILE_SIZE - 1u), VFS_SEEK_SET);
    uint8_t z = 0;
    vfs_write(fd, &z, 1);
    vfs_sync(fd);
  } else {
    /* Resume the ring where the last power cycle left it. */
    vfs_lseek(fd, 0, VFS_SEEK_SET);
    uint8_t hdr[HSL_FILE_HDR_BYTES];
    if (vfs_read(fd, hdr, sizeof(hdr)) == (int)sizeof(hdr) &&
        get_u32(&hdr[0]) == HSL_MAGIC && get_u16(&hdr[4]) == HSL_VERSION &&
        get_u32(&hdr[16]) == HSL_RING_SECTORS) {
      const uint32_t slot = get_u32(&hdr[20]);
      if (slot < HSL_RING_SECTORS) {
        s_slot = slot;
        s_seq = get_u32(&hdr[24]) + HSL_SEQ_RESUME_MARGIN;
        s_wraps = get_u32(&hdr[28]);
      }
    } else {
      fresh = true; /* unreadable header: start the ring over */
    }
  }

  if (fresh) {
    s_slot = 0;
    s_seq = 1;
    s_wraps = 0;
  }

  s_fd = fd;
  (void)flush_preamble();
  s_fd = -1;
  vfs_close(fd);
}
