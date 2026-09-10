#ifndef VAYU_STORAGE_IMU_HS_LOG_H
#define VAYU_STORAGE_IMU_HS_LOG_H

/**
 * @file include/storage/imu_hs_log.h
 * @brief High-speed IMU-to-SD stream for frequency analysis (HSL).
 *
 * Records the gyro and accel at the FULL IMU sample rate (IMU_SAMPLE_FREQ_HZ,
 * 2 kHz -> 1 kHz Nyquist) to its own SD file, so vibration peaks can be found
 * offline with an FFT. This is what the FFT notch needs to be tuned against,
 * and what the 2026-09-07 flyaway had to be reconstructed from 12 IMU_RAW
 * telemetry samples for want of.
 *
 * Deliberately NOT the blackbox logger and NOT telemetry:
 *   - the blackbox does open->lseek->write->sync->close PER RECORD; at 2 kHz
 *     that is 2000 SD transactions a second, which the card cannot do;
 *   - telemetry is link-limited to ~50 Hz, an order of magnitude short of the
 *     prop frequencies we are hunting.
 * So HSL batches samples into 512 B sectors and hands whole sectors to the FS
 * task, ~49 writes/s at 24 KB/s.
 *
 * Only armed time is recorded, and sessions ACCUMULATE into a circular ring:
 * the file always holds the last ~22 min of armed flight, however many arms
 * that spans, and never stops recording. Nothing is ever deleted or truncated
 * -- the oldest sector is simply overwritten.
 *
 * ===========================================================================
 * ON-DISK FORMAT -- "HSL1"
 * ===========================================================================
 * Payload-independent by construction: the file is a stream of self-delimiting
 * frames, and every frame declares its own length. A decoder that meets a type
 * it does not know SKIPS `len` bytes and keeps going. That single rule is the
 * whole extension contract -- new data goes in a new frame type or a new
 * stream, and tools written today keep reading files written later.
 *
 * All integers little-endian (the only endianness this firmware runs on; the
 * magic doubles as the check, since it reads as ASCII "HSL1" only when the
 * reader's byte order matches).
 *
 * Layout: sector 0 is the PREAMBLE, every sector after it is one RING slot.
 *
 *   FILE HEADER (32 B, at offset 0, inside the preamble sector)
 *     u32 magic         0x314C5348 = "HSL1"
 *     u16 version       1
 *     u16 hdr_len       32 -- skip this many bytes to reach the first preamble
 *                          frame; a later header may be longer and old readers
 *                          still land on it
 *     u32 clock_hz      the unit of EVERY cycle stamp in this file
 *     u32 ring_start    byte offset of ring slot 0 (= 512)
 *     u32 ring_sectors  slots in the ring
 *     u32 head_slot     HINT: slot the next frame will be written to
 *     u32 next_seq      HINT: seq the next frame will carry
 *     u32 wraps         how many times the ring has been round
 *
 *     head_slot/next_seq are a HINT, not the truth: they are rewritten every
 *     HSL_HDR_SYNC_FRAMES sectors, so a power cut leaves them up to that many
 *     behind. A reader must NOT use them -- it recovers the order from `seq`
 *     (below), which is exact. The firmware uses them only to know where to
 *     resume, and deliberately resumes at next_seq + HSL_SEQ_RESUME_MARGIN so
 *     that a resumed write can never re-use a seq an orphaned pre-crash sector
 *     already carries.
 *
 *   FRAME (repeats)
 *     u8  type
 *     u8  flags
 *     u16 len           payload bytes following this 4-byte header
 *     u8  payload[len]
 *
     In the PREAMBLE, flags is 0 and frames are packed to fill the sector.
 *     In the RING, every frame is exactly one sector (len = 508) and flags is
 *     HSL_RING_SENTINEL. The sentinel is what distinguishes a slot this
 *     firmware wrote from a slot still holding whatever the card had before
 *     the file's clusters were allocated -- see imu_hs_log_boot_init.
 *
 *   COMMON RING-FRAME PREFIX (payload bytes 0..7, every ring type)
 *     u8  [0]     type-specific
 *     u8  [1]     session_tag -- low 8 bits of the owning session
 *     u16 [2..3]  type-specific
 *     u32 [4..7]  seq -- see ORDERING
 *
 *     The tag and the seq sit at a fixed offset in EVERY ring frame because
 *     the consumer stamps them uniformly, and because a reader must be able to
 *     order and group a frame whose type it does not recognise. Type-specific
 *     payload starts at byte 8.
 *
 *   type 0x00 PAD -- nothing. Rounds the preamble up to a whole sector so
 *                    every ring slot is sector-aligned. Needs no decoder
 *                    support: skipped by the same `len` rule as any
 *                    unrecognised frame.
 *
 *   type 0x01 FMT -- declares one stream. Lives in the preamble, so the file
 *                    is readable with no external schema.
 *     u8  stream_id
 *     u8  rec_bytes     size of one record in this stream's BLOCKs
 *     u16 rate_hz       nominal sample rate
 *     u8  n_fields
 *     u8  reserved[3]
 *     then n_fields x 16 B, in record order:
 *       char name[8]    NUL-padded
 *       u8   ftype      1=i16 2=u16 3=i32 4=f32
 *       u8   reserved[3]
 *       f32  scale      multiplier from stored units to SI
 *
 *     Binary rather than a text schema on purpose: formatting a float on this
 *     target drags in newlib's %f machinery for the sake of a string a decoder
 *     would only parse back into the float we already had.
 *
 *   type 0x02 BLOCK -- n records of one stream. One ring slot.
 *     [0]      u8  stream_id
 *     [1]      u8  session_tag
 *     [2..3]   u16 n
 *     [4..7]   u32 seq
 *     [8..11]  u32 t_first    cycle stamp of record 0
 *     [12..15] u32 t_last     cycle stamp of record n-1
 *     [16..]   u8  rec[n * rec_bytes]
 *
 *     A sector holds up to 492/rec_bytes records; `n` says how many are really
 *     there, so a short sector (a stream flushed at disarm) is an ordinary
 *     block and needs no special case.
 *
 *     Per-record time is INTERPOLATED across [t_first, t_last]. A stamp per
 *     record would cost 33% more than the samples themselves, and the
 *     sub-sample jitter it would buy back does not survive an FFT.
 *
 *     The stamps are the raw 32-bit DWT counter and WRAP (~51 s at 84 MHz).
 *     Consecutive frames are ~20 ms apart, so a reader accumulates wrap-safe
 *     u32 deltas and never needs a wider counter on the wire.
 *
 *   type 0x04 SESSION -- opens one armed period. One ring slot, written before
 *                        that period's first BLOCK.
 *     [1]      u8  session_tag
 *     [4..7]   u32 seq
 *     [8..11]  u32 session    counter, monotonic for the life of the file
 *
 *     Every ring frame ALSO carries the low 8 bits of its session at payload
 *     byte 1, because the ring can overwrite a session's SESSION frame while
 *     its blocks are still live -- those blocks would otherwise read as a
 *     continuation of the session before them. A reader splits sessions on
 *     that tag changing and uses the SESSION frame, when it survives, for the
 *     wall-clock anchor. Adjacent sessions cannot alias.
 *     [12..19] u64 unix_ms   disciplined wall clock when recording started
 *     [20..23] u32 cyc0      DWT stamp at that same moment, so a block's cycle
 *                            stamp maps to absolute time:
 *                              unix_ms + (t - cyc0)/clock_hz * 1000
 *     [24]     u8  synced    1 if the GCS had ever disciplined the clock; when
 *                            0, unix_ms is FC uptime, not epoch -- the
 *                            recording is still valid, you just cannot line it
 *                            up with anything else
 *     u8  reserved[3]
 *
 *   type 0x03 EVENT -- sparse annotation. One ring slot, emitted when a watched
 *                      value changes, so a reader can see WHY the numbers move.
 *     [1]      u8  session_tag
 *     [4..7]   u32 seq
 *     [8..11]  u32 t_cyc
 *     [12]     u8  kind      HSL_EV_*
 *     [16..19] u32 a         the new value
 *     [20..23] u32 b         the previous value
 *
 *     A whole sector for a handful of bytes, deliberately: events number in the
 *     tens per flight, keeping every ring slot the same size keeps the reader
 *     trivial, and the waste is ~0.02% of the ring.
 *
 * ---------------------------------------------------------------------------
 * ORDERING -- how a reader reconstructs a circular, crash-interrupted file
 * ---------------------------------------------------------------------------
 * Every ring frame carries `seq`, which increases strictly for the life of the
 * file and never resets. That single field is enough:
 *
 *   1. Read all ring slots. Keep those carrying the sentinel and a plausible
 *      frame; the rest are card content from before the clusters were used.
 *   2. Sort by seq. That is chronological order, whatever the ring rotation.
 *   3. A gap in seq means sectors were lost (overwritten after a crash, see
 *      the resume margin above). Report it; do not treat it as the end.
 *
 * A reader must NOT assume slot order is time order, and must NOT stop at the
 * first slot that fails to parse -- a ring has no "end", and after the first
 * wrap the oldest live data sits immediately after the newest.
 *
 * Streams defined today:
 *   id 1  "imu"  12 B  2 kHz. gx,gy,gz,ax,ay,az as i16, SENSOR-NATIVE counts
 *                      and sensor axes -- pre-filter, pre-bias, pre-axis-remap.
 *                      Pre-filter is the point: the whole reason to record
 *                      this is to see what the filters should be removing.
 *                      The body-frame axis map (-x, +y, -z) is in the scale
 *                      signs, and is irrelevant to a magnitude spectrum.
 *
 *   id 2  "act"  12 B  400 Hz (the ESC PWM rate; there is nothing above it).
 *                      m1..m4 and the throttle SETPOINT as u16 of full scale,
 *                      plus flags. Without this the whole recording is one FFT
 *                      smeared across every throttle setting -- prop frequency
 *                      tracks motor command, so this is what lets the spectrum
 *                      be binned by throttle and a notch centre be scheduled
 *                      against it. It is also the only way to tell a commanded
 *                      climb from the collective shift MIXER_AIRMODE_RP applies
 *                      under saturation, which is what the 2026-09-07 flyaway
 *                      turned out to be.
 *
 *   id 3  "vrt"  28 B  20 Hz. The vertical estimator's own view -- baro, AGL,
 *                      ToF, fused altitude, climb rate and the accel bias --
 *                      on the SAME timebase as the vibration that corrupts it.
 *                      accel_bias is the direct read-out of how badly the
 *                      accelerometer is being rectified by motor noise.
 *
 * Decoder: tools/telemetry/hslog.py (also carries the format self-tests).
 */

#include <stdbool.h>
#include <stdint.h>

/* Wire constants -- keep in step with tools/telemetry/hslog.py. */
#define HSL_MAGIC 0x314C5348u /* "HSL1" */
#define HSL_VERSION 1u
#define HSL_FILE_HDR_BYTES 32u

#define HSL_FRAME_HDR_BYTES 4u
#define HSL_RING_SENTINEL 0xA5u /* frame `flags` in a ring slot */

#define HSL_TYPE_PAD 0x00u
#define HSL_TYPE_FMT 0x01u
#define HSL_TYPE_BLOCK 0x02u
#define HSL_TYPE_EVENT 0x03u
#define HSL_TYPE_SESSION 0x04u

#define HSL_STREAM_IMU 1u
#define HSL_IMU_REC_BYTES 12u
#define HSL_STREAM_ACT 2u
#define HSL_ACT_REC_BYTES 12u /* 4x u16 motor, u16 throttle, u16 flags       */
#define HSL_STREAM_VRT 3u
#define HSL_VRT_REC_BYTES 28u /* 6x f32, u16 flags, u16 pad                  */

/* Emission rates. Decimation is on the CYCLE STAMP, not a call counter, so a
 * stream lands at its stated rate whatever rate its producer happens to run
 * at -- and keeps doing so if that producer is later re-paced. */
#define HSL_ACT_RATE_HZ 400u /* the ESC PWM rate; nothing above it is real   */
#define HSL_VRT_RATE_HZ 20u

/* "act" flag bits. */
#define HSL_ACT_F_ARMED 0x0001u
#define HSL_ACT_F_IN_AIR 0x0002u

/* "vrt" flag bits. */
#define HSL_VRT_F_TOF_VALID 0x0001u
#define HSL_VRT_F_ACCEL_UNHEALTHY 0x0002u
#define HSL_VRT_F_VALID 0x0004u
#define HSL_VRT_F_HOVER_MEASURED 0x0008u

/* EVENT kinds. `a` is the new value, `b` the previous one. */
#define HSL_EV_STATE 1u        /* sys_state_t                                */
#define HSL_EV_MODE 2u         /* flight_mode_t                              */
#define HSL_EV_ACCEL_HEALTH 3u /* vertical estimator's accel_unhealthy latch */
#define HSL_EV_NOTCH 4u        /* gyro notch enable                          */

/* FMT field descriptor: 8 B name + type + 3 reserved + f32 scale. */
#define HSL_FMT_FIELD_BYTES 16u
#define HSL_FTYPE_I16 1u
#define HSL_FTYPE_U16 2u
#define HSL_FTYPE_I32 3u
#define HSL_FTYPE_F32 4u

/* One frame is one SD sector, so a write never straddles a sector boundary and
 * never provokes a read-modify-write. 512 - 4 (frame) - 16 (block) = 492 for
 * records, which is exactly 41 x 12 -- no padding at all. */
#define HSL_SECTOR_BYTES 512u
#define HSL_BLOCK_HDR_BYTES 16u
#define HSL_BLOCK_PAYLOAD_BYTES                                                \
  (HSL_SECTOR_BYTES - HSL_FRAME_HDR_BYTES - HSL_BLOCK_HDR_BYTES)
#define HSL_RECS_PER_BLOCK (HSL_BLOCK_PAYLOAD_BYTES / HSL_IMU_REC_BYTES)

/* Each stream gets its OWN buffer ring, because each has a different producer
 * task and a single shared ring would have three writers -- which is not an
 * SPSC queue any more, and this module holds no locks. Per stream: producer
 * owns head, the FS task owns tail, usable = N-1.
 *
 * Depth is set by how long the stream can tolerate the FS task being busy.
 * IMU fills a sector every ~20 ms, so 4 buffers ~= 61 ms of SD stall. ACT
 * fills one every ~100 ms and VRT every ~850 ms, so 2 apiece is already far
 * more slack than IMU's 4. Each buffer is a flat 512 B of .bss and the F401 is
 * RAM-starved -- see the FS_LOG_QUEUE_CAP note in fs_owner.c before raising
 * any of these. */
#define HSL_IMU_BUFFERS 4u
#define HSL_ACT_BUFFERS 2u
#define HSL_VRT_BUFFERS 2u

/* How often the file header's head_slot/next_seq hint is rewritten, in ring
 * sectors. Data sectors are 512 B and sector-aligned, so f_write hands them
 * straight to the card -- they are durable the moment they are written and
 * need no sync. Only this hint does. 16 sectors is ~330 ms of resume
 * uncertainty for one extra sector write every ~330 ms. */
#define HSL_HDR_SYNC_FRAMES 16u

/* Seq gap left on resume so a resumed write can never collide with the seq of
 * a sector written after the last hint update but before a power cut. Must
 * exceed HSL_HDR_SYNC_FRAMES. */
#define HSL_SEQ_RESUME_MARGIN (HSL_HDR_SYNC_FRAMES * 2u)

/**
 * @brief Boot-time setup: commit the file's cluster chain and recover the ring
 *        cursor. Call from fs_owner_boot_init(), before the scheduler starts.
 */
void imu_hs_log_boot_init(void);

/**
 * @brief Publish the sensor's count->SI scale factors, which go into the FMT
 *        frame so a recording carries its own calibration.
 *
 * Pushed in by the driver rather than pulled from it: storage has no business
 * knowing which IMU is fitted, and the host tests link this module without any
 * driver at all. Defaults to 1.0 (i.e. raw counts) until set.
 */
void imu_hs_log_set_scale(float gyr_dps_per_count, float acc_mps2_per_count);

/**
 * @brief Append one IMU sample. Called from the IMU sample path at 2 kHz.
 *
 * Pure RAM: copies 12 bytes into the sector being filled and, when that sector
 * completes, publishes it to the FS task. Never blocks, never touches the
 * filesystem, and does nothing at all while a recording is not running.
 *
 * @param gyr    sensor-native raw gyro counts, PRE-filter
 * @param acc    sensor-native raw accel counts, PRE-filter
 * @param t_cyc  DWT cycle stamp of this sample
 */
void imu_hs_log_sample(const int16_t gyr[3], const int16_t acc[3],
                       uint32_t t_cyc);

/**
 * @brief Append one actuator sample (stream "act"). Called from the rate loop.
 *
 * Decimated internally to HSL_ACT_RATE_HZ, so the caller passes every sample
 * and does not care what rate it runs at.
 *
 * @param motors    per-motor command, 0..1
 * @param throttle  collective setpoint into the mixer, 0..1
 * @param flags     HSL_ACT_F_*
 * @param t_cyc     DWT cycle stamp of the driving IMU sample
 */
void imu_hs_log_act(const float motors[4], float throttle, uint16_t flags,
                    uint32_t t_cyc);

/** One "vrt" record. A plain struct rather than the estimator's own type:
 *  storage has no business including est/, and the host tests link this module
 *  without the estimator at all. */
typedef struct {
  float baro_altitude;
  float agl;
  float agl_tof;
  float altitude;
  float climb_rate;
  float accel_bias;
  uint16_t flags; /* HSL_VRT_F_* */
} hsl_vert_sample_t;

/**
 * @brief Append one vertical-estimator sample (stream "vrt"). Called from the
 *        VERT task; decimated internally to HSL_VRT_RATE_HZ.
 */
void imu_hs_log_vert(const hsl_vert_sample_t *v, uint32_t t_cyc);

/**
 * @brief Service the stream: start/stop with the arm state and write any
 *        completed sectors. Call once per fs_owner_task loop -- it is the FS
 *        task context that keeps SD/VFS single-owner.
 */
void imu_hs_log_drain(void);

/** @brief Sectors dropped because the ring was full (SD could not keep up). */
uint32_t imu_hs_log_dropped(void);

/** @brief Ring slot the next frame will be written to. */
uint32_t imu_hs_log_head_slot(void);

/** @brief Total ring slots. */
uint32_t imu_hs_log_ring_sectors(void);

/** @brief Current armed-period counter. */
uint32_t imu_hs_log_session(void);

/** @brief seq the next frame will carry; its rate of change is the sector rate. */
uint32_t imu_hs_log_seq(void);

/** @brief Times the ring has wrapped since the file was created. */
uint32_t imu_hs_log_wraps(void);

/** @brief True while a recording is open. */
bool imu_hs_log_active(void);

#endif /* VAYU_STORAGE_IMU_HS_LOG_H */
