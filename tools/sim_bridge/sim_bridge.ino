/**
 * @file sim_bridge.ino
 * @brief FS-i6 PPM -> PC bridge for flight-simulator input.
 *
 * Reads a positive-shift PPM frame from an FS-iA6B receiver (FS-i6 set
 * to PPM output mode) on D2, decodes up to 8 channels via a single
 * RISING-edge interrupt, and forwards them to the PC over USB-serial
 * as CSV at ~50 Hz.
 *
 * Wiring (FS-iA6B `CH1` servo pin -> Arduino Nano):
 *   Signal -> D2     +V -> 5V     GND -> GND
 *
 * PC output (115200 8N1):
 *   # sim_bridge ready          (one-shot startup banner)
 *   1500,1500,1000,1500,2000    (one line per frame, us per channel)
 *   NO_SIGNAL                   (no valid frame for >100 ms)
 *
 * Channel pulse widths are in microseconds (typ. 1000-2000).
 *
 * Default FS-i6 channel order (Mode 2):
 *   CH1 aileron (roll), CH2 elevator (pitch), CH3 throttle,
 *   CH4 rudder (yaw),   CH5 SwA,              CH6 VrA/SwB/SwD.
 */

#include <Arduino.h>

/* ---- Configuration ---- */
static const uint8_t  PPM_PIN           = 2;
static const uint8_t  MAX_CHANNELS      = 8;
static const uint32_t SYNC_GAP_US       = 3000;    /* > this gap = frame end  */
static const uint32_t MIN_VALID_US      = 800;     /* narrowest channel pulse */
static const uint32_t MAX_VALID_US      = 2200;    /* widest channel pulse    */
static const uint32_t SIGNAL_TIMEOUT_US = 100000;  /* 100 ms = signal lost    */
static const uint32_t SEND_INTERVAL_MS  = 20;      /* 50 Hz output            */
static const uint32_t SERIAL_BAUD       = 115200;

/* ---- ISR-shared state ----
 * All access from loop() is wrapped in noInterrupts() / interrupts() to take
 * an atomic snapshot. The ISR itself runs with IRQs disabled, so updates
 * inside it are naturally atomic. */
static volatile uint16_t ppm_channels[MAX_CHANNELS];
static volatile uint8_t  ppm_channel_count = 0;  /* channels in last frame */
static volatile uint8_t  ppm_idx_building  = 0;  /* index while filling    */
static volatile uint32_t ppm_last_edge_us  = 0;
static volatile uint32_t ppm_last_frame_us = 0;
static volatile bool     ppm_have_frame    = false;

/* ---- ISR: one call per rising edge on D2 ---- */
static void ppm_isr() {
  uint32_t now = micros();
  uint32_t dt  = now - ppm_last_edge_us;
  ppm_last_edge_us = now;

  if (dt > SYNC_GAP_US) {
    /* Long gap closes the previous frame. */
    if (ppm_idx_building > 0) {
      ppm_channel_count = ppm_idx_building;
      ppm_have_frame    = true;
      ppm_last_frame_us = now;
    }
    ppm_idx_building = 0;
    return;
  }

  if (dt >= MIN_VALID_US && dt <= MAX_VALID_US &&
      ppm_idx_building < MAX_CHANNELS) {
    ppm_channels[ppm_idx_building++] = (uint16_t)dt;
  } else {
    /* Out-of-range gap mid-frame -> discard the partial frame. */
    ppm_idx_building = 0;
  }
}

void setup() {
  pinMode(PPM_PIN, INPUT);
  Serial.begin(SERIAL_BAUD);
  /* Brief settle: Nano resets when the host opens the port; harmless otherwise. */
  delay(50);
  Serial.println(F("# sim_bridge ready"));
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppm_isr, RISING);
}

void loop() {
  static uint32_t last_send_ms = 0;
  uint32_t now_ms = millis();
  if (now_ms - last_send_ms < SEND_INTERVAL_MS) return;
  last_send_ms = now_ms;

  /* Atomic snapshot of ISR state. */
  uint16_t snap[MAX_CHANNELS];
  uint8_t  count;
  uint32_t last_frame;
  bool     have;
  noInterrupts();
  count      = ppm_channel_count;
  last_frame = ppm_last_frame_us;
  have       = ppm_have_frame;
  for (uint8_t i = 0; i < count; i++) snap[i] = ppm_channels[i];
  interrupts();

  bool lost = (uint32_t)(micros() - last_frame) > SIGNAL_TIMEOUT_US;
  if (!have || lost) {
    Serial.println(F("NO_SIGNAL"));
    return;
  }

  for (uint8_t i = 0; i < count; i++) {
    if (i > 0) Serial.print(',');
    Serial.print(snap[i]);
  }
  Serial.println();
}
