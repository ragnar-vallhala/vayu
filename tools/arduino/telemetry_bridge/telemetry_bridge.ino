/*
 * telemetry_bridge.ino — drone-side telemetry/command bridge (ESP8266 NodeMCU).
 *
 * FC USART6 <-> GCS over WiFi/UDP, both directions:
 *   FC telemetry -> ESP -> UDP -> GCS
 *   GCS commands -> UDP -> ESP -> FC
 *
 * FRAME-AWARE RELAY (the important part):
 *   The FC->ESP UART link is clean, so we parse the telemetry framing HERE and
 *   only ever put WHOLE frames into a UDP packet — never a frame split across
 *   two packets. That way a dropped UDP packet loses whole frames cleanly and
 *   the next packet still starts on a frame boundary, so the GCS parser never
 *   desyncs. (Splitting frames across packets was turning every lost packet
 *   into a burst of RAW/UNK garbage at the GCS.)
 *
 *   Frame wire format (matches the GCS PacketDecoder):
 *     sync(0x56) typever(1) length(1) devid(1) ts(4) payload(length) crc32(4)
 *     -> total = 8 + length + 4.  version is the low nibble of typever, == 1.
 *   We don't CRC here (the GCS does); we only need boundaries, and length comes
 *   straight off the clean UART, so framing stays aligned frame-to-frame.
 *
 *   Two buffers, by role:
 *     acc[]  writer side — raw UART bytes accumulate here, frames are sliced out
 *     out[]  reader side — whole frames are packed here and sent as one datagram
 *   Pack frames until the next won't fit (<=512 B), or flush on a 2 ms timer
 *   tick (hardware timer1 ISR -> flag) so latency stays bounded.
 *
 * FLASHABLE WITH THE FC WIRED IN (default UART0 pins GPIO1/3 stay on USB):
 *   FC PC6 (USART6 TX) -> ESP GPIO13 / D7   UART0 RX after Serial.swap()
 *   FC PC7 (USART6 RX) <- ESP GPIO2  / D4   UART1 TX (Serial1) — commands out
 *   FC GND             -> ESP GND
 *   ESP GPIO15 / D8    -> leave UNCONNECTED (boot strap)
 *
 * WIFI_NONE_SLEEP avoids modem-sleep UDP loss; unicast (once the GCS announces
 * itself) gets MAC-layer retries. Library: WiFiManager (tzapu). Core: esp8266.
 * Baud must match the FC's UART_BAUDRATE.
 */
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <WiFiManager.h>

#define FC_BAUD     230400
#define UDP_PORT    14555
#define CFG_PORTAL  "vayu-config"
#define MAX_UDP     512
// timer1 @ 5 MHz (TIM_DIV16) -> 0.2 us/tick; 2 ms = 10000 ticks.
#define FLUSH_TICKS 10000

WiFiUDP udp;
WiFiManager wm;
IPAddress gcsIp(0, 0, 0, 0);
bool haveGcs = false;
bool wifiUp = false;

uint8_t acc[2048];  // writer side: raw UART bytes awaiting framing
int accLen = 0;
uint8_t out[MAX_UDP]; // reader side: whole frames packed for one datagram
int outLen = 0;
volatile bool flushDue = false; // set by the timer ISR, cleared in loop()

uint8_t cmd[256];   // GCS -> FC command scratch
uint32_t lastFrame = 0, lastBeat = 0;

void IRAM_ATTR onFlushTick() { flushDue = true; }

static void udpTo(const uint8_t *p, int n) {
  IPAddress dst = haveGcs ? gcsIp : IPAddress(255, 255, 255, 255);
  udp.beginPacket(dst, UDP_PORT);
  udp.write(p, n);
  udp.endPacket();
}

static void flushOut() {
  if (outLen <= 0)
    return;
  udpTo(out, outLen);
  outLen = 0;
}

void setup() {
  // UART0: FC telemetry IN, swapped to GPIO13/15 so GPIO1/3 stay free for USB
  // flashing. Big RX ring so a WiFi TX stall can't overflow it mid-frame.
  Serial.setRxBufferSize(2048);
  Serial.begin(FC_BAUD);
  Serial.swap();
  // UART1: commands OUT to the FC on GPIO2 (TX-only, safe boot strap).
  Serial1.begin(FC_BAUD);

  WiFi.persistent(false);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setOutputPower(20.5f);

  wm.setDebugOutput(false);
  wm.setConfigPortalBlocking(false);
  wm.setConfigPortalTimeout(0);
  wm.autoConnect(CFG_PORTAL);

  udp.begin(UDP_PORT);
  wifiUp = (WiFi.status() == WL_CONNECTED);

  timer1_attachInterrupt(onFlushTick);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  timer1_write(FLUSH_TICKS);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    wifiUp = false;
    wm.process();
    return;
  }
  if (!wifiUp) {
    wifiUp = true;
    udp.begin(UDP_PORT); // re-bind after a reconnect
  }

  // GCS -> FC: forward command frames (sync 0x56) out UART1; learn the GCS IP.
  if (udp.parsePacket() > 0) {
    gcsIp = udp.remoteIP();
    haveGcs = true;
    int rn = udp.read(cmd, sizeof(cmd));
    if (rn > 0 && cmd[0] == 0x56)
      Serial1.write(cmd, rn);
  }

  // FC -> acc: pull whatever the ISR-filled UART ring holds.
  int avail = Serial.available();
  if (avail > 0) {
    int room = (int)sizeof(acc) - accLen;
    if (avail > room)
      avail = room;
    accLen += Serial.readBytes(acc + accLen, avail);
  }

  // Slice whole frames out of acc and pack them into out (frame-aligned UDP).
  int i = 0;
  while (i < accLen) {
    if (acc[i] != 0x56) { // not a frame start — skip junk
      i++;
      continue;
    }
    if (accLen - i < 8)
      break; // need the full 8-byte header to read length
    // Accept v1 (typever low nibble == 1) AND NavLink v2 (byte1 == 0x02). Both
    // carry payload_len at [i+2] and exactly 12 B of framing overhead (v1: 8 hdr
    // + 4 CRC32; v2: 10 hdr + 2 CRC16), so `total` below is identical for both —
    // we only must not reject a v2 frame as a false sync. (INTEGRATION.md)
    if ((acc[i + 1] & 0x0F) != 0x01 && acc[i + 1] != 0x02) { // false sync
      i++;
      continue;
    }
    int total = 8 + acc[i + 2] + 4; // header + payload + crc (v1 8+4 == v2 10+2)
    if (accLen - i < total)
      break; // rest of this frame hasn't arrived yet
    if (outLen + total > MAX_UDP)
      flushOut(); // current datagram is full — send it, start a new one
    memcpy(out + outLen, acc + i, total);
    outLen += total;
    i += total;
    lastFrame = millis();
  }
  if (i > 0) { // drop consumed/junk bytes, keep any partial trailing frame
    memmove(acc, acc + i, accLen - i);
    accLen -= i;
  }

  // Flush the packed frames on the 2 ms tick (bounded latency) or when full.
  if (flushDue) {
    flushDue = false;
    flushOut();
  } else if (outLen >= MAX_UDP) {
    flushOut();
  }

  // Idle heartbeat (no frames for 2 s) so a bare link is testable with no FC.
  uint32_t now = millis();
  if (now - lastFrame > 2000 && now - lastBeat > 1000) {
    lastBeat = now;
    static const char hb[] = "BRIDGE-HB";
    udpTo(reinterpret_cast<const uint8_t *>(hb), sizeof(hb) - 1);
  }
}
