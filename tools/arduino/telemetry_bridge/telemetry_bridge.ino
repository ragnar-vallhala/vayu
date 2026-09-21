# Copyright (C) 2026 NAVRobotec Pvt Ltd
# Author: Ragnar Vallhala
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
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

#define FC_BAUD     460800  // must match the FC's UART_BAUDRATE (vaios_app_config.h)
#define UDP_PORT    14555
#define CFG_PORTAL  "vayu-config"
// Datagram cap. Coalescing more whole frames per datagram is what raises the
// effective frame throughput: the binding limit is the ESP's sustainable UDP
// *datagrams/s* (per-packet WiFi airtime + unicast MAC retries), NOT bytes/s and
// NOT RAM. 1472 = 1500 MTU - 20 IP - 8 UDP: the largest payload that still rides
// in a SINGLE WiFi frame. Do NOT raise past this "because we have RAM" — above
// MTU lwIP fragments into multiple IP fragments, each still its own over-the-air
// TX (no airtime saved) and losing any one fragment drops the WHOLE datagram
// (UDP has no retransmit). One datagram = one <=MTU WiFi frame keeps loss atomic.
#define MAX_UDP     1472
// Flush cadence = a latency-for-throughput knob. Frames accrue only at the UART
// rate (~46 B ~= 1 frame per 2 ms @ 230400), so a longer interval packs more
// frames per datagram -> fewer datagrams/s -> further under the ESP ceiling, at
// the cost of that much added latency. 8 ms (~4-5 frames/datagram, ~100 dg/s)
// is a good monitoring default; drop to 2 ms (10000 ticks) if you need
// low-latency stick-feel data. timer1 @ 5 MHz (TIM_DIV16) -> 0.2 us/tick.
#define FLUSH_MS    8
#define FLUSH_TICKS (FLUSH_MS * 5000)  // 5000 ticks/ms at TIM_DIV16

WiFiUDP udp;
WiFiManager wm;
IPAddress gcsIp(0, 0, 0, 0);
bool haveGcs = false;
bool wifiUp = false;

uint8_t acc[4096];  // writer side: raw UART bytes awaiting framing. 4 KiB (was 2 KiB)
                    // to absorb a WiFi-TX stall at the higher baud without overflow:
                    // at 460800 (~46 B/ms) 4 KiB buffers ~89 ms of stall (cf. ~22 ms for
                    // 2 KiB at the old 230400). This + the matching RX ring below is what
                    // lets us raise baud past the old ESP cap. firmware/docs/plans/link-bandwidth-boost.md
int accLen = 0;
uint8_t out[MAX_UDP]; // reader side: whole frames packed for one datagram
int outLen = 0;
volatile bool flushDue = false; // set by the timer ISR, cleared in loop()

uint8_t cmd[512];   // GCS -> FC command scratch. MUST be >= NAVLINK_MAX_FRAME
                    // (267 B = 10 hdr + 255 payload + 2 CRC): a full XFER_DATA
                    // upload chunk is a 266 B frame, and udp.read() silently
                    // truncates to sizeof(cmd) — a 256 B buffer dropped the CRC
                    // tail of large uplink frames, wedging file uploads.
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
  // flashing. Big RX ring so a WiFi TX stall can't overflow it mid-frame. 4 KiB
  // (was 2 KiB) to survive the higher FC_BAUD — see the acc[] note above.
  Serial.setRxBufferSize(4096);
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
    // Adaptive flush, fast path: if the next whole frame won't fit the MTU-sized
    // datagram, send what we have now and start a new one. A single frame never
    // exceeds MAX_UDP (max NavLink frame = 255 payload + 12 = 267 B << 1472), so
    // an empty `out` always has room for one frame. Bursts (e.g. the ~1336 B perf
    // report) thus go out the instant they fill a datagram, without waiting for
    // the timer — keeping latency low while still coalescing.
    if (outLen + total > MAX_UDP)
      flushOut();
    memcpy(out + outLen, acc + i, total);
    outLen += total;
    i += total;
    lastFrame = millis();
  }
  if (i > 0) { // drop consumed/junk bytes, keep any partial trailing frame
    memmove(acc, acc + i, accLen - i);
    accLen -= i;
  }

  // Adaptive flush, slow path: send whatever frames have accumulated on the
  // FLUSH_MS timer tick, bounding latency when traffic is too light to fill a
  // datagram. (The fill path above already handles the bursty case.)
  if (flushDue) {
    flushDue = false;
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
