"""Egress link-impairment model: a UDP sender that delays, drops, duplicates,
reorders, corrupts and rate-limits datagrams to emulate a real radio/network
link. Each endpoint owns one Link to its peer, so the uplink and downlink can be
impaired independently.

Impairments are applied at send time; a background worker releases datagrams at
their scheduled times (jitter naturally produces reordering; an explicit reorder
knob adds occasional large delays). Decisions are drawn from a seeded RNG so a
run is broadly reproducible (exact timing still depends on the scheduler)."""
import heapq
import random
import threading
import time
from dataclasses import asdict, dataclass


@dataclass
class LinkConfig:
    latency_ms: float = 0.0     # one-way base latency
    jitter_ms: float = 0.0      # uniform [0, jitter] added to each datagram
    loss: float = 0.0           # P(drop)
    dup: float = 0.0            # P(send a duplicate copy)
    reorder: float = 0.0        # P(add a large extra delay → reorder)
    corrupt: float = 0.0        # P(flip one random bit; CRC should catch it)
    rate_bytes: float = 0.0     # egress bandwidth cap (B/s); 0 = unlimited
    seed: int = 0

    @classmethod
    def from_dict(cls, d):
        return cls(**{k: d[k] for k in d if k in cls.__dataclass_fields__})


class Link:
    def __init__(self, sock, dest, cfg, direction=0):
        self.sock = sock
        self.dest = dest
        self.cfg = cfg
        self.rng = random.Random((cfg.seed << 1) ^ direction)
        self._heap = []
        self._ctr = 0
        self._serial_free = 0.0
        self._cv = threading.Condition()
        self._running = True
        # counters
        self.sent = self.dropped = self.duped = self.corrupted = 0
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    def send(self, data):
        c = self.cfg
        if c.loss and self.rng.random() < c.loss:
            self.dropped += 1
            return
        copies = 2 if (c.dup and self.rng.random() < c.dup) else 1
        if copies == 2:
            self.duped += 1
        for _ in range(copies):
            buf = bytearray(data)
            if c.corrupt and self.rng.random() < c.corrupt:
                buf[self.rng.randrange(len(buf))] ^= 1 << self.rng.randrange(8)
                self.corrupted += 1
            delay = c.latency_ms / 1000.0
            if c.jitter_ms:
                delay += self.rng.uniform(0.0, c.jitter_ms / 1000.0)
            if c.reorder and self.rng.random() < c.reorder:
                delay += self.rng.uniform(0.0, max(c.latency_ms, 20.0) / 1000.0 * 3.0)
            now = time.monotonic()
            release = now + delay
            if c.rate_bytes > 0:                       # serialization / bandwidth cap
                self._serial_free = max(self._serial_free, now) + len(buf) / c.rate_bytes
                release = max(release, self._serial_free)
            with self._cv:
                self._ctr += 1
                heapq.heappush(self._heap, (release, self._ctr, bytes(buf)))
                self._cv.notify()
            self.sent += 1

    def _run(self):
        while True:
            with self._cv:
                while self._running and not self._heap:
                    self._cv.wait()
                if not self._running and not self._heap:
                    return
                release, _, buf = self._heap[0]
                now = time.monotonic()
                if release > now:
                    self._cv.wait(timeout=release - now)
                    continue
                heapq.heappop(self._heap)
            try:
                self.sock.sendto(buf, self.dest)
            except OSError:
                pass

    def drain(self, timeout=2.0):
        """Block until queued datagrams have been released (best effort)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self._cv:
                if not self._heap:
                    return
            time.sleep(0.01)

    def close(self):
        with self._cv:
            self._running = False
            self._cv.notify_all()
        self._worker.join(timeout=2.0)

    def stats(self):
        return {"sent": self.sent, "dropped": self.dropped,
                "duped": self.duped, "corrupted": self.corrupted,
                "cfg": asdict(self.cfg)}
