#!/usr/bin/env python3
"""Decode a udp_telem_sniff --raw-bin VREC capture and characterize pitch
oscillation: dominant frequency (FFT), amplitude, throttle correlation, and a
roll-vs-pitch comparison to tell an axis-specific cascade from airframe shake.
"""
import os, sys, struct
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "navlink", "sim"))
sys.path.insert(0, os.path.join(ROOT, "navlink", "generated", "python"))
import frame                    # noqa: E402
import navlink_msgs as nl       # noqa: E402

CT = nl.ControlTrace.MSGID

def walk(path):
    """Yield (t_s, msgid, payload) for every coalesced frame in the VREC."""
    with open(path, "rb") as f:
        blob = f.read()
    magic, fmt, proto, startms = struct.unpack_from("<IIIQ", blob, 0)
    assert magic == 0x56524543, f"bad VREC magic {magic:#x}"
    off = 20
    while off + 12 <= len(blob):
        t_us, dlen = struct.unpack_from("<QI", blob, off)
        off += 12
        data = blob[off:off + dlen]; off += dlen
        t = t_us / 1e6
        p = 0
        while len(data) - p >= frame.HDR_LEN + 2:
            if data[p] != 0x56 or data[p + 1] != 0x02:
                p += 1; continue
            flen = frame.HDR_LEN + data[p + 2] + 2
            if len(data) - p < flen:
                break
            d = frame.decode(data[p:p + flen]); p += flen
            if d.ok:
                yield t, d.msgid, d.payload

def main():
    path = sys.argv[1]
    rows = []
    for t, mid, pl in walk(path):
        if mid == CT:
            c = nl.ControlTrace.unpack(pl)
            rows.append((t, c.pitch_rate_curr, c.pitch_rate_sp, c.pitch_out,
                         c.pitch_angle_curr, c.roll_rate_curr, c.roll_out,
                         c.thro_out, c.inner_dt))
    a = np.array(rows, float)
    if len(a) < 10:
        print("no ControlTrace data"); return
    t = a[:, 0] - a[0, 0]
    pr, prsp, pout, pang, rr, rout, thr, idt = a[:, 1], a[:, 2], a[:, 3], a[:, 4], a[:, 5], a[:, 6], a[:, 7], a[:, 8]
    dur = t[-1] - t[0]
    fs = len(t) / dur
    print(f"=== {os.path.basename(path)} ===")
    print(f"ControlTrace samples: {len(a)}   span {dur:.1f}s   ~{fs:.1f} Hz   inner_dt {np.median(idt)*1e3:.2f} ms")
    print(f"throttle thro_out: min {thr.min():.3f}  max {thr.max():.3f}  mean {thr.mean():.3f}")

    # Uniform resample for spectral work (arrival jitter -> even grid).
    n = int(dur * fs)
    tu = np.linspace(t[0], t[-1], n)
    def spec(x, lbl):
        xu = np.interp(tu, t, x)
        xu = xu - xu.mean()
        win = np.hanning(len(xu))
        F = np.fft.rfft(xu * win)
        fr_ = np.fft.rfftfreq(len(xu), 1.0 / fs)
        mag = np.abs(F)
        band = fr_ > 0.5            # ignore DC/very-low drift
        k = np.argmax(mag * band)
        return fr_[k], mag, fr_
    fpk, magp, frq = spec(pr, "pitch")
    frpk, magr, _ = spec(rr, "roll")

    def stats(x):
        return x.std(), np.percentile(x, 97.5) - np.percentile(x, 2.5), np.abs(x).max()
    prs, prpp, prmax = stats(pr)
    rrs, rrpp, rrmax = stats(rr)
    print()
    print(f"PITCH rate: RMS {prs:6.1f}  p2p(95%) {prpp:7.1f}  |max| {prmax:7.1f} deg/s   dominant {fpk:.2f} Hz")
    print(f"ROLL  rate: RMS {rrs:6.1f}  p2p(95%) {rrpp:7.1f}  |max| {rrmax:7.1f} deg/s   dominant {frpk:.2f} Hz")
    print(f"PITCH out u: RMS {pout.std():.3f}  |max| {np.abs(pout).max():.3f}  (sat |u|>=0.99: {100*np.mean(np.abs(pout)>=0.99):.1f}% of samples)")
    print(f"ROLL  out u: RMS {rout.std():.3f}  |max| {np.abs(rout).max():.3f}")
    print(f"pitch ang range: {pang.min():.1f} .. {pang.max():.1f} deg")

    # Top spectral peaks for pitch.
    order = np.argsort(magp)[::-1]
    seen = []
    print("\npitch-rate spectral peaks:")
    for k in order:
        f0 = frq[k]
        if f0 < 0.5: continue
        if any(abs(f0 - s) < 0.6 for s in seen): continue
        seen.append(f0)
        print(f"   {f0:5.2f} Hz   rel power {magp[k]/magp[order[0]]:.2f}")
        if len(seen) >= 5: break

    # Throttle correlation: split the window by throttle and report pitch RMS.
    print("\npitch-rate RMS vs throttle band:")
    for lo, hi in [(0.0, 0.1), (0.1, 0.3), (0.3, 0.5), (0.5, 1.01)]:
        m = (thr >= lo) & (thr < hi)
        if m.sum() > 5:
            print(f"   thr [{lo:.1f},{hi:.1f}): n={m.sum():5d}  pitch RMS {pr[m].std():6.1f}  roll RMS {rr[m].std():6.1f} deg/s")

    # Worst 1-second window by pitch-rate RMS.
    bw = int(fs)
    best, bi = -1, 0
    for i in range(0, len(pr) - bw, max(1, bw // 4)):
        s = pr[i:i+bw].std()
        if s > best: best, bi = s, i
    print(f"\nworst 1s window @ t+{t[bi]:.1f}s: pitch RMS {best:.1f} deg/s, "
          f"thr {thr[bi:bi+bw].mean():.2f}, pitch ang {pang[bi:bi+bw].min():.0f}..{pang[bi:bi+bw].max():.0f} deg")

if __name__ == "__main__":
    main()
