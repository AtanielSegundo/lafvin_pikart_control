#!/usr/bin/env python3
"""
Standalone encoder counter — isolate quadrature counting in ONE process.

Purpose: test whether the real problem is the per-edge `pi.read()` socket calls
in the server's callback (it almost certainly is). This uses a FAST callback
that never touches the socket per edge — it uses the `level` argument pigpio
already provides — so it can service far higher edge rates without dropping.

It also counts edges PER PHASE (A and B separately), so a dead / intermittent
phase shows up immediately (one column stuck near 0 while the other climbs).

Run on the Pi with the daemon up:
    sudo pigpiod                 # if not already running
    python3 encoder_counter.py

Spin each wheel by hand, or drive the car, and watch the counts. Ctrl-C to stop.
"""
import time
import pigpio

# BCM pins per motor: (phase_a, phase_b) — keep in sync with Server/config.py
PINS = {
    "M1": (25, 5),
    "M2": (26, 20),
    "M3": (6, 12),
    "M4": (8, 7),
}

GLITCH_US = 0        # 0 = filter OFF (shaped Hall output tolerates it).
                     # Raise to 10–50 only if you see counts creep with the
                     # wheels still (electrical noise over-counting).
PRINT_HZ  = 5        # status lines per second

# x4 quadrature transition table, indexed by (prev_state << 2) | new_state,
# with state = (A << 1) | B.  Same table as encoders.py.
QUAD = [0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0]


class Enc:
    """Fast x4 decoder for one motor — NO socket reads inside the callback."""

    def __init__(self, pi, a, b, name=""):
        self.pi, self.a, self.b, self.name = pi, a, b, name
        self.count = 0
        self.a_edges = 0
        self.b_edges = 0

        for p in (a, b):
            pi.set_mode(p, pigpio.INPUT)
            pi.set_pull_up_down(p, pigpio.PUD_UP)
            pi.set_glitch_filter(p, GLITCH_US)   # 0 disables the filter

        # Seed levels ONCE (socket reads here are fine — not per edge).
        self.la = pi.read(a)
        self.lb = pi.read(b)
        self.state = (self.la << 1) | self.lb

        self.cba = pi.callback(a, pigpio.EITHER_EDGE, self._cb)
        self.cbb = pi.callback(b, pigpio.EITHER_EDGE, self._cb)

    def _cb(self, gpio, level, tick):
        # `level` is the NEW level of the pin that just changed (0 or 1).
        # 2 == watchdog timeout; ignore it. No pi.read() -> no socket round-trip.
        if level > 1:
            return
        if gpio == self.a:
            self.la = level
            self.a_edges += 1
        else:
            self.lb = level
            self.b_edges += 1
        new = (self.la << 1) | self.lb
        self.count += QUAD[(self.state << 2) | new]
        self.state = new

    def cancel(self):
        self.cba.cancel()
        self.cbb.cancel()


def main():
    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("pigpiod not reachable — start it:  sudo pigpiod")

    encs = {tag: Enc(pi, a, b, tag) for tag, (a, b) in PINS.items()}
    print(f"counting (glitch={GLITCH_US}us). pins={PINS}")
    print("spin each wheel / drive. Ctrl-C to stop.\n")
    print("format:  TAG=count (Δcount  Aedges/Bedges)\n")

    last = {tag: (0, 0, 0) for tag in encs}   # (count, a_edges, b_edges)
    t0 = time.time()
    try:
        period = 1.0 / PRINT_HZ
        while True:
            time.sleep(period)
            now = time.time(); dt = now - t0; t0 = now
            parts = []
            for tag, e in encs.items():
                c, ae, be = e.count, e.a_edges, e.b_edges
                dc = c - last[tag][0]
                dae = ae - last[tag][1]
                dbe = be - last[tag][2]
                last[tag] = (c, ae, be)
                flag = ""
                if dae + dbe > 0 and (dae == 0 or dbe == 0):
                    flag = " <-DEAD PHASE"   # only one phase moving
                parts.append(f"{tag}={c:>8d} ({dc:+5d}  A{dae:>3d}/B{dbe:>3d}){flag}")
            print("   ".join(parts))
    except KeyboardInterrupt:
        pass
    finally:
        for e in encs.values():
            e.cancel()
        print("\nfinal counts:", {t: e.count for t, e in encs.items()})
        print("total A/B edges:",
              {t: (e.a_edges, e.b_edges) for t, e in encs.items()})
        pi.stop()


if __name__ == "__main__":
    main()
