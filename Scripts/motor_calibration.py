#!/usr/bin/python3
"""
Per-motor PWM calibration for a CENTRED in-place turn.

The gyro turn (DriveController heading branch) sends one duty magnitude to all
four motors, but the motors do not answer it with one speed. This rig measures
each motor's speed-vs-duty line and writes the coefficients that
Server/motor_correction.py:correct_pwm uses to map the reference duty (m1,
forward) onto every other motor and direction:

    w(u) = k*u + c                 per slot m1..m4, per direction fwd/rev
    u_target = a*u_ref + b         a = k_ref/k_t,  b = (c_ref - c_t)/k_t

SET-UP: the kart SUSPENDED, all four wheels free. No floor, no slip, no load --
the encoders then measure the motor and nothing else.

PROCEDURE
---------
For every duty level, both directions, and every slot, ONE motor is driven
alone (the others brake at 0) and the wheel speed is the least-squares slope of
its encoder total against the control loop's timestamp, over a window that
starts after spin-up. Driving one motor at a time is deliberate: with all four
free-spinning, the summed edge rate passes the encoder acquisition ceiling
(see Scripts/gray_regression.py, ENC_MAX_CPS) and the counts come back short.
The slots are interleaved per level so every motor at a given duty is measured
within seconds of m1, on the same battery charge.

Each point also records WHICH encoder answered, so the slot -> encoder mapping
(CMD_MOTOR position vs. M1..M4 in config.py) is measured, not assumed, and a
quadrature encoder counting against its configured sign is flagged.

Then a verification pass drives each slot at the raw and at the corrected duty
for a few levels inside the calibrated range and reports the left/right speed
imbalance of a ccw and a cw turn, before and after.

ENCODER SPEED LIMIT
-------------------
Past a certain WHEEL speed the encoders stop counting: the reading flattens,
then falls as the duty rises, then freezes -- while the wheel spins faster. A
free wheel on the 12 V pack gets there around duty 1500. The first run hit it
at ~5.5 rev/s on every encoder, the single-phase M3 included, although M3
makes half the callbacks per revolution: the limit follows the per-phase pulse
width (~140 us there), which points at encoders.GLITCH_FILTER_US (100 us
then, 20 us since) rather than the Python callback rate. With the filter at
20 us the callback rate is the limit: ~20k callbacks/s, ~8 rev/s for a
full-quadrature encoder driven alone.

So each motor/direction is swept upward only until its encoder gives out
(above --max-rps, or slower than at the previous duty); its higher levels are
skipped. The lines are fitted over the duty range EVERY motor was measured in
-- the top --fit-span of it, the part closest to the heading duties -- and the
report says when the heading runs above that range, i.e. when correct_pwm is
extrapolating. Raise --max-rps only after raising the encoder limit itself.

Transport (no Pi libraries needed, laptop on the kart's network):
  * motor duty  -> HTTP POST /command   CMD_MOTOR#m1#m2#m3#m4   (mode 'one')
  * telemetry   -> WebSocket /ws        drive.encoders, drive.control_ts

USAGE
-----
    python Scripts/motor_calibration.py --ip 192.168.0.237
    python Scripts/motor_calibration.py --ip ... --levels 500:2500:125 --repeats 2
    python Scripts/motor_calibration.py --refit Scripts/data/motor_cal_<ts>.csv

Writes Server/motor_calibration.json (copy it to Server/ on the Pi and restart
the server -- P_control loads it at start-up) and the raw points to
Scripts/data/motor_cal_<ts>.csv, which --refit re-fits offline (e.g. with a
different --fit-min/--fit-max/--max-rps) without spinning anything.

CAVEAT: the lines are measured unloaded. On the floor every motor also carries
the scrub load of the turn; what this removes is the motor-to-motor spread,
which is the part that decentres the rotation.
"""
from __future__ import annotations

import argparse
import base64
import csv
import json
import math
import os
import socket
import statistics
import struct
import sys
import threading
import time
from collections import Counter, deque
from dataclasses import asdict, dataclass, fields
from datetime import datetime
from typing import Dict, List, Optional

import requests as rq

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SERVER = os.path.join(REPO, "Server")
# Server/ is not a package -- see the same note in gray_regression.py.
if SERVER not in sys.path:
    sys.path.insert(0, SERVER)

from config import CONFIG, ENCODER_TAGS, SINGLE_PHASE_ENCODERS  # noqa: E402
from motor_correction import (CALIBRATION_PATH, SLOTS,  # noqa: E402
                              coefficients_from_curves, correct_pwm,
                              direction_of, validate)

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
DEFAULT_IP     = "192.168.0.237"
DEFAULT_PORT   = 8080
DEFAULT_LEVELS = "750:2500:250"
SPINUP_S       = 0.7      # tau of a free wheel is ~0.1-0.2 s; 0.7 s is settled
WINDOW_S       = 2.0      # rate-fit window: ~20 control snapshots at 10 Hz
REST_S         = 0.4      # brake between points so each starts from rest
FIT_SPAN       = 750      # fit the top 750 duty of the range all motors share
REFERENCE      = ("m1", "fwd")
MOVING_RPS     = 0.05     # wheel rev/s below which the motor counts as stopped
CROSSTALK_FRAC = 0.05     # another encoder moving >5% of the driven one: wiring?
# Highest wheel speed the encoders are trusted at (see ENCODER SPEED LIMIT).
# They gave out at ~5.5 rev/s with GLITCH_FILTER_US = 100 (use --max-rps 4.5
# to refit those runs); at 20 us the callback rate caps a full-quadrature
# encoder near 8 rev/s, so 7 keeps a margin. DROP_FRAC catches it if not.
MAX_RPS        = 7.0
# A reading this much SLOWER than at the previous (lower) duty means the
# encoder lost counts; level-to-level steps are 10%+ and the noise ~1%.
DROP_FRAC      = 0.03
MIN_SAMPLES    = 5
DATA_DIR       = os.path.join(HERE, "data")

LEFT_SLOTS, RIGHT_SLOTS = ("m1", "m2"), ("m3", "m4")   # Motor.setMotorModel


def counts_per_rev(tag: str) -> float:
    """Raw counts per wheel revolution: config's figure is post-scale (x4 base),
    a single-phase encoder's raw count is 1/scale of that."""
    scale = float(SINGLE_PHASE_ENCODERS.get(tag, {}).get("scale", 1.0))
    return CONFIG.wheel.motor_counts_per_rev[tag] / scale


# ---------------------------------------------------------------------------
# Motor commands  (HTTP POST /command)
# ---------------------------------------------------------------------------
class Kart:
    def __init__(self, ip: str, port: int):
        self.url = f"http://{ip}:{port}/command"
        self.http = rq.Session()

    def motors(self, d: List[int], retries: int = 3) -> bool:
        msg = "CMD_MOTOR#" + "#".join(str(int(x)) for x in d)
        for _ in range(retries):
            try:
                if self.http.post(self.url, data=msg, timeout=1.0).ok:
                    return True
            except rq.RequestException:
                continue
        return False

    def stop(self) -> None:
        if not self.motors([0, 0, 0, 0], retries=5):
            print("[ERROR] FAILED TO STOP THE MOTORS -- CHECK THE KART BY HAND")


# ---------------------------------------------------------------------------
# Telemetry  (persistent read-only WebSocket /ws, in a thread)
# ---------------------------------------------------------------------------
@dataclass
class Sample:
    t_local: float            # laptop monotonic, at receipt
    t_pi: float               # when the snapshot was taken on the Pi
    precise: bool             # t_pi is control_ts (vs. the broadcast ts)
    stale: bool
    mode: str
    battery: Optional[float]
    counts: Dict[str, int]


class _WsConn:
    """Minimal RFC 6455 client: enough to read text frames and answer pings."""

    def __init__(self, ip: str, port: int, path: str, timeout: float):
        self.sock = socket.create_connection((ip, port), timeout=timeout)
        self._buf = bytearray()
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((
            f"GET {path} HTTP/1.1\r\nHost: {ip}:{port}\r\n"
            "Upgrade: websocket\r\nConnection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
        ).encode())
        while b"\r\n\r\n" not in self._buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("closed during handshake")
            self._buf += chunk
        head, _, rest = bytes(self._buf).partition(b"\r\n\r\n")
        self._buf = bytearray(rest)        # may already hold the first frame
        if b" 101" not in head.split(b"\r\n")[0]:
            raise ConnectionError(f"handshake refused: {head[:80]!r}")

    def _recvn(self, n: int) -> bytes:
        while len(self._buf) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("socket closed")
            self._buf += chunk
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def read_frame(self):
        b1, b2 = self._recvn(2)
        length = b2 & 0x7F
        if length == 126:
            length = struct.unpack(">H", self._recvn(2))[0]
        elif length == 127:
            length = struct.unpack(">Q", self._recvn(8))[0]
        mask = self._recvn(4) if b2 & 0x80 else None
        payload = self._recvn(length) if length else b""
        if mask:
            payload = bytes(c ^ mask[i % 4] for i, c in enumerate(payload))
        return bool(b1 & 0x80), b1 & 0x0F, payload

    def send_frame(self, opcode: int, payload: bytes = b"") -> None:
        # Client frames MUST be masked (RFC 6455 5.3). Only pongs go out here,
        # and a control frame payload is <= 125 bytes.
        mask = os.urandom(4)
        body = bytes(c ^ mask[i % 4] for i, c in enumerate(payload[:125]))
        self.sock.sendall(bytes([0x80 | opcode, 0x80 | len(body)]) + mask + body)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class TelemetryStream:
    """Keeps the last few hundred telemetry samples, reconnecting on failure.

    Persistent rather than one socket per point: the server's WebSocket has a
    30 s heartbeat, so a long-lived client must answer pings -- done here.
    """

    def __init__(self, ip: str, port: int, path: str = "/ws"):
        self.ip, self.port, self.path = ip, port, path
        self._samples: deque = deque(maxlen=600)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._conn: Optional[_WsConn] = None
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="telemetry")

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._conn is not None:
            self._conn.close()

    def latest(self) -> Optional[Sample]:
        with self._lock:
            return self._samples[-1] if self._samples else None

    def between(self, t0: float, t1: float) -> List[Sample]:
        with self._lock:
            return [s for s in self._samples if t0 <= s.t_local <= t1]

    def wait_first(self, timeout: float) -> Optional[Sample]:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            s = self.latest()
            if s is not None:
                return s
            time.sleep(0.05)
        return None

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._conn = _WsConn(self.ip, self.port, self.path, timeout=5.0)
                self._read_loop(self._conn)
            except (OSError, ConnectionError) as e:
                if not self._stop.is_set():
                    print(f"[telemetry] {e}; reconnecting...")
            finally:
                if self._conn is not None:
                    self._conn.close()
            self._stop.wait(0.5)

    def _read_loop(self, conn: _WsConn) -> None:
        text = b""
        while not self._stop.is_set():
            fin, opcode, payload = conn.read_frame()
            if opcode == 0x9:                       # ping -> pong
                conn.send_frame(0xA, payload)
            elif opcode == 0x8:                     # close
                return
            elif opcode in (0x1, 0x0):              # text / continuation
                text += payload
                if fin:
                    self._on_text(text)
                    text = b""

    def _on_text(self, data: bytes) -> None:
        try:
            msg = json.loads(data.decode())
        except ValueError:
            return
        if msg.get("type") != "telemetry":
            return
        drive = msg.get("drive") or {}
        enc = drive.get("encoders") or {}
        if not all(t in enc for t in ENCODER_TAGS):
            return
        cts = drive.get("control_ts")
        sample = Sample(
            t_local=time.monotonic(),
            t_pi=float(cts) if cts else float(msg.get("ts", 0.0)),
            precise=bool(cts),
            stale=bool(drive.get("stale", False)),
            mode=str(msg.get("mode", "")),
            battery=msg.get("battery"),
            counts={t: int(enc[t]) for t in ENCODER_TAGS})
        with self._lock:
            self._samples.append(sample)


# ---------------------------------------------------------------------------
# Rate estimation
# ---------------------------------------------------------------------------
def _linfit(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0.0:
        return None
    k = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return k, my - k * mx


def count_rates(samples: List[Sample]):
    """({tag: raw counts/s}, n) from the snapshots in ``samples``.

    The broadcaster (10 Hz) and the control loop (20 Hz) are not synchronised,
    so one snapshot can be sent twice; with control_ts the repeat carries the
    same stamp and is dropped. Without it a repeat cannot be told from an
    encoder that stopped counting, so every message is kept: a repeat only
    adds timing noise, while dropping unchanged counts would hide exactly the
    frozen encoder this rig has to detect.
    """
    uniq: List[Sample] = []
    for s in samples:
        if uniq and s.precise and s.t_pi == uniq[-1].t_pi:
            continue
        uniq.append(s)
    if len(uniq) < MIN_SAMPLES:
        return None, len(uniq)
    t0 = uniq[0].t_pi
    ts = [s.t_pi - t0 for s in uniq]
    rates = {}
    for tag in ENCODER_TAGS:
        fit = _linfit(ts, [s.counts[tag] for s in uniq])
        if fit is None:
            return None, len(uniq)
        rates[tag] = fit[0]
    return rates, len(uniq)


# ---------------------------------------------------------------------------
# One measurement point
# ---------------------------------------------------------------------------
@dataclass
class Point:
    phase: str                # sweep | verify_raw | verify_cor
    repeat: int
    slot: str                 # m1..m4 (CMD_MOTOR position)
    duty: int                 # signed duty actually commanded
    ref_duty: int             # signed reference duty it stands for
    tag: str                  # encoder that answered ("" = wheel did not move)
    rps: float                # wheel rev/s, >= 0
    sign_ok: str              # "1" ok, "0" counted against config sign, "" n/a
    crosstalk: float          # fastest OTHER encoder / driven one
    n: int                    # unique telemetry snapshots in the window
    battery: float
    note: str = ""            # why the point is excluded from the fit, if it is


def measure(kart: Kart, stream: TelemetryStream, slot: str, duty: int,
            args, phase: str = "sweep", repeat: int = 0,
            ref_duty: Optional[int] = None) -> Optional[Point]:
    cmd = [0, 0, 0, 0]
    cmd[SLOTS.index(slot)] = int(duty)
    try:
        if not kart.motors(cmd):
            print(f"  [ERROR] {slot} {duty:+d}: kart not reachable, point skipped")
            return None
        time.sleep(args.spinup)
        t0 = time.monotonic()
        time.sleep(args.window)
        t1 = time.monotonic()
    finally:
        kart.stop()
    time.sleep(args.rest)

    window = stream.between(t0, t1)
    fresh = [s for s in window if not s.stale]
    rates, n = count_rates(fresh)
    if rates is None:
        why = "control loop not reporting (stale)" if window and not fresh \
            else "telemetry not arriving"
        print(f"  [warn] {slot} {duty:+5d}: {why} (n={n}), point skipped")
        return None

    rps = {t: abs(r) / counts_per_rev(t) for t, r in rates.items()}
    tag = max(rps, key=rps.get)
    moving = rps[tag] >= MOVING_RPS
    others = max((v for t, v in rps.items() if t != tag), default=0.0)
    crosstalk = others / rps[tag] if moving else 0.0

    sign_ok = ""
    if moving and tag not in SINGLE_PHASE_ENCODERS:   # single phase: unsigned
        expected = math.copysign(1.0, duty) * CONFIG.sides.signs.get(tag, 1)
        sign_ok = "1" if rates[tag] * expected > 0 else "0"

    bats = [s.battery for s in fresh if isinstance(s.battery, (int, float))]
    p = Point(phase=phase, repeat=repeat, slot=slot, duty=int(duty),
              ref_duty=int(duty if ref_duty is None else ref_duty),
              tag=tag if moving else "", rps=rps[tag] if moving else 0.0,
              sign_ok=sign_ok, crosstalk=round(crosstalk, 4), n=n,
              battery=round(statistics.fmean(bats), 2) if bats else 0.0)

    notes = []
    if sign_ok == "0":
        notes.append("COUNTED BACKWARDS (decoder desync or wrong sign)")
    if crosstalk > CROSSTALK_FRAC:
        notes.append(f"crosstalk {crosstalk:.0%}")
    speed = (f"{p.tag} {p.rps:6.3f} rev/s ({p.rps * 60:5.1f} rpm)" if moving
             else "-- not moving")
    print(f"  {slot} {duty:+5d} -> {speed}  n={n:2d}  vbat={p.battery:.2f}"
          + ("  [" + "; ".join(notes) + "]" if notes else ""))
    return p


# ---------------------------------------------------------------------------
# Experiments
# ---------------------------------------------------------------------------
def limit_reason(rps: float, prev: Optional[float], max_rps: float) -> str:
    """Why a reading is past the encoder's speed limit, or "" if it is not.
    ``prev`` is the same motor/direction's reading at the previous duty."""
    if rps > max_rps:
        return f"over max-rps ({rps:.2f} > {max_rps:g})"
    if prev and rps < prev * (1.0 - DROP_FRAC):
        return f"fell with more duty ({prev:.2f} -> {rps:.2f} rev/s)"
    return ""


def flag_limits(points: List[Point], max_rps: float) -> None:
    """Offline twin of the sweep's stop rule, for a CSV recorded without it:
    per repeat/motor/direction, flag the first reading past the limit and every
    higher duty after it."""
    groups: Dict[tuple, List[Point]] = {}
    for p in points:
        if p.phase == "sweep":
            key = (p.repeat, p.slot, direction_of(p.duty))
            groups.setdefault(key, []).append(p)
        elif p.rps > max_rps and not p.note:
            p.note = f"over max-rps ({p.rps:.2f} > {max_rps:g})"
    for pts in groups.values():
        prev, hit = None, False
        for p in sorted(pts, key=lambda q: abs(q.duty)):
            if hit:
                p.note = p.note or "above the encoder limit"
                continue
            p.note = p.note or limit_reason(p.rps, prev, max_rps)
            if p.note:
                hit = True
            else:
                prev = p.rps


def sweep(kart, stream, args, out: List[Point]) -> None:
    for rep in range(args.repeats):
        prev: Dict[tuple, float] = {}       # (slot, dir) -> rps, previous level
        done: Dict[tuple, int] = {}         # (slot, dir) -> duty it gave out at
        for level in args.levels:
            for sign in (1, -1):
                d = direction_of(sign)
                todo = [s for s in SLOTS if (s, d) not in done]
                if not todo:
                    continue
                print(f"[sweep {rep + 1}/{args.repeats}] duty {sign * level:+d}")
                for slot in todo:
                    p = measure(kart, stream, slot, sign * level, args,
                                repeat=rep)
                    if p is None:
                        continue        # telemetry hiccup: no evidence either way
                    p.note = limit_reason(p.rps, prev.get((slot, d)), args.max_rps)
                    if p.note:
                        done[(slot, d)] = level
                        print(f"  [limit] {slot} {d}: encoder past its speed "
                              f"limit -- {p.note}; higher duties skipped")
                    else:
                        prev[(slot, d)] = p.rps
                    out.append(p)
            if len(done) == 2 * len(SLOTS):
                print("every motor reached the encoder limit; sweep done")
                break


def verify(kart, stream, params, args, out: List[Point]) -> None:
    for level in args.verify_levels:
        for sign in (1, -1):
            u = sign * level
            print(f"[verify] reference duty {u:+d}: raw, then corrected")
            for phase in ("verify_raw", "verify_cor"):
                for slot in SLOTS:
                    duty = u if phase == "verify_raw" else correct_pwm(u, slot, params)
                    p = measure(kart, stream, slot, duty, args, phase=phase,
                                ref_duty=u)
                    if p is None:
                        continue
                    if p.rps > args.max_rps:
                        p.note = f"over max-rps ({p.rps:.2f} > {args.max_rps:g})"
                        print(f"  [limit] {slot}: {p.note}, not trusted")
                    out.append(p)


# ---------------------------------------------------------------------------
# Fitting
# ---------------------------------------------------------------------------
def fit_line(pts):
    """w = k*u + c by least squares, one pass of MAD outlier rejection (a
    point whose counts went missing near the encoder ceiling, a bad window)."""
    if len({u for u, _ in pts}) < 3:
        return None
    k, c = _linfit(*zip(*pts))
    res = [w - (k * u + c) for u, w in pts]
    med = statistics.median(res)
    mad = statistics.median(abs(r - med) for r in res)
    thr = max(4.0 * 1.4826 * mad, 0.01 * max(w for _, w in pts))
    kept = [p for p, r in zip(pts, res) if abs(r - med) <= thr]
    dropped = len(pts) - len(kept)
    if dropped and len({u for u, _ in kept}) >= 3:
        k, c = _linfit(*zip(*kept))
        pts = kept
    else:
        dropped = 0
    my = statistics.fmean(w for _, w in pts)
    ss_tot = sum((w - my) ** 2 for _, w in pts)
    ss_res = sum((w - (k * u + c)) ** 2 for u, w in pts)
    return {"k": k, "c": c, "u0": -c / k if k > 0 else None,
            "r2": 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0,
            "n": len(pts), "dropped": dropped}


def _usable(points: List[Point]) -> List[Point]:
    """Sweep readings fit to use: the wheel moved, its encoder was inside the
    speed limit and counted the right way."""
    return [p for p in points if p.phase == "sweep" and p.tag and not p.note
            and p.sign_ok != "0"]


def fit_range(points: List[Point], args):
    """(lo, hi, tops): the duty range to fit, and each motor/direction's
    highest usable duty.

    hi defaults to the LOWEST of those tops, so every line is fitted over the
    same duties; comparing lines fitted over different stretches of a bent
    curve would put the bend into the correction. lo defaults to hi - fit_span.
    """
    tops: Dict[tuple, int] = {}
    for p in _usable(points):
        key = (p.slot, direction_of(p.duty))
        tops[key] = max(tops.get(key, 0), abs(p.duty))
    if not tops:
        return None
    hi = min(tops.values()) if args.fit_max is None else args.fit_max
    lo = hi - args.fit_span if args.fit_min is None else args.fit_min
    return lo, hi, tops


def fit_curves(points: List[Point], fit_min: float, fit_max: float) -> dict:
    curves: dict = {}
    usable = _usable(points)
    for slot in SLOTS:
        for d in ("fwd", "rev"):
            pts = [(abs(p.duty), p.rps) for p in usable
                   if p.slot == slot and direction_of(p.duty) == d
                   and fit_min <= abs(p.duty) <= fit_max]
            fit = fit_line(pts)
            if fit is not None and fit["k"] > 0:
                curves.setdefault(slot, {})[d] = fit
    return curves


def slot_mapping(points: List[Point]) -> dict:
    mapping = {}
    for slot in SLOTS:
        seen = Counter(p.tag for p in points if p.slot == slot and p.tag)
        if not seen:
            print(f"[warn] {slot}: no encoder ever moved -- motor or encoder dead?")
            continue
        tag, n = seen.most_common(1)[0]
        mapping[slot] = tag
        if n < sum(seen.values()):
            print(f"[warn] {slot}: answered by several encoders {dict(seen)}")
        side = CONFIG.sides.left if slot in LEFT_SLOTS else CONFIG.sides.right
        if tag not in side:
            print(f"[warn] {slot} drives {tag}, which config.SideMapping puts "
                  f"on the OTHER side -- odometry sides are swapped")
    if len(set(mapping.values())) < len(mapping):
        print(f"[warn] two slots drive the same encoder: {mapping}")
    return mapping


def build_params(points: List[Point], args) -> Optional[dict]:
    rng = fit_range(points, args)
    if rng is None:
        print("[ERROR] no usable readings at all -- nothing to fit")
        return None
    lo, hi, tops = rng
    print("\nusable up to duty (encoder limit or last level): " +
          "  ".join(f"{s} {d} {tops[(s, d)]}" for s in SLOTS
                    for d in ("fwd", "rev") if (s, d) in tops))
    curves = fit_curves(points, lo, hi)
    for (s, d), top in tops.items():
        if d in curves.get(s, {}):
            curves[s][d]["top"] = top
    ref = curves.get(REFERENCE[0], {}).get(REFERENCE[1])
    if ref is None:
        print(f"[ERROR] no usable {REFERENCE[0]} {REFERENCE[1]} line in duty "
              f"[{lo:g}, {hi:g}] (needs 3+ levels) -- nothing to calibrate "
              f"against. Use finer --levels or widen --fit-span.")
        return None
    bats = [p.battery for p in points if p.phase == "sweep" and p.battery]
    params = {
        "version": 1,
        "created": datetime.now().isoformat(timespec="seconds"),
        "reference": {"motor": REFERENCE[0], "direction": REFERENCE[1]},
        "model": "w = k*|duty| + c (wheel rev/s, kart suspended); "
                 "duty_target = a*|duty_ref| + b",
        "fit_range": [lo, hi],
        "max_rps": args.max_rps,
        "battery_v": round(statistics.fmean(bats), 2) if bats else None,
        "slots": slot_mapping(points),
        "curves": {s: {d: {k: (round(v, 9) if isinstance(v, float) else v)
                           for k, v in f.items()}
                       for d, f in by_d.items()}
                   for s, by_d in curves.items()},
        "correction": {s: {d: {"a": round(c["a"], 5), "b": round(c["b"], 2)}
                           for d, c in by_d.items()}
                       for s, by_d in coefficients_from_curves(curves,
                                                               REFERENCE).items()},
    }
    for slot in SLOTS:
        for d in ("fwd", "rev"):
            if d not in params["correction"].get(slot, {}):
                print(f"[warn] {slot} {d}: no fit -> left UNCORRECTED")
    return params


def report(params: dict) -> None:
    lo, hi = params["fit_range"]
    probe = (int((lo + hi) / 2), int(hi))
    ref = params["curves"][REFERENCE[0]][REFERENCE[1]]
    w_ref = {u: ref["k"] * u + ref["c"] for u in probe}
    print("\n" + "=" * 78)
    print(f"FIT  w = k*u + c  over u in {params['fit_range']}   "
          f"reference {REFERENCE[0]} {REFERENCE[1]}   "
          f"vbat {params.get('battery_v')}")
    print("=" * 78)
    print(f"{'slot':4} {'enc':3} {'dir':3} {'k [rps/duty]':>13} {'u0':>7} "
          f"{'r2':>7} {'n':>3} | {'a':>7} {'b':>8} | "
          f"speed vs ref @{probe[0]} / @{probe[1]}")
    for slot in SLOTS:
        for d in ("fwd", "rev"):
            f = params["curves"].get(slot, {}).get(d)
            c = params["correction"].get(slot, {}).get(d)
            enc = params["slots"].get(slot, "?")
            if f is None:
                print(f"{slot:4} {enc:3} {d:3}  -- no fit")
                continue
            dev = "  ".join(f"{((f['k'] * u + f['c']) / w_ref[u] - 1) * 100:+6.1f}%"
                            for u in probe)
            drop = f"  ({f['dropped']} outlier)" if f.get("dropped") else ""
            print(f"{slot:4} {enc:3} {d:3} {f['k']:13.4e} {f['u0'] or 0:7.0f} "
                  f"{f['r2']:7.4f} {f['n']:3d} | {c['a']:7.4f} {c['b']:+8.1f} | "
                  f"{dev}{drop}")
    low = [(s, d) for s, by in params["curves"].items() for d, f in by.items()
           if f["r2"] < 0.99]
    if low:
        print(f"[warn] r2 < 0.99 for {low}: the line is a poor model there -- "
              f"narrow --fit-min/--fit-max to the duties the heading uses")
    hg = CONFIG.heading
    if hg.output_limit > hi:
        print(f"[warn] the heading turn drives {hg.min_turn_duty:.0f}.."
              f"{hg.output_limit:.0f} duty, but the lines only cover {lo:g}.."
              f"{hi:g}: above that correct_pwm EXTRAPOLATES them. Calibrating "
              f"higher needs a higher encoder speed limit (see --max-rps).")


def summarize_verification(points: List[Point]) -> Optional[dict]:
    """Left/right speed imbalance of a ccw and a cw turn, raw vs corrected."""
    by = {}
    for p in points:
        if p.phase.startswith("verify") and not p.note:
            by[(p.phase, abs(p.ref_duty), direction_of(p.ref_duty), p.slot)] = p.rps
    if not by:
        return None
    levels = sorted({k[1] for k in by})

    def side(phase, u, d, slots):
        vals = [by.get((phase, u, d, s)) for s in slots]
        return statistics.fmean(vals) if all(v is not None for v in vals) else None

    def imbalance(phase, u, left_dir, right_dir):
        left = side(phase, u, left_dir, LEFT_SLOTS)
        right = side(phase, u, right_dir, RIGHT_SLOTS)
        if not left or not right:
            return None
        return (right - left) / ((right + left) / 2.0) * 100.0

    def spread(phase, u):
        vals = [v for (ph, uu, _d, _s), v in by.items() if ph == phase and uu == u]
        return (max(vals) - min(vals)) / statistics.fmean(vals) * 100.0 \
            if len(vals) == 8 and min(vals) > 0 else None

    def fmt(x):
        return f"{x:+6.1f}%" if x is not None else "    n/a"

    print("\n" + "=" * 78)
    print("VERIFICATION  (right - left) / mean wheel speed of an in-place turn; "
          "0% = centred")
    print("=" * 78)
    print(f"{'duty':>5} | {'ccw raw':>8} {'ccw cor':>8} | {'cw raw':>8} "
          f"{'cw cor':>8} | {'8-wheel spread raw -> cor':>26}")
    summary = {}
    for u in levels:
        row = {
            "ccw_raw": imbalance("verify_raw", u, "rev", "fwd"),
            "ccw_cor": imbalance("verify_cor", u, "rev", "fwd"),
            "cw_raw": imbalance("verify_raw", u, "fwd", "rev"),
            "cw_cor": imbalance("verify_cor", u, "fwd", "rev"),
            "spread_raw": spread("verify_raw", u),
            "spread_cor": spread("verify_cor", u),
        }
        summary[str(u)] = {k: (round(v, 2) if v is not None else None)
                           for k, v in row.items()}
        sr, sc = row["spread_raw"], row["spread_cor"]
        print(f"{u:5d} | {fmt(row['ccw_raw']):>8} {fmt(row['ccw_cor']):>8} | "
              f"{fmt(row['cw_raw']):>8} {fmt(row['cw_cor']):>8} | "
              f"{'n/a' if sr is None else f'{sr:.1f}%':>12} -> "
              f"{'n/a' if sc is None else f'{sc:.1f}%':<11}")
    return summary


# ---------------------------------------------------------------------------
# Persistence
# ---------------------------------------------------------------------------
CSV_FIELDS = [f.name for f in fields(Point)]


def save_points(path: str, points: List[Point]) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        for p in points:
            w.writerow(asdict(p))
    print(f"\npoints saved to {path}")


def load_points(path: str) -> List[Point]:
    types = {"repeat": int, "duty": int, "ref_duty": int, "rps": float,
             "crosstalk": float, "n": int, "battery": float}
    with open(path, newline="") as f:
        return [Point(**{k: types.get(k, str)(v) for k, v in row.items()})
                for row in csv.DictReader(f)]


def save_params(path: str, params: dict) -> None:
    problem = validate(params)
    if problem:
        # Written anyway, for inspection: load_calibration on the Pi refuses
        # it for the same reason, so the kart stays uncorrected.
        print(f"[ERROR] calibration fails validation: {problem}")
    with open(path, "w") as f:
        json.dump(params, f, indent=2)
    print(f"calibration written to {path}"
          + ("" if problem else "  -> copy to Server/ on the Pi, restart the server"))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def parse_levels(text: str) -> List[int]:
    if ":" in text:
        a, b, step = (int(x) for x in text.split(":"))
        return list(range(a, b + 1, step))
    return [int(x) for x in text.split(",") if x.strip()]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--ip", default=DEFAULT_IP)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--levels", default=DEFAULT_LEVELS,
                    help="duty magnitudes: start:stop:step or a,b,c "
                         f"(default {DEFAULT_LEVELS})")
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--spinup", type=float, default=SPINUP_S)
    ap.add_argument("--window", type=float, default=WINDOW_S)
    ap.add_argument("--rest", type=float, default=REST_S)
    ap.add_argument("--max-rps", type=float, default=MAX_RPS,
                    help="wheel rev/s above which encoder readings are not "
                         f"trusted (default {MAX_RPS})")
    ap.add_argument("--fit-min", type=float, default=None,
                    help="default: fit-max - fit-span")
    ap.add_argument("--fit-max", type=float, default=None,
                    help="default: highest duty every motor was usable at")
    ap.add_argument("--fit-span", type=float, default=FIT_SPAN)
    ap.add_argument("--verify-levels", default="auto",
                    help="reference duties for the before/after check "
                         "(default: low/mid/top of the fitted range)")
    ap.add_argument("--no-verify", action="store_true")
    ap.add_argument("--out", default=CALIBRATION_PATH)
    ap.add_argument("--refit", metavar="CSV",
                    help="re-fit saved points offline; nothing is driven")
    args = ap.parse_args()
    args.levels = parse_levels(args.levels)
    auto_verify = args.verify_levels == "auto"
    args.verify_levels = [] if auto_verify else parse_levels(args.verify_levels)

    if args.refit:
        points = load_points(args.refit)
        flag_limits(points, args.max_rps)
        params = build_params(points, args)
        if params is None:
            sys.exit(1)
        report(params)
        ver = summarize_verification(points)
        if ver:
            params["verification"] = ver
        save_params(args.out, params)
        return

    stream = TelemetryStream(args.ip, args.port)
    stream.start()
    first = stream.wait_first(timeout=6.0)
    if first is None:
        print(f"[ERROR] no telemetry from ws://{args.ip}:{args.port}/ws -- "
              f"is the server running?")
        sys.exit(1)
    if first.mode != "one":
        print(f"[ERROR] kart is in mode {first.mode!r}; CMD_MOTOR is only "
              f"accepted in mode 'one'")
        sys.exit(1)
    if not first.precise:
        print("[warn] server sends no drive.control_ts (older server.py): rates "
              "are timed by the broadcast ts, ~1-2% noisier. Use --repeats 2+.")

    per_point = args.spinup + args.window + args.rest + 0.15
    n_sweep = args.repeats * len(args.levels) * 2 * len(SLOTS)
    n_verify = 0 if args.no_verify else (len(args.verify_levels) or 3) * 2 * 2 * len(SLOTS)
    print(f"kart {args.ip}:{args.port}   battery {first.battery} V")
    print(f"levels {args.levels}  x2 directions  x{len(SLOTS)} motors  "
          f"x{args.repeats} -> at most {n_sweep} points (+{n_verify} verify), "
          f"<= {(n_sweep + n_verify) * per_point / 60:.0f} min -- each motor "
          f"stops at its encoder limit ({args.max_rps:g} rev/s)")
    input("\nKart SUSPENDED, all wheels free? Enter to start (Ctrl+C aborts)... ")

    kart = Kart(args.ip, args.port)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(DATA_DIR, f"motor_cal_{ts}.csv")
    points: List[Point] = []
    params = None
    try:
        sweep(kart, stream, args, points)
        params = build_params(points, args)
        if params is not None:
            report(params)
            save_params(args.out, params)        # saved before verify spins
            if not args.no_verify and validate(params) is None:
                if auto_verify:
                    lo, hi = params["fit_range"]
                    args.verify_levels = sorted({int(round(x / 25.0)) * 25 for x in
                                                 (lo + (hi - lo) / 3, (lo + hi) / 2, hi)})
                verify(kart, stream, params, args, points)
                ver = summarize_verification(points)
                if ver:
                    params["verification"] = ver
                    save_params(args.out, params)
    except KeyboardInterrupt:
        print("\ninterrupted.")
    finally:
        kart.stop()
        stream.close()
        if points:
            save_points(csv_path, points)
            if params is None:
                print(f"re-fit what was measured with:  "
                      f"python Scripts/motor_calibration.py --refit {csv_path}")


if __name__ == "__main__":
    main()
