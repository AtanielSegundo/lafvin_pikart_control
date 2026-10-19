#!/usr/bin/python3
"""
Step-response auto-tuner for the position and heading PID loops.

Say what step response you want -- settling time, rise time, overshoot -- and
this searches kp, ki, kd and decel_gain ON THE KART until the measured response
matches. The search is CMA-ES, an evolution strategy: it only needs to rank
candidates, so the stiction floor, the decel ceiling, gyro noise and encoder
quirks all stay in the loop instead of being modelled away. max_time is not
searched but pinned to ts + tr: a move that has not settled by then is cut off
by the controller itself, and pays for it in the steady-state term below.

PROCEDURE
---------
1. Read the live gains from telemetry (msg.gains). They seed the search, set
   its box (--span either side, in log space) and are what gets restored on
   an abort.
2. Candidate 0 is the seed, run unchanged except for max_time: the baseline.
3. Each generation CMA-ES proposes --popsize candidates. Each one is sent with
   set_gains, CONFIRMED in telemetry (the loop must be running it, not merely
   have been told), then stepped --repeats times -- turn +/-step deg, or
   drive_distance +/-step m -- alternating so the kart stays where it started.
4. The --finalists best candidates and the seed are re-run --validate times,
   interleaved. A search score is one noisy draw and the best of eighty is
   partly the luckiest; only the re-runs pick the winner.
5. The winner is applied (unless --no-apply) and printed as a config.py
   snippet: set_gains changes P_control's copy only, lost on restart.

POSITION GAINS ARE SCHEDULED BY DISTANCE: PositionGains kp/ki/kd/decel_gain are
{breakpoint_m: value} dicts, published as kp@0.3 etc. A position run tunes the
ONE breakpoint --step falls into (the smallest breakpoint >= step, as the
controller picks it) and leaves the others alone. Run once per breakpoint, with
a --step inside its range. max_time is not scheduled: the winner's ts + tr
applies to every move length.

Ctrl-C once during the search: stop searching, validate what there is.
Ctrl-C again (or any error): stop the motors, restore the live gains.

METRICS  (folded so the commanded step is +r; t = 0 at the command)
-------
  y_end  mean over the last TAIL_AVG_S of the record -- the kart at rest
  tr     10% -> 90% of y_end                 (MATLAB stepinfo's convention)
  ts     last exit from y_end +/- --band %
  Mpp    (max(y) - y_end) / y_end * 100      overshoot against the END value
  ess    |r - y_end| beyond the loop's own arrival tolerance. Not a spec, but
         without it a move that stops halfway matches tr/ts/Mpp perfectly.
  drift  what the step should have left ALONE, before vs. after (same end
         window as y_end) -- each loop is also judged on the other's axis:
           heading   |(x, y)| the kart moved during the turn, m   ideal (0, 0)
           position  |heading change| during the drive, deg       ideal 0
         Both come from the odometry pose, whose heading is the gyro's when it
         is connected; the (x, y) is encoder dead reckoning, so it sees wheel
         asymmetry but not scrub the encoders do not count.

COST   w_ts*e_ts^2 + w_tr*e_tr^2 + w_mp*e_mp^2 + w_ss*e_ss^2 + w_dr*e_dr^2
       (--weights), with e = (measured - spec) / spec -- Mpp's scaled by
       max(spec, MP_FLOOR) so a 0 % spec is not infinitely strict --
       e_ss = ess / (band * r) and e_dr = drift / --drift-tol. Set --drift-tol
       to the drift you would accept: there it costs as much as missing ts by
       100 %. The baseline's drift (candidate 0) shows what the kart does now.
       --upper-bounds makes the specs ceilings: faster / smaller is free.

RESOLUTION: samples arrive at ControlConfig.telemetry_hz (10 Hz) and times are
interpolated between them. Raising telemetry_hz to the loop rate (20) halves
the step; t = 0 is mapped onto the Pi clock as in motor_calibration.py, good to
about one control tick.

Transport (laptop on the kart's network, as Scripts/motor_calibration.py):
  * commands  -> HTTP POST /command   JSON set_gains / turn / drive_distance
  * telemetry -> WebSocket /ws        gains, drive.goal_active, gyro, pose

USAGE
-----
    python Scripts/tuning.py heading  -ts 2.0 -tr 0.6 -Mpp 5
    python Scripts/tuning.py Position -ts 2.5 -tr 0.8 -Mpp 2 --step 0.3
    python Scripts/tuning.py heading  -ts 1.8 -tr 0.5 -Mpp 0 --upper-bounds \\
        --params kp,kd --generations 15

The kart in mode 'one', on the floor it will be used on, with room for --step
both ways (position: the front guard refuses a forward move at an obstacle).
Writes Scripts/data/tuning_<mode>_<stamp>.csv (one row per step, flushed as it
goes), .json (everything, responses included) and .png (the dashboard).
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import numpy as np
import requests as rq

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SERVER = os.path.join(REPO, "Server")
# Server/ is not a package -- see the same note in gray_regression.py.
if SERVER not in sys.path:
    sys.path.insert(0, SERVER)

from config import position_breakpoint                           # noqa: E402
from motor_calibration import (DEFAULT_IP, DEFAULT_PORT, Kart,  # noqa: E402
                               TelemetryStream)
from proc_control import (ControlApplier, schedule_key,          # noqa: E402
                          split_schedule_key)

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
TUNABLE       = ("kp", "ki", "kd", "decel_gain")
DEFAULT_STEP  = {"heading": 90.0, "position": 0.30}   # deg / m
UNIT          = {"heading": "deg", "position": "m"}
# Drift is measured on the OTHER axis: (x, y) for a turn, heading for a drive.
DRIFT_UNIT    = {"heading": "m", "position": "deg"}
DRIFT_TOL     = {"heading": 0.02, "position": 1.0}   # m / deg, see COST
GENERATIONS   = 10
SIGMA0        = 0.2      # initial CMA step, in the [0, 1]-scaled log box
SPAN          = 6.0      # search box: live value / SPAN .. live value * SPAN
REPEATS       = 2        # steps per candidate in the search: one +/- pair
FINALISTS     = 3
VALIDATE      = 4        # re-run steps per finalist (rounded up to even)
BAND_PCT      = 2.0
WEIGHTS       = "1,1,1,2,1"  # ts, tr, Mpp, steady-state error, drift
REST_S        = 0.8      # kart still this long before a step
TAIL_S        = 1.0      # keep recording after the loop declares the move done
TAIL_AVG_S    = 0.4      # y_end = mean over the record's last 0.4 s
PRE_S         = 0.3      # baseline kept in the record before t = 0
START_GRACE_S = 1.5      # the move must show up in telemetry within this
STALE_S       = 1.0      # no telemetry for this long -> the step is void
STILL_YAW_DEG = 0.3      # yaw spread over REST_S that still counts as still
STILL_POS_M   = 0.002    # position spread over REST_S, same idea
MIN_GAIN      = 1.0      # log-space search: a gain never reaches exactly 0
ZERO_SEED_HI  = 1000.0   # box top for a gain whose live value is 0
MP_FLOOR      = 2.0      # %, overshoot error scale floor
FAIL_COST     = 1000.0   # a step that never got going
OUT_PENALTY   = 10.0     # per unit^2 a CMA sample lands outside the box
DATA_DIR      = os.path.join(HERE, "data")

# The server's own sanity rails for set_gains -- a value outside them would be
# refused there, and the confirmation below would then time out.
LIMITS = ControlApplier.GAIN_LIMITS


class TrialError(RuntimeError):
    """A step that could not be measured (as opposed to one measured and bad)."""


# ---------------------------------------------------------------------------
# Commands  (HTTP POST /command)
# ---------------------------------------------------------------------------
class TuningKart(Kart):
    """motor_calibration's sender -- its stop() is CMD_MOTOR zeros, which also
    releases the drive -- plus the JSON commands the tuner needs."""

    def send(self, payload: dict, retries: int = 1) -> bool:
        body = json.dumps(payload)
        for _ in range(retries):
            try:
                if self.http.post(self.url, data=body, timeout=1.0).ok:
                    return True
            except rq.RequestException:
                continue
        return False

    def set_gains(self, section: str, values: dict) -> bool:
        # Idempotent, so retried. A move is NOT: a retry after a lost response
        # would start a second move from wherever the first one had got to.
        return self.send({"type": "set_gains", "section": section,
                          "values": values}, retries=3)

    def step(self, mode: str, amount: float) -> bool:
        if mode == "heading":
            return self.send({"type": "turn", "angle": round(amount, 3)})
        return self.send({"type": "drive_distance", "distance": round(amount, 4)})


# ---------------------------------------------------------------------------
# Telemetry  (WebSocket /ws)
# ---------------------------------------------------------------------------
@dataclass
class Tick:
    t_local: float            # laptop monotonic, at receipt
    t_pi: float               # when P_control took the snapshot
    precise: bool             # t_pi is control_ts (vs. the broadcast ts)
    stale: bool
    mode: str
    battery: Optional[float]
    active: bool              # drive.goal_active: a move / turn is running
    guard: bool               # drive.guard_blocked
    gyro_ok: bool
    yaw_deg: float            # continuous, not wrapped (heading.GyroMPU)
    x: float
    y: float
    theta: float
    gains: dict


class Telemetry(TelemetryStream):
    """motor_calibration's stream (reconnects, answers pings), keeping what a
    step response needs instead of the encoder totals."""

    def _on_text(self, data: bytes) -> None:
        try:
            msg = json.loads(data.decode())
        except ValueError:
            return
        if msg.get("type") != "telemetry":
            return
        drive = msg.get("drive") or {}
        pose = drive.get("pose") or {}
        gyro = drive.get("gyro") or {}
        cts = drive.get("control_ts")
        tick = Tick(
            t_local=time.monotonic(),
            t_pi=float(cts) if cts else float(msg.get("ts", 0.0)),
            precise=bool(cts),
            stale=bool(drive.get("stale", False)),
            mode=str(msg.get("mode", "")),
            battery=msg.get("battery"),
            active=bool(drive.get("goal_active", False)),
            guard=bool(drive.get("guard_blocked", False)),
            gyro_ok=bool(gyro.get("connected", False)),
            yaw_deg=float(gyro.get("yaw_deg", 0.0)),
            x=float(pose.get("x", 0.0)),
            y=float(pose.get("y", 0.0)),
            theta=float(pose.get("theta", 0.0)),
            gains=msg.get("gains") or {})
        with self._lock:
            # The 10 Hz broadcaster can send one 20 Hz snapshot twice, with the
            # same stamp. Keep the first arrival: the clock mapping in record()
            # wants the shortest delay.
            last = self._samples[-1] if self._samples else None
            if last is not None and tick.precise and tick.t_pi == last.t_pi:
                return
            self._samples.append(tick)

    def since(self, t_local: float) -> List[Tick]:
        """Ticks that ARRIVED since laptop time t_local."""
        with self._lock:
            return [s for s in self._samples if s.t_local >= t_local]

    def record(self, t_cmd: float, pre: float) -> Tuple[List[Tick], float]:
        """Ticks TAKEN from `pre` s before laptop time t_cmd on, and t_cmd on
        the Pi clock. Selected by when they were taken, not when they arrived:
        a WiFi stall delivers a backlog late (see TelemetryStream.between)."""
        with self._lock:
            samples = list(self._samples)
        kind = samples[-1].precise          # never mix the two clocks
        samples = [s for s in samples if s.precise == kind]
        offset = min(s.t_local - s.t_pi for s in samples)
        t0 = t_cmd - offset
        return sorted((s for s in samples if s.t_pi >= t0 - pre),
                      key=lambda s: s.t_pi), t0


def scheduled_breakpoints(gains: dict) -> List[float]:
    """Breakpoints of the distance-scheduled gains in a telemetry gains dict
    (keys like kp@0.3); empty for an unscheduled section (heading)."""
    return sorted({float(bp) for name, bp in map(split_schedule_key, gains)
                   if name in TUNABLE and bp is not None})


def fetch_live_gains(tel: Telemetry, section: str, step: float,
                     timeout: float = 3.0
                     ) -> Optional[Tuple[dict, Optional[float]]]:
    """The gains P_control is running right now, and the breakpoint `step`
    selects (None when the section is not scheduled). Scheduled gains are
    returned under their plain names (kp, not kp@0.3), so the search never
    sees the schedule. They ride on the control loop's snapshots, so the first
    frames after a restart may lack them."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        s = tel.latest()
        g = (s.gains or {}).get(section) if s is not None else None
        if g:
            bps = scheduled_breakpoints(g)
            bp = position_breakpoint(step, bps) if bps else None
            view = dict(g)
            if bp is not None:
                for n in TUNABLE:
                    view[n] = g.get(schedule_key(n, bp))
            if all(view.get(k) is not None
                   for k in (*TUNABLE, "max_time", "tolerance")):
                return view, bp
        time.sleep(0.1)
    return None


# ---------------------------------------------------------------------------
# One step on the kart
# ---------------------------------------------------------------------------
@dataclass
class Response:
    t: np.ndarray             # s since the command (Pi clock), PRE_S before 0
    y: np.ndarray             # progress toward the target, folded positive
    drift: np.ndarray         # per tick, off-axis motion since the start:
                              # heading (N, 2) forward/left m in the start
                              # frame; position (N, 1) heading change, deg
    direction: int
    battery: Optional[float]
    active_s: float           # how long telemetry showed the move running


class Rig:
    """The kart side of a trial: push gains, take one step, hand back the
    response. Knows nothing about specs or costs."""

    def __init__(self, mode: str, kart: TuningKart, tel: Telemetry,
                 rest: float, tail: float, dash: "Dashboard" = None,
                 breakpoint: Optional[float] = None):
        self.mode = mode
        self.kart = kart
        self.tel = tel
        self.rest = rest
        self.tail = tail
        self.dash = dash
        # Distance-scheduled section: the breakpoint being tuned. The search
        # speaks plain names (kp); the wire wants kp@<breakpoint>.
        self.breakpoint = breakpoint

    def wire(self, values: dict) -> dict:
        if self.breakpoint is None:
            return dict(values)
        return {(schedule_key(k, self.breakpoint) if k in TUNABLE else k): v
                for k, v in values.items()}

    def wait(self, seconds: float) -> None:
        """Sleep, keeping the dashboard window responsive."""
        end = time.monotonic() + seconds
        while True:
            if self.dash is not None:
                self.dash.pump()
            left = end - time.monotonic()
            if left <= 0:
                return
            time.sleep(min(0.02, left))

    def halt(self) -> None:
        self.kart.stop()

    def latest(self) -> Tick:
        s = self.tel.latest()
        if s is None or time.monotonic() - s.t_local > STALE_S:
            raise TrialError("telemetry stopped arriving")
        if s.stale:
            raise TrialError("the control loop stopped reporting "
                             "(drive.stale) -- check /health")
        return s

    def check_ready(self) -> None:
        s = self.latest()
        if s.mode != "one":
            raise TrialError(f"kart is in mode {s.mode!r}; moves need mode 'one'")
        if self.mode == "heading" and not s.gyro_ok:
            # turn_in_place would fall back to an ENCODER move, run by the
            # position gains -- the heading gains would be tuned blind.
            raise TrialError("gyro not connected")

    def push_gains(self, section: str, values: dict,
                   timeout: float = 3.0) -> None:
        """set_gains, then wait until telemetry shows the loop running them."""
        values = self.wire(values)
        t_sent = time.monotonic()
        if not self.kart.set_gains(section, values):
            raise TrialError("set_gains was not acknowledged (HTTP)")
        while time.monotonic() - t_sent < timeout:
            s = self.tel.latest()
            if s is not None and s.t_local > t_sent:
                live = (s.gains or {}).get(section) or {}
                if all(k in live and math.isclose(live[k], v, rel_tol=1e-6,
                                                  abs_tol=1e-4)
                       for k, v in values.items()):
                    return
            self.wait(0.05)
        raise TrialError(f"{section} gains did not land within {timeout:g} s "
                         f"-- refused? (the server logs why)")

    def wait_still(self, timeout: float = 15.0) -> List[Tick]:
        """Block until no move runs and the kart has not moved for `rest` s.
        Returns that quiet window: the step's baseline."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.wait(0.05)
            now = time.monotonic()
            if self.latest().active:
                continue
            win = self.tel.since(now - self.rest)
            if (len(win) < 3 or any(w.active for w in win)
                    or win[0].t_local > now - 0.8 * self.rest):
                continue
            # Both axes in both modes: the baseline holds the step's own axis
            # AND the one its drift is measured on.
            yaw = [w.yaw_deg for w in win]
            xs, ys = [w.x for w in win], [w.y for w in win]
            if (max(yaw) - min(yaw) < STILL_YAW_DEG
                    and max(xs) - min(xs) < STILL_POS_M
                    and max(ys) - min(ys) < STILL_POS_M):
                return win
        raise TrialError(f"the kart did not come to rest within {timeout:g} s")

    @staticmethod
    def _baseline(quiet: List[Tick]) -> tuple:
        """(yaw_deg, x, y, theta) of the kart at rest before the step."""
        return (float(np.mean([k.yaw_deg for k in quiet])),
                float(np.mean([k.x for k in quiet])),
                float(np.mean([k.y for k in quiet])), quiet[-1].theta)

    def _progress(self, k: Tick, base: tuple) -> float:
        yaw0, x0, y0, th = base
        if self.mode == "heading":
            return k.yaw_deg - yaw0
        # Distance along the heading the move started on: drive_distance is a
        # straight line, so sideways drift is not progress.
        return (k.x - x0) * math.cos(th) + (k.y - y0) * math.sin(th)

    def _drift(self, k: Tick, base: tuple) -> Tuple[float, ...]:
        _, x0, y0, th = base
        if self.mode == "heading":
            # In the frame the turn started in, so the components mean the same
            # thing on every step: forward = turn centre off the axle line,
            # left = off the kart's centreline.
            dx, dy = k.x - x0, k.y - y0
            return (dx * math.cos(th) + dy * math.sin(th),
                    -dx * math.sin(th) + dy * math.cos(th))
        return (math.degrees(math.remainder(k.theta - th, math.tau)),)

    def step(self, direction: int, size: float, max_time: float) -> Response:
        base = self._baseline(self.wait_still())
        self.check_ready()

        t_send = time.monotonic()
        if not self.kart.step(self.mode, direction * size):
            raise TrialError("the step command was not acknowledged (HTTP)")

        seen, t_done = False, None
        hard_end = t_send + max_time + self.tail + START_GRACE_S
        while True:
            self.wait(0.03)
            now = time.monotonic()
            s = self.latest()
            if s.t_local > t_send:
                if s.guard:
                    raise TrialError("the front guard cut the move -- clear "
                                     "the path")
                if s.active:
                    seen, t_done = True, None
                elif seen and t_done is None:
                    t_done = now
            if t_done is not None and now - t_done >= self.tail:
                break
            if not seen and now - t_send > START_GRACE_S:
                raise TrialError("the move never started (mode, front guard "
                                 "or a refused command?)")
            if now > hard_end:
                break       # the loop's max_time should have ended it by now

        ticks, t0 = self.tel.record(t_send, PRE_S)
        t = np.array([k.t_pi - t0 for k in ticks])
        y = np.array([self._progress(k, base) for k in ticks]) * direction
        drift = np.array([self._drift(k, base) for k in ticks])
        on = [k.t_pi for k in ticks if k.active]
        return Response(t, y, drift, direction, ticks[-1].battery,
                        (on[-1] - on[0]) if on else 0.0)


# ---------------------------------------------------------------------------
# Metrics and cost
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Spec:
    ts: float
    tr: float
    mp: float


@dataclass
class Metrics:
    tr: float
    ts: float
    mp: float                 # %
    ess: float                # step units, beyond the arrival tolerance
    y_end: float
    drift: float              # DRIFT_UNIT, off-axis motion start -> end


def _first_crossing(t: np.ndarray, y: np.ndarray, level: float) -> float:
    above = np.nonzero(y >= level)[0]
    if len(above) == 0:
        return math.inf
    i = int(above[0])
    if i == 0:
        return float(t[0])
    return float(t[i - 1] + (level - y[i - 1]) / (y[i] - y[i - 1])
                 * (t[i] - t[i - 1]))


def step_metrics(t: np.ndarray, y: np.ndarray, drift: np.ndarray, r: float,
                 band_pct: float, tolerance: float) -> Metrics:
    keep = t >= 0.0
    t, y, drift = t[keep], y[keep], drift[keep]
    end = t >= t[-1] - TAIL_AVG_S
    y_end = float(np.mean(y[end]))
    ess = max(0.0, abs(r - y_end) - tolerance)
    # Norm of the mean, not mean of the norms: noise must not add up to drift.
    dr = float(np.linalg.norm(np.mean(drift[end], axis=0)))
    if y_end < 0.1 * r:             # barely moved: tr/ts/Mpp mean nothing
        return Metrics(math.inf, math.inf, 0.0, ess, y_end, dr)

    tr = (_first_crossing(t, y, 0.9 * y_end)
          - _first_crossing(t, y, 0.1 * y_end))
    mp = max(0.0, (float(np.max(y)) - y_end) / y_end * 100.0)

    band = band_pct / 100.0 * y_end
    err = np.abs(y - y_end)
    out = np.nonzero(err > band)[0]
    if len(out) == 0:
        ts = 0.0
    elif out[-1] == len(y) - 1:     # still outside at the very end
        ts = float(t[-1])
    else:
        k = int(out[-1])            # interpolate the last entry into the band
        ts = float(t[k] + (err[k] - band) / (err[k] - err[k + 1])
                   * (t[k + 1] - t[k]))
    return Metrics(tr, ts, mp, ess, y_end, dr)


def drift_label(mode: str) -> str:
    return ("(x, y) moved during the turn" if mode == "heading"
            else "heading change during the drive")


def step_cost(m: Metrics, spec: Spec, weights, r: float, band_pct: float,
              drift_tol: float, upper: bool) -> float:
    if not (math.isfinite(m.tr) and math.isfinite(m.ts)):
        # Graded, so CMA-ES can still rank two failures: further is better.
        return FAIL_COST * (2.0 - max(0.0, m.y_end) / r)
    e = [(m.ts - spec.ts) / spec.ts,
         (m.tr - spec.tr) / spec.tr,
         (m.mp - spec.mp) / max(spec.mp, MP_FLOOR)]
    if upper:
        e = [max(0.0, x) for x in e]
    e.append(m.ess / (band_pct / 100.0 * r))
    e.append(m.drift / drift_tol)   # a minimum, not a spec: always one-sided
    return float(sum(w * x * x for w, x in zip(weights, e)))


# ---------------------------------------------------------------------------
# Search
# ---------------------------------------------------------------------------
class SearchSpace:
    """Gains <-> CMA-ES coordinates. Each gain lives on [lo, hi] in LOG space,
    scaled to [0, 1]: the gains differ by orders of magnitude between them and
    a loop answers ratios (kp x2), not differences (kp +500)."""

    def __init__(self, names, live: dict, span: float,
                 overrides: Dict[str, Tuple[float, float]], limits: dict):
        self.names = list(names)
        self.lo, self.hi = {}, {}
        for n in self.names:
            seed = float(live[n])
            if n in overrides:
                lo, hi = overrides[n]
            elif seed < MIN_GAIN:
                print(f"[note] {n} is 0 live; searching [{MIN_GAIN:g}, "
                      f"{ZERO_SEED_HI:g}] -- choose with --bound {n}=LO:HI")
                lo, hi = MIN_GAIN, ZERO_SEED_HI
            else:
                lo, hi = seed / span, seed * span
            lo = max(lo, MIN_GAIN, limits[n][0])
            hi = min(hi, limits[n][1])
            if hi <= lo * 1.01:
                raise SystemExit(f"[ERROR] {n}: empty search range "
                                 f"[{lo:g}, {hi:g}]")
            self.lo[n], self.hi[n] = lo, hi
        self._lo = np.log([self.lo[n] for n in self.names])
        self._hi = np.log([self.hi[n] for n in self.names])

    def encode(self, params: dict) -> np.ndarray:
        v = np.log([max(float(params[n]), MIN_GAIN) for n in self.names])
        return np.clip((v - self._lo) / (self._hi - self._lo), 0.0, 1.0)

    def decode(self, z: np.ndarray) -> Dict[str, float]:
        v = np.exp(self._lo + np.clip(z, 0.0, 1.0) * (self._hi - self._lo))
        return {n: float(f"{x:.5g}") for n, x in zip(self.names, v)}


class CMAES:
    """(mu/mu_w, lambda)-CMA-ES, minimising -- after Hansen, "The CMA Evolution
    Strategy: A Tutorial" (2016). Rank-based: it uses only the ORDER of the
    costs, which is what makes it tolerant of a noisy measurement. Written out
    rather than imported so the laptop needs nothing beyond numpy."""

    def __init__(self, x0: np.ndarray, sigma: float, popsize: int = None,
                 rng: np.random.Generator = None):
        n = len(x0)
        self.n, self.mean, self.sigma = n, np.asarray(x0, float), float(sigma)
        self.rng = rng if rng is not None else np.random.default_rng()
        self.lam = popsize or default_popsize(n)
        self.mu = self.lam // 2
        w = math.log(self.mu + 0.5) - np.log(np.arange(1, self.mu + 1))
        self.w = w / w.sum()
        self.mueff = 1.0 / float(np.sum(self.w ** 2))
        self.cc = (4 + self.mueff / n) / (n + 4 + 2 * self.mueff / n)
        self.cs = (self.mueff + 2) / (n + self.mueff + 5)
        self.c1 = 2 / ((n + 1.3) ** 2 + self.mueff)
        self.cmu = min(1 - self.c1, 2 * (self.mueff - 2 + 1 / self.mueff)
                       / ((n + 2) ** 2 + self.mueff))
        self.damps = (1 + 2 * max(0.0, math.sqrt((self.mueff - 1) / (n + 1)) - 1)
                      + self.cs)
        self.chi_n = math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n))
        self.pc, self.ps = np.zeros(n), np.zeros(n)
        self.B, self.D, self.C = np.eye(n), np.ones(n), np.eye(n)
        self.gen = 0

    def ask(self) -> np.ndarray:
        z = self.rng.standard_normal((self.lam, self.n))
        return self.mean + self.sigma * (z * self.D) @ self.B.T

    def tell(self, xs: np.ndarray, costs) -> None:
        sel = np.asarray(xs)[np.argsort(costs)[: self.mu]]
        old = self.mean
        self.mean = self.w @ sel
        y_w = (self.mean - old) / self.sigma

        inv_sqrt = self.B @ np.diag(1.0 / self.D) @ self.B.T
        self.ps = ((1 - self.cs) * self.ps
                   + math.sqrt(self.cs * (2 - self.cs) * self.mueff) * inv_sqrt @ y_w)
        ps_norm = float(np.linalg.norm(self.ps))
        hsig = (ps_norm / math.sqrt(1 - (1 - self.cs) ** (2 * (self.gen + 1)))
                / self.chi_n) < 1.4 + 2 / (self.n + 1)
        self.pc = ((1 - self.cc) * self.pc
                   + hsig * math.sqrt(self.cc * (2 - self.cc) * self.mueff) * y_w)

        art = (sel - old) / self.sigma
        self.C = ((1 - self.c1 - self.cmu) * self.C
                  + self.c1 * (np.outer(self.pc, self.pc)
                               + (1 - hsig) * self.cc * (2 - self.cc) * self.C)
                  + self.cmu * (art.T * self.w) @ art)
        self.sigma *= math.exp((self.cs / self.damps) * (ps_norm / self.chi_n - 1))

        self.C = (self.C + self.C.T) / 2
        d2, self.B = np.linalg.eigh(self.C)
        self.D = np.sqrt(np.maximum(d2, 1e-20))
        self.gen += 1


def default_popsize(n: int) -> int:
    return 4 + int(3 * math.log(n))


# ---------------------------------------------------------------------------
# Bookkeeping
# ---------------------------------------------------------------------------
@dataclass
class Trial:
    index: int
    phase: str                # search | validate
    response: Response
    metrics: Metrics
    cost: float


def _mean(xs) -> float:
    xs = [x for x in xs if math.isfinite(x)]
    return sum(xs) / len(xs) if xs else math.inf


@dataclass
class Candidate:
    cid: int
    kind: str                 # seed | cma
    gen: int                  # -1 for the seed
    params: Dict[str, float]  # all of TUNABLE; untuned ones keep the seed's
    search: List[Trial] = field(default_factory=list)
    validation: List[Trial] = field(default_factory=list)

    @property
    def trials(self) -> List[Trial]:
        return self.search + self.validation

    @property
    def cost(self) -> float:
        return _mean([t.cost for t in self.search])

    @property
    def val_cost(self) -> float:
        return _mean([t.cost for t in self.validation])

    def metric(self, key: str) -> float:
        return _mean([getattr(t.metrics, key) for t in self.trials])


def _fmt(x) -> str:
    return f"{x:.6g}" if isinstance(x, float) else str(x)


class TrialLog:
    """One CSV row per step, flushed as it goes: an aborted run keeps its data."""

    FIELDS = ("trial", "candidate", "kind", "gen", "phase", "direction",
              *TUNABLE, "max_time", "tr", "ts", "mpp", "ess", "y_end", "drift",
              "cost", "battery")

    def __init__(self, path: str):
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(self.FIELDS)

    def write(self, cand: Candidate, trial: Trial, max_time: float) -> None:
        m = trial.metrics
        self._w.writerow([_fmt(x) for x in (
            trial.index, cand.cid, cand.kind, cand.gen, trial.phase,
            trial.response.direction, *(cand.params[n] for n in TUNABLE),
            max_time, m.tr, m.ts, m.mp, m.ess, m.y_end, m.drift, trial.cost,
            trial.response.battery)])
        self._f.flush()

    def close(self) -> None:
        self._f.close()


# ---------------------------------------------------------------------------
# The run
# ---------------------------------------------------------------------------
class Tuner:
    def __init__(self, args, rig: Rig, live: dict, dash: "Dashboard" = None,
                 log: TrialLog = None):
        self.args = args
        self.rig = rig
        self.section = args.mode
        self.breakpoint = rig.breakpoint      # None unless scheduled
        self.unit = UNIT[args.mode]
        self.live = live                      # the restore point
        self.spec = Spec(args.ts, args.tr, args.mpp)
        self.max_time = round(args.ts + args.tr, 2)
        self.r = args.step
        tol = float(live["tolerance"])
        self.tolerance = math.degrees(tol) if args.mode == "heading" else tol
        self.drift_unit = DRIFT_UNIT[args.mode]
        self.drift_tol = args.drift_tol
        self.names = args.params
        self.space = SearchSpace(self.names, live, args.span, args.bound,
                                 LIMITS[self.section])
        self.seed = Candidate(0, "seed", -1, {n: float(live[n]) for n in TUNABLE})
        self.cands: List[Candidate] = [self.seed]
        self.dash = dash
        self.log = log
        self.n_trials = 0
        self.status = ""
        self.winner: Optional[Candidate] = None

    # -- evaluation ----------------------------------------------------------
    def best(self) -> Optional[Candidate]:
        if self.winner is not None:
            return self.winner
        done = [c for c in self.cands if c.search]
        return min(done, key=lambda c: c.cost) if done else None

    def _step(self, direction: int) -> Response:
        for attempt in (1, 2):
            try:
                return self.rig.step(direction, self.r, self.max_time)
            except TrialError as e:
                if attempt == 2:
                    raise
                print(f"  [retry] {e}")
                self.rig.halt()
                self.rig.wait(1.0)

    def evaluate(self, cand: Candidate, n: int, phase: str) -> None:
        """n steps of `cand`, alternating +/-: an even n leaves the kart where
        it started and tests both directions equally."""
        self.rig.push_gains(self.section, {**cand.params,
                                           "max_time": self.max_time})
        for k in range(n):
            resp = self._step(1 if k % 2 == 0 else -1)
            m = step_metrics(resp.t, resp.y, resp.drift, self.r,
                             self.args.band, self.tolerance)
            trial = Trial(self.n_trials, phase, resp, m,
                          step_cost(m, self.spec, self.args.weights, self.r,
                                    self.args.band, self.drift_tol,
                                    self.args.upper_bounds))
            self.n_trials += 1
            (cand.search if phase == "search" else cand.validation).append(trial)
            if self.log is not None:
                self.log.write(cand, trial, self.max_time)
            self._print(cand, trial)
            if self.dash is not None:
                self.dash.render(self)

    def _print(self, cand: Candidate, trial: Trial) -> None:
        m = trial.metrics
        gains = " ".join(f"{n} {cand.params[n]:<7.5g}" for n in self.names)
        print(f"#{trial.index:<3} c{cand.cid:<3} {trial.phase:<8} "
              f"{trial.response.direction * self.r:+g} {self.unit:<3} | {gains}"
              f"| tr {m.tr:5.2f}  ts {m.ts:5.2f}  Mpp {m.mp:5.1f}%  "
              f"ess {m.ess:.3g}  drift {m.drift:.3g} {self.drift_unit} "
              f"| J {trial.cost:.3g}")

    # -- phases --------------------------------------------------------------
    def search(self) -> None:
        a = self.args
        es = CMAES(self.space.encode(self.seed.params), a.sigma, a.popsize,
                   np.random.default_rng(a.rng_seed))
        for g in range(a.generations):
            xs = es.ask()
            costs = []
            for x in xs:
                z = np.clip(x, 0.0, 1.0)
                cand = Candidate(len(self.cands), "cma", g,
                                 {**self.seed.params, **self.space.decode(z)})
                self.cands.append(cand)
                self.status = (f"generation {g + 1}/{a.generations} | "
                               f"sigma {es.sigma:.3f}")
                self.evaluate(cand, a.repeats, "search")
                # Rank the UNCLIPPED sample, penalised for leaving the box, so
                # the distribution is pulled back in rather than piling up on
                # the wall it was clipped to.
                costs.append(cand.cost + OUT_PENALTY * float(np.sum((x - z) ** 2)))
            es.tell(xs, costs)
            best = self.best()
            print(f"--- generation {g + 1}/{a.generations}: best of gen "
                  f"{min(costs):.3g}, overall #{best.cid} {best.cost:.3g}, "
                  f"sigma {es.sigma:.3f}")
            if es.sigma < 1e-3:
                print("--- converged")
                break

    def validate(self) -> Candidate:
        ranked = sorted((c for c in self.cands if c.kind != "seed" and c.search),
                        key=lambda c: c.cost)
        finalists = [self.seed] + ranked[: self.args.finalists]
        rounds = (self.args.validate + 1) // 2
        if rounds == 0 or len(finalists) == 1:
            return self.best()
        print(f"\n=== validation: {len(finalists)} candidates x {2 * rounds} "
              f"steps, interleaved ===")
        # Interleaved, so battery sag and motor warm-up land on every finalist
        # alike instead of on whoever happens to run last.
        for k in range(rounds):
            for c in finalists:
                self.status = f"validation round {k + 1}/{rounds} | #{c.cid}"
                self.evaluate(c, 2, "validate")
        return min(finalists, key=lambda c: c.val_cost)

    def run(self) -> bool:
        a = self.args
        lam = a.popsize or default_popsize(len(self.names))
        n_steps = (a.repeats * (1 + a.generations * lam)
                   + 2 * ((a.validate + 1) // 2) * (a.finalists + 1))
        per_step = self.max_time + a.tail + a.rest + 0.5
        print(f"spec: ts {a.ts:g} s, tr {a.tr:g} s, Mpp {a.mpp:g} %  ->  "
              f"max_time {self.max_time:g} s")
        print(f"drift: {drift_label(self.section)}, ideal 0, tol "
              f"{self.drift_tol:g} {self.drift_unit}")
        print("search box: " + ", ".join(
            f"{n} [{self.space.lo[n]:.4g}, {self.space.hi[n]:.4g}]"
            for n in self.names))
        print(f"plan: up to ~{n_steps} steps of {self.r:g} {self.unit}, "
              f"~{n_steps * per_step / 60:.0f} min worst case\n")
        try:
            self.rig.check_ready()
            self.status = "baseline: live gains"
            self.evaluate(self.seed, a.repeats, "search")
            try:
                self.search()
            except KeyboardInterrupt:
                print("\n[search interrupted] validating what there is -- "
                      "Ctrl-C again to abort")
                self.rig.halt()
            self.winner = self.validate()
            self.finish()
            return True
        except (KeyboardInterrupt, TrialError) as e:
            self.rig.halt()
            why = "interrupted" if isinstance(e, KeyboardInterrupt) else str(e)
            print(f"\n[ABORT] {why}")
            self.restore()
            return False

    # -- outcome -------------------------------------------------------------
    def restore(self) -> None:
        values = {n: self.live[n] for n in (*TUNABLE, "max_time")}
        try:
            self.rig.push_gains(self.section, values)
            print("live gains restored")
        except TrialError as e:
            print(f"[ERROR] could not restore the gains ({e}); set by hand: "
                  + json.dumps(values))

    def finish(self) -> None:
        w = self.winner
        if self.args.no_apply:
            self.restore()
        else:
            self.rig.push_gains(self.section, {**w.params,
                                               "max_time": self.max_time})
        self.status = f"winner #{w.cid}" + (" (the live gains)" if w is self.seed
                                            else "")
        if self.dash is not None:
            self.dash.render(self)

        s = self.seed
        print(f"\n=== Result: candidate #{w.cid}"
              + (" -- the live gains were already the closest" if w is s else "")
              + " ===")
        print(f"{'':12}{'live':>12}{'winner':>12}")
        for n in TUNABLE:
            print(f"{n:12}{s.params[n]:>12.5g}{w.params[n]:>12.5g}")
        print(f"{'max_time':12}{self.live['max_time']:>12.5g}{self.max_time:>12.5g}")
        print(f"\n{'':12}{'spec':>12}{'live':>12}{'winner':>12}")
        for key, label, target in (("ts", "ts (s)", self.spec.ts),
                                   ("tr", "tr (s)", self.spec.tr),
                                   ("mp", "Mpp (%)", self.spec.mp)):
            print(f"{label:12}{target:>12.3g}{s.metric(key):>12.3g}"
                  f"{w.metric(key):>12.3g}")
        print(f"{'ess (' + self.unit + ')':12}{'-':>12}{s.metric('ess'):>12.3g}"
              f"{w.metric('ess'):>12.3g}")
        print(f"{'drift (' + self.drift_unit + ')':12}{0:>12}"
              f"{s.metric('drift'):>12.3g}{w.metric('drift'):>12.3g}")
        print(f"{'cost':12}{'-':>12}{s.val_cost:>12.3g}{w.val_cost:>12.3g}"
              "   (validation means)")

        print("\n" + ("NOT applied (--no-apply); live gains restored."
                      if self.args.no_apply else
                      "Applied to the running kart -- lost on restart."))
        cls = "HeadingGains" if self.section == "heading" else "PositionGains"
        if self.breakpoint is None:
            print(f"To keep it, Server/config.py -> {cls}:")
            for n in TUNABLE:
                print(f"    {n:<14}: float = {w.params[n]:.6g}")
        else:
            bp = self.breakpoint
            print(f"To keep it, Server/config.py -> {cls}, the {bp:g} entry "
                  f"of each schedule:")
            for n in TUNABLE:
                print(f"    {n:<14}: {{..., {bp:g}: {w.params[n]:.6g}, ...}}")
        print(f"    {'max_time':<14}: float = {self.max_time:g}")

    def save_json(self, path: str) -> None:
        def num(x):
            return x if isinstance(x, (int, float)) and math.isfinite(x) else None

        def trial(t: Trial) -> dict:
            return {"index": t.index, "phase": t.phase,
                    "direction": t.response.direction,
                    "battery": t.response.battery,
                    "active_s": round(t.response.active_s, 3),
                    "metrics": {k: num(v) for k, v in asdict(t.metrics).items()},
                    "cost": num(t.cost),
                    "t": [round(float(v), 4) for v in t.response.t],
                    "y": [round(float(v), 5) for v in t.response.y],
                    "drift": [[round(float(v), 5) for v in row]
                              for row in t.response.drift]}

        a = self.args
        out = {
            "mode": self.section, "unit": self.unit, "step": self.r,
            "breakpoint": self.breakpoint,
            "spec": {"ts": a.ts, "tr": a.tr, "mpp": a.mpp},
            "max_time": self.max_time, "band_pct": a.band,
            "drift_tol": self.drift_tol, "drift_unit": self.drift_unit,
            "drift_axes": (["forward_m", "left_m"] if self.section == "heading"
                           else ["heading_deg"]),
            "weights": list(a.weights), "upper_bounds": a.upper_bounds,
            "live_gains": self.live,
            "bounds": {n: [self.space.lo[n], self.space.hi[n]]
                       for n in self.names},
            "winner": self.winner.cid if self.winner is not None else None,
            "candidates": [{"cid": c.cid, "kind": c.kind, "gen": c.gen,
                            "params": c.params, "cost": num(c.cost),
                            "val_cost": num(c.val_cost),
                            "trials": [trial(t) for t in c.trials]}
                           for c in self.cands],
        }
        with open(path, "w") as f:
            json.dump(out, f, indent=1)


# ---------------------------------------------------------------------------
# Dashboard
# ---------------------------------------------------------------------------
# Chart ink and the first four categorical slots of the dataviz reference
# palette (validated: CVD and normal-vision separation pass on the light
# surface; aqua and yellow sit under 3:1 contrast, hence the direct labels on
# the gain lines and the table panel).
SURFACE, INK, INK_2, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
GRID, AXIS = "#e1e0d9", "#c3c2b7"
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100")
THEME = {
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.edgecolor": AXIS, "axes.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "grid.linestyle": "-", "axes.axisbelow": True,
    "axes.titlelocation": "left", "axes.titlesize": 10,
    "axes.titlecolor": INK, "axes.labelcolor": INK_2, "text.color": INK,
    "xtick.color": AXIS, "ytick.color": AXIS,
    "xtick.labelcolor": MUTED, "ytick.labelcolor": MUTED,
    "font.family": "sans-serif",
    "font.sans-serif": ["Segoe UI", "DejaVu Sans", "Arial"],
    "font.size": 9, "legend.frameon": False, "legend.fontsize": 8,
}


class Dashboard:
    """Live figure, rebuilt after every step. The rig pumps its event loop
    while it waits, so the window stays responsive while the kart moves."""

    RECENT = 6      # latest steps kept on the response panel, in grey

    def __init__(self, title: str):
        import matplotlib.pyplot as plt
        from matplotlib.ticker import FuncFormatter, MaxNLocator
        self.plt = plt
        self._ratio = FuncFormatter(lambda v, _: f"×{v:g}")
        # A locator binds to one axis, so each panel gets a fresh one.
        self._integer = lambda: MaxNLocator(integer=True)
        plt.rcParams.update(THEME)
        plt.ion()
        self.fig = plt.figure(figsize=(16, 9), layout="constrained")
        try:
            self.fig.canvas.manager.set_window_title(title)
        except AttributeError:
            pass
        self.fig.suptitle(title, x=0.01, ha="left", fontsize=12,
                          fontweight="bold")
        gs = self.fig.add_gridspec(3, 4, height_ratios=(1.4, 1.0, 1.0))
        self.ax_resp = self.fig.add_subplot(gs[0, :3])
        self.ax_cost = self.fig.add_subplot(gs[0, 3])
        self.ax_ts = self.fig.add_subplot(gs[1, 0])
        self.ax_tr = self.fig.add_subplot(gs[1, 1])
        self.ax_mp = self.fig.add_subplot(gs[1, 2])
        self.ax_dr = self.fig.add_subplot(gs[1, 3])
        self.ax_par = self.fig.add_subplot(gs[2, :3])
        self.ax_tab = self.fig.add_subplot(gs[2, 3])
        plt.show(block=False)

    def alive(self) -> bool:
        return self.plt.fignum_exists(self.fig.number)

    def pump(self) -> None:
        if self.alive():
            self.fig.canvas.flush_events()

    def render(self, tu: Tuner) -> None:
        # Artists are rebuilt even with the window closed, so the saved PNG is
        # current; only the screen refresh needs a live window.
        best = tu.best()
        trials = sorted(((c, t) for c in tu.cands for t in c.trials),
                        key=lambda p: p[1].index)
        done = [c for c in tu.cands if c.search]
        if not done:
            return
        self._responses(tu, trials, best)
        self._cost(tu, done)
        self._metrics(tu, trials, best)
        self._gains(tu, done)
        self._table(tu, best)
        if self.alive():
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def _responses(self, tu: Tuner, trials, best: Candidate) -> None:
        ax = self.ax_resp
        ax.cla()
        r, b, s = tu.r, tu.args.band / 100.0, tu.spec
        t_end = max([t.response.t[-1] for _, t in trials] + [1.5 * s.ts])
        # The spec as an envelope around the commanded step: under the
        # overshoot ceiling until ts, inside the band after it.
        ax.fill_between([s.ts, t_end], r * (1 - b), r * (1 + b), color=GRID,
                        lw=0, zorder=0)
        ax.hlines(r * (1 + s.mp / 100.0), 0.0, s.ts, colors=MUTED, lw=1,
                  linestyles="--", zorder=1)
        ax.axvline(s.ts, color=MUTED, lw=1, ls="--", zorder=1)
        ax.axhline(r, color=INK_2, lw=1, zorder=1)
        ax.annotate("ts spec", (s.ts, 0.0), xytext=(4, 4),
                    textcoords="offset points", color=INK_2, fontsize=8)
        ax.annotate(f"Mpp spec {s.mp:g}%", (0.0, r * (1 + s.mp / 100.0)),
                    xytext=(4, 4), textcoords="offset points", color=INK_2,
                    fontsize=8)

        for _, t in trials[-self.RECENT:]:
            ax.plot(t.response.t, t.response.y, color=MUTED, lw=1, alpha=0.5,
                    zorder=2)
        ax.plot([], [], color=MUTED, lw=1, label=f"latest {self.RECENT} steps")
        shown = [(best, SERIES[0], 2.0,
                  f"best #{best.cid}" + (" (live gains)" if best is tu.seed
                                         else ""))]
        if best is not tu.seed:
            shown.append((tu.seed, SERIES[1], 1.5, "live gains #0"))
        for cand, color, lw, label in reversed(shown):
            for i, t in enumerate(cand.trials):
                ax.plot(t.response.t, t.response.y, color=color, lw=lw,
                        zorder=3, label=label if i == 0 else None)
        ax.set_xlim(-PRE_S, t_end)
        ax.set_xlabel("time since the command (s)")
        ax.set_ylabel(f"progress ({tu.unit})")
        ax.set_title(f"Step response, r = {tu.r:g} {tu.unit} "
                     "(both directions folded positive)")
        ax.legend(loc="lower right")

    def _cost(self, tu: Tuner, done: List[Candidate]) -> None:
        ax = self.ax_cost
        ax.cla()
        xs = [c.cid for c in done]
        ys = [max(c.cost, 1e-4) for c in done]
        ax.scatter(xs, ys, s=24, color=MUTED, lw=0, zorder=2,
                   label="candidate (mean of its steps)")
        ax.step(xs, np.minimum.accumulate(ys), where="post", color=SERIES[0],
                lw=2, zorder=3, label="best so far")
        val = [c for c in tu.cands if c.validation]
        if val:
            ax.scatter([c.cid for c in val], [max(c.val_cost, 1e-4) for c in val],
                       s=60, facecolors="none", edgecolors=SERIES[0], lw=1.5,
                       zorder=4, label="validation re-runs")
        ax.set_yscale("log")
        ax.xaxis.set_major_locator(self._integer())
        ax.set_xlabel("candidate")
        ax.set_title("Cost (lower = closer to spec)")
        ax.legend(loc="best")

    def _metrics(self, tu: Tuner, trials, best: Candidate) -> None:
        s = tu.spec
        drift = ("Drift |(x, y)| after the turn" if tu.section == "heading"
                 else "Heading drift after the drive")
        for ax, key, title, target, unit, mark in (
                (self.ax_ts, "ts", "Settling time ts", s.ts, "s", "spec"),
                (self.ax_tr, "tr", "Rise time tr, 10-90%", s.tr, "s", "spec"),
                (self.ax_mp, "mp", "Overshoot Mpp", s.mp, "%", "spec"),
                (self.ax_dr, "drift", drift, tu.drift_tol, tu.drift_unit,
                 "tol")):
            ax.cla()
            pts = [(c.cid, getattr(t.metrics, key)) for c, t in trials
                   if math.isfinite(getattr(t.metrics, key))]
            if pts:
                ax.scatter(*zip(*pts), s=16, color=MUTED, lw=0, zorder=2)
            mine = [(c, v) for c, v in pts if c == best.cid]
            if mine:
                ax.scatter(*zip(*mine), s=30, color=SERIES[0], lw=0, zorder=3)
            ax.axhline(target, color=INK_2, lw=1, ls="--", zorder=1)
            ax.annotate(mark, xy=(1.0, target),
                        xycoords=("axes fraction", "data"), xytext=(-2, 3),
                        textcoords="offset points", ha="right", color=INK_2,
                        fontsize=8)
            if key == "drift":
                ax.set_ylim(bottom=0.0)          # the ideal is the axis floor
            ax.xaxis.set_major_locator(self._integer())
            ax.set_title(f"{title} ({unit}), {mark} {target:g}")
            ax.set_xlabel("candidate (blue = best)")

    def _gains(self, tu: Tuner, done: List[Candidate]) -> None:
        ax = self.ax_par
        ax.cla()
        ax.axhline(1.0, color=INK_2, lw=1, zorder=1)
        xs = [c.cid for c in done]
        leader, running = None, []
        for c in done:
            if leader is None or c.cost < leader.cost:
                leader = c
            running.append(leader)
        for n in tu.names:
            color = SERIES[TUNABLE.index(n)]   # follows the gain, not --params
            ref = max(tu.seed.params[n], MIN_GAIN)
            ax.scatter(xs, [c.params[n] / ref for c in done], s=14, color=color,
                       alpha=0.35, lw=0, zorder=2)
            line = [c.params[n] / ref for c in running]
            ax.step(xs, line, where="post", color=color, lw=2, zorder=3, label=n)
            ax.annotate(f"{n} ×{line[-1]:.2f}", (xs[-1], line[-1]),
                        xytext=(6, 0), textcoords="offset points", va="center",
                        color=INK_2, fontsize=8)
        ax.set_yscale("log", base=2)
        ax.yaxis.set_major_formatter(self._ratio)
        ax.set_xlim(-0.5, xs[-1] + max(4.0, 0.18 * xs[-1]))
        ax.xaxis.set_major_locator(self._integer())
        ax.set_xlabel("candidate")
        ax.set_title("Gains relative to the live seed: line = best so far, "
                     "dots = candidates")
        ax.legend(loc="upper left", ncols=len(tu.names))

    def _table(self, tu: Tuner, best: Candidate) -> None:
        ax = self.ax_tab
        ax.cla()
        ax.axis("off")
        s, sp = tu.seed, tu.spec
        rows = [f"{'':<11}{'live':>10}{'best':>10}"]
        rows += [f"{n:<11}{s.params[n]:>10.5g}{best.params[n]:>10.5g}"
                 for n in TUNABLE]
        rows.append(f"{'max_time':<11}{tu.live['max_time']:>10.5g}"
                    f"{tu.max_time:>10.5g}")
        rows += ["", f"{'':<11}{'spec':>10}{'live':>10}{'best':>10}"]
        for key, label, target in (("ts", "ts (s)", sp.ts),
                                   ("tr", "tr (s)", sp.tr),
                                   ("mp", "Mpp (%)", sp.mp)):
            rows.append(f"{label:<11}{target:>10.3g}{s.metric(key):>10.3g}"
                        f"{best.metric(key):>10.3g}")
        rows.append(f"{'ess (' + tu.unit + ')':<11}{'-':>10}"
                    f"{s.metric('ess'):>10.3g}{best.metric('ess'):>10.3g}")
        rows.append(f"{'drift (' + tu.drift_unit + ')':<11}{0:>10}"
                    f"{s.metric('drift'):>10.3g}{best.metric('drift'):>10.3g}")
        rows.append(f"{'cost':<11}{'-':>10}{s.cost:>10.3g}{best.cost:>10.3g}")
        rows += ["", f"best #{best.cid} | step {tu.n_trials}", tu.status]
        ax.text(0.0, 1.0, "\n".join(rows), transform=ax.transAxes, va="top",
                ha="left", family="monospace", fontsize=8.5, color=INK)
        ax.set_title("Live vs best (mean per candidate)")

    def save(self, path: str) -> None:
        try:
            self.fig.savefig(path, dpi=130)
        except Exception as e:                                  # noqa: BLE001
            print(f"[plot] could not save {path}: {e}")

    def hold(self) -> None:
        if self.alive():
            print("close the dashboard window to exit")
            self.plt.ioff()
            self.plt.show()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Details: see the module docstring (Scripts/tuning.py).")
    ap.add_argument("mode", type=str.lower, choices=("position", "heading"),
                    help="the loop to tune (case-insensitive)")
    ap.add_argument("-ts", "--settling-time", dest="ts", type=float,
                    required=True, help="target settling time, s")
    ap.add_argument("-tr", "--rise-time", dest="tr", type=float, required=True,
                    help="target 10-90%% rise time, s")
    ap.add_argument("-Mpp", "--overshoot", dest="mpp", type=float,
                    required=True, help="target overshoot, %% of the final value")
    ap.add_argument("--ip", default=DEFAULT_IP)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--step", type=float, default=None,
                    help="step size: deg for heading (default 90), m for "
                         "position (default 0.30)")
    ap.add_argument("--params", default=",".join(TUNABLE),
                    help=f"gains to search (default {','.join(TUNABLE)}); the "
                         "rest keep their live values")
    ap.add_argument("--bound", action="append", default=[], metavar="NAME=LO:HI",
                    help="explicit search range for one gain (repeatable)")
    ap.add_argument("--span", type=float, default=SPAN,
                    help="default range: live / span .. live * span (%(default)s)")
    ap.add_argument("--generations", type=int, default=GENERATIONS)
    ap.add_argument("--popsize", type=int, default=None,
                    help="candidates per generation (default 4 + 3 ln n)")
    ap.add_argument("--sigma", type=float, default=SIGMA0,
                    help="initial CMA-ES step in the [0, 1] log box (%(default)s)")
    ap.add_argument("--repeats", type=int, default=REPEATS,
                    help="steps per candidate while searching (%(default)s)")
    ap.add_argument("--finalists", type=int, default=FINALISTS)
    ap.add_argument("--validate", type=int, default=VALIDATE,
                    help="re-run steps per finalist, rounded up to even; 0 "
                         "skips validation (%(default)s)")
    ap.add_argument("--band", type=float, default=BAND_PCT,
                    help="settling band, %% of the final value (%(default)s)")
    ap.add_argument("--drift-tol", type=float, default=None,
                    help="drift that costs as much as missing ts by 100%%: m "
                         "of (x, y) for heading (default "
                         f"{DRIFT_TOL['heading']:g}), deg of heading for "
                         f"position (default {DRIFT_TOL['position']:g})")
    ap.add_argument("--weights", default=WEIGHTS,
                    help="cost weights ts,tr,Mpp,ess,drift (%(default)s); a 0 "
                         "drops that term")
    ap.add_argument("--upper-bounds", action="store_true",
                    help="treat the specs as ceilings instead of targets")
    ap.add_argument("--rest", type=float, default=REST_S)
    ap.add_argument("--tail", type=float, default=TAIL_S)
    ap.add_argument("--rng-seed", type=int, default=None)
    ap.add_argument("--no-apply", action="store_true",
                    help="restore the live gains at the end instead of "
                         "applying the winner")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--out-dir", default=DATA_DIR)
    args = ap.parse_args(argv)

    if not (args.ts > 0 and args.tr > 0 and args.mpp >= 0):
        ap.error("-ts and -tr must be > 0 and -Mpp >= 0")
    if args.tr >= args.ts:
        ap.error("the rise time must be shorter than the settling time")
    lo, hi = LIMITS[args.mode]["max_time"]
    if not lo <= args.ts + args.tr <= hi:
        ap.error(f"max_time = ts + tr = {args.ts + args.tr:g} s is outside "
                 f"the server's [{lo:g}, {hi:g}]")
    args.step = args.step or DEFAULT_STEP[args.mode]
    if args.step <= 0:
        ap.error("--step must be > 0 (the direction alternates by itself)")
    args.drift_tol = args.drift_tol or DRIFT_TOL[args.mode]
    if args.drift_tol <= 0:
        ap.error("--drift-tol must be > 0")
    if args.tail < 2 * TAIL_AVG_S:
        ap.error(f"--tail must be >= {2 * TAIL_AVG_S:g} s: y_end is the mean "
                 f"of the last {TAIL_AVG_S:g} s")

    names = [p.strip() for p in args.params.split(",") if p.strip()]
    bad = set(names) - set(TUNABLE)
    if bad or not names:
        ap.error(f"--params: choose from {','.join(TUNABLE)} (got {args.params})")
    args.params = tuple(n for n in TUNABLE if n in names)

    bounds = {}
    for spec in args.bound:
        try:
            name, rng = spec.split("=")
            lo_s, hi_s = rng.split(":")
            bounds[name.strip()] = (float(lo_s), float(hi_s))
        except ValueError:
            ap.error(f"--bound {spec!r}: expected NAME=LO:HI")
        if name.strip() not in args.params:
            ap.error(f"--bound {spec!r}: {name.strip()} is not being tuned")
    args.bound = bounds

    try:
        args.weights = tuple(float(w) for w in args.weights.split(","))
    except ValueError:
        ap.error("--weights: five numbers, ts,tr,Mpp,ess,drift")
    if len(args.weights) != 5 or min(args.weights) < 0:
        ap.error("--weights: five non-negative numbers, ts,tr,Mpp,ess,drift")
    if args.repeats < 1 or args.generations < 0:
        ap.error("--repeats must be >= 1 and --generations >= 0")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)

    tel = Telemetry(args.ip, args.port)
    tel.start()
    if tel.wait_first(5.0) is None:
        print(f"[ERROR] no telemetry from ws://{args.ip}:{args.port}/ws -- "
              f"is web.py running?")
        tel.close()
        return 1
    fetched = fetch_live_gains(tel, args.mode, args.step)
    if fetched is None:
        print(f"[ERROR] telemetry carries no '{args.mode}' gains -- server "
              f"older than the tuning table?")
        tel.close()
        return 1
    live, bp = fetched
    if bp is not None:
        bps = scheduled_breakpoints(live)
        lower = max((b for b in bps if b < bp), default=0.0)
        upper = "" if bp < bps[-1] else " and longer"
        print(f"--step {args.step:g} m tunes the {bp:g} m breakpoint (moves "
              f"{lower:g} < d <= {bp:g} m{upper}); the others are untouched")
    print(f"live {args.mode} gains: " + ", ".join(
        f"{k}={live[k]:g}" for k in (*TUNABLE, "max_time")))

    os.makedirs(args.out_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join(args.out_dir, f"tuning_{args.mode}_{stamp}")
    title = (f"PID tuning: {args.mode} | spec ts {args.ts:g} s, "
             f"tr {args.tr:g} s, Mpp {args.mpp:g} %")

    dash = None if args.no_plot else Dashboard(title)
    rig = Rig(args.mode, TuningKart(args.ip, args.port), tel, args.rest,
              args.tail, dash, breakpoint=bp)
    log = TrialLog(base + ".csv")
    tuner = Tuner(args, rig, live, dash, log)
    try:
        ok = tuner.run()
    finally:
        log.close()
        tuner.save_json(base + ".json")
        if dash is not None:
            dash.save(base + ".png")
        tel.close()
        print(f"\nwrote {base}.csv / .json" + (" / .png" if dash else ""))
    if dash is not None:
        dash.hold()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
