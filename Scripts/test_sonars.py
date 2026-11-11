#!/usr/bin/python3
"""
Sonar bench test: checks the wiring of every HC-SR04 in config.SONAR_PINS,
pin by pin, with a log line for everything it sees.

Runs ON THE PI, against pigpiod, below UltrasonicPigpio: it records every edge
on every trigger and echo line itself, so it can tell a dead sensor from a
floating echo, a swapped trigger pair, or an echo that never ends.

STOP THE SERVER FIRST. P_sensors pings the same pins; interleaved triggers
garble both.

    sudo systemctl stop main-server.service
    sudo pigpiod                                  # if not already running
    python3 Scripts/test_sonars.py                # all sonars, 20 pings each
    python3 Scripts/test_sonars.py -s left -n 50  # one sonar, more pings
    python3 Scripts/test_sonars.py -v             # + every raw edge
    python3 Scripts/test_sonars.py --live         # round-robin, Ctrl+C stops
    python3 Scripts/test_sonars.py --driver 10    # through UltrasonicPigpio
    python3 Scripts/test_sonars.py --log sonar.log

STAGES
------
1. environment  pigpiod reachable, Pi revision, the pin table and
                config.gpio_conflicts().
2. idle         per sonar: TRIG driven low and read back; ECHO read with the
                pull resistor off, down and up. A healthy HC-SR04 drives ECHO
                LOW at rest, so the level ignores the pull. A level that
                FOLLOWS the pull is a floating line: ECHO not connected, the
                divider open, or the sensor unpowered.
3. pings        per sonar, one at a time: a 10 us trigger, then the echo pulse
                timed from pigpio's microsecond ticks. Each ping logs the
                trigger -> echo latency (~0.2-1 ms on a real HC-SR04), the
                pulse width and the distance. An echo edge on ANOTHER sonar's
                line while only this one was triggered means crossed wiring.
4. summary      per sonar: status counts, distance statistics, verdict, and
                what to check when it failed.

Statuses: OK, NO_ECHO (never answered), NO_FALL (echo rose, never fell before
the timeout), NO_OBSTACLE (pulse longer than the range: nothing in front),
TOO_CLOSE (< 2 cm), BUSY (echo still high from the previous ping).
"""
from __future__ import annotations

import argparse
import logging
import os
import statistics
import subprocess
import sys
import threading
import time
from typing import Dict, List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SERVER = os.path.join(REPO, "Server")
# Server/ is not a package -- see the same note in gray_regression.py.
if SERVER not in sys.path:
    sys.path.insert(0, SERVER)

from config import SONAR_PINS, gpio_conflicts, gpio_owners   # noqa: E402

try:
    import pigpio
except ImportError:
    pigpio = None

SERVICE        = "main-server.service"
TRIG_US        = 10           # HC-SR04 datasheet: >= 10 us high
CM_PER_US      = 0.0343 / 2.0
MIN_CM         = 2.0          # datasheet minimum range
MAX_CM         = 400.0        # datasheet maximum range
ECHO_TIMEOUT_S = 0.25         # no obstacle: ~38 ms on a genuine HC-SR04,
                              # some clones hold ECHO high far longer
GAP_S          = 0.06         # between pings, so the last echo dies out
LATENCY_US     = (100, 3000)  # plausible trigger -> echo rise
NOISY_CM       = 2.0          # stdev above this on a still target: noisy

# BCM GPIO -> physical pin on the 40-pin header, for the wiring hints.
PHYS = {2: 3, 3: 5, 4: 7, 17: 11, 27: 13, 22: 15, 10: 19, 9: 21, 11: 23,
        0: 27, 5: 29, 6: 31, 13: 33, 19: 35, 26: 37, 14: 8, 15: 10, 18: 12,
        23: 16, 24: 18, 25: 22, 8: 24, 7: 26, 1: 28, 12: 32, 16: 36,
        20: 38, 21: 40}


def pin(gpio: int) -> str:
    return f"GPIO{gpio} (pin {PHYS.get(gpio, '?')})"
log = logging.getLogger("sonar-test")


# ---------------------------------------------------------------------------
# Edge recording
# ---------------------------------------------------------------------------
class EdgeRecorder:
    """Every level change on the given pins, with pigpio's microsecond tick.
    pigpio samples output pins too, so the trigger pulse itself is recorded."""

    def __init__(self, pi, pins: List[int]):
        self._edges: List[tuple] = []
        self._lock = threading.Lock()
        self._cbs = [pi.callback(p, pigpio.EITHER_EDGE, self._on_edge)
                     for p in pins]

    def _on_edge(self, gpio, level, tick):
        if level == 2:                     # watchdog timeout, not an edge
            return
        with self._lock:
            self._edges.append((gpio, level, tick))

    def mark(self) -> int:
        with self._lock:
            return len(self._edges)

    def since(self, index: int) -> List[tuple]:
        with self._lock:
            return list(self._edges[index:])

    def cancel(self) -> None:
        for cb in self._cbs:
            cb.cancel()


# ---------------------------------------------------------------------------
# Stage 1: environment
# ---------------------------------------------------------------------------
def service_active() -> Optional[bool]:
    try:
        out = subprocess.run(["systemctl", "is-active", SERVICE],
                             capture_output=True, text=True, timeout=3)
        return out.stdout.strip() == "active"
    except (OSError, subprocess.SubprocessError):
        return None


def check_environment(pi, tags: List[str]) -> None:
    log.info("=" * 70)
    log.info("STAGE 1 -- environment")
    log.info("=" * 70)
    log.info("pigpiod connected: version %s, Pi hardware revision 0x%x",
             pi.get_pigpio_version(), pi.get_hardware_revision())
    log.info("sonars under test: %s", ", ".join(tags))
    for tag in tags:
        trig, echo, bearing = SONAR_PINS[tag]
        log.info("  %-6s bearing %+6.1f deg  TRIG %-16s ECHO %s",
                 tag, bearing, pin(trig), pin(echo))
    conflicts = gpio_conflicts()
    if conflicts:
        for g, who in sorted(conflicts.items()):
            log.error("GPIO%d is claimed twice: %s", g, ", ".join(who))
    else:
        log.info("config.gpio_conflicts(): none (%d GPIOs assigned)",
                 len(gpio_owners()))


# ---------------------------------------------------------------------------
# Stage 2: idle levels
# ---------------------------------------------------------------------------
def check_idle(pi, tag: str) -> str:
    """'driven-low' (healthy), 'floating', or 'driven-high'."""
    trig, echo, _ = SONAR_PINS[tag]
    pi.set_mode(trig, pigpio.OUTPUT)
    pi.write(trig, 0)
    pi.set_mode(echo, pigpio.INPUT)

    pi.write(trig, 1)
    time.sleep(0.001)
    hi = pi.read(trig)
    pi.write(trig, 0)
    time.sleep(0.001)
    lo = pi.read(trig)
    if (hi, lo) == (1, 0):
        log.info("[%s] TRIG GPIO%d readback ok (wrote 1 -> read %d, "
                 "wrote 0 -> read %d)", tag, trig, hi, lo)
    else:
        log.warning("[%s] TRIG GPIO%d readback WRONG (wrote 1 -> read %d, "
                    "wrote 0 -> read %d): the line is shorted or driven by "
                    "something else", tag, trig, hi, lo)

    levels = {}
    for name, pud in (("off", pigpio.PUD_OFF), ("down", pigpio.PUD_DOWN),
                      ("up", pigpio.PUD_UP)):
        pi.set_pull_up_down(echo, pud)
        time.sleep(0.005)
        levels[name] = pi.read(echo)
    pi.set_pull_up_down(echo, pigpio.PUD_OFF)
    log.info("[%s] ECHO GPIO%d idle level: pull off=%d  pull down=%d  "
             "pull up=%d", tag, echo, levels["off"], levels["down"],
             levels["up"])

    if levels["down"] == 0 and levels["up"] == 1:
        log.error("[%s] ECHO %s is FLOATING (follows the pull resistor): "
                  "ECHO wire not connected, divider open, or the sensor has "
                  "no power / no common GND", tag, pin(echo))
        return "floating"
    if levels["down"] == 1 and levels["up"] == 1:
        log.warning("[%s] ECHO GPIO%d is driven HIGH at rest: echo stuck "
                    "(sensor mid-measurement or faulty) or wired to VCC", tag,
                    echo)
        return "driven-high"
    log.info("[%s] ECHO GPIO%d is driven LOW at rest -- as a powered HC-SR04 "
             "should be", tag, echo)
    return "driven-low"


# ---------------------------------------------------------------------------
# Stage 3: pings
# ---------------------------------------------------------------------------
def ping(pi, rec: EdgeRecorder, tag: str, n: int,
         timeout_s: float) -> Dict:
    trig, echo, _ = SONAR_PINS[tag]
    others = {SONAR_PINS[t][1]: t for t in SONAR_PINS if t != tag}
    res = {"n": n, "status": None, "cm": None, "width_us": None,
           "latency_us": None, "cross": [], "rises": 0}

    if pi.read(echo) == 1:
        log.warning("[%s] #%02d ECHO already HIGH before the trigger -- the "
                    "previous measurement has not ended", tag, n)
        end = time.monotonic() + timeout_s
        while pi.read(echo) == 1 and time.monotonic() < end:
            time.sleep(0.001)
        if pi.read(echo) == 1:
            res["status"] = "BUSY"
            log.error("[%s] #%02d BUSY: ECHO stayed high %.0f ms, ping "
                      "skipped", tag, n, timeout_s * 1000)
            return res

    idx = rec.mark()
    t_cmd = pi.get_current_tick()
    pi.gpio_trigger(trig, TRIG_US, 1)

    # Wait for this sonar's echo to rise and fall, or the timeout.
    end = time.monotonic() + timeout_s
    rise = fall = None
    while time.monotonic() < end:
        for gpio, level, tick in rec.since(idx):
            if gpio != echo:
                continue
            if level == 1 and rise is None:
                rise = tick
            elif level == 0 and rise is not None:
                fall = tick
        if fall is not None:
            break
        time.sleep(0.0005)
    time.sleep(GAP_S)                   # let stray echoes land, then judge
    edges = rec.since(idx)

    trig_ticks = [tk for g, lv, tk in edges if g == trig and lv == 1]
    t0 = trig_ticks[0] if trig_ticks else t_cmd
    for gpio, level, tick in edges:
        log.debug("[%s] #%02d   edge GPIO%-2d -> %d  at +%6d us", tag, n,
                  gpio, level, pigpio.tickDiff(t0, tick))
        if gpio in others and level == 1:
            res["cross"].append(others[gpio])
    res["rises"] = sum(1 for g, lv, _ in edges if g == echo and lv == 1)
    if not trig_ticks:
        log.debug("[%s] #%02d   trigger edge not sampled by pigpiod; "
                  "latency measured from the command instead", tag, n)

    if rise is None:
        res["status"] = "NO_ECHO"
    elif fall is None:
        res["status"] = "NO_FALL"
        res["latency_us"] = pigpio.tickDiff(t0, rise)
    else:
        width = pigpio.tickDiff(rise, fall)
        cm = width * CM_PER_US
        res.update(width_us=width, cm=cm,
                   latency_us=pigpio.tickDiff(t0, rise))
        res["status"] = ("NO_OBSTACLE" if cm > MAX_CM else
                         "TOO_CLOSE" if cm < MIN_CM else "OK")

    lat = (f"{res['latency_us']:5d} us" if res["latency_us"] is not None
           else "    --  ")
    width = (f"{res['width_us']:6d} us" if res["width_us"] is not None
             else "     --  ")
    dist = f"{res['cm']:7.1f} cm" if res["cm"] is not None else "     -- cm"
    line = (f"[{tag}] #{n:02d} {res['status']:<11} latency {lat}  "
            f"pulse {width}  {dist}")
    if res["rises"] > 1:
        line += f"  ({res['rises']} echo rises -- bouncing/noisy line)"
    if res["cross"]:
        line += f"  CROSSTALK on {', '.join(sorted(set(res['cross'])))}"
    lvl = logging.INFO if res["status"] == "OK" and not res["cross"] \
        else logging.WARNING
    log.log(lvl, line)
    if res["latency_us"] is not None and not (
            LATENCY_US[0] <= res["latency_us"] <= LATENCY_US[1]):
        log.warning("[%s] #%02d   latency %d us is outside the usual %d-%d "
                    "us", tag, n, res["latency_us"], *LATENCY_US)
    return res


# ---------------------------------------------------------------------------
# Stage 4: summary
# ---------------------------------------------------------------------------
def summarize(tag: str, idle: str, results: List[Dict]) -> bool:
    trig, echo, _ = SONAR_PINS[tag]
    counts: Dict[str, int] = {}
    for r in results:
        counts[r["status"]] = counts.get(r["status"], 0) + 1
    ok = [r["cm"] for r in results if r["status"] == "OK"]
    lats = [r["latency_us"] for r in results if r["latency_us"] is not None]
    cross = sorted({c for r in results for c in r["cross"]})

    log.info("-" * 70)
    log.info("[%s] TRIG %s / ECHO %s  idle: %s", tag, pin(trig), pin(echo),
             idle)
    log.info("[%s] statuses: %s", tag,
             ", ".join(f"{k} {v}" for k, v in sorted(counts.items())))
    if ok:
        sd = statistics.pstdev(ok) if len(ok) > 1 else 0.0
        log.info("[%s] distance: n=%d  min %.1f  median %.1f  max %.1f  "
                 "stdev %.2f cm", tag, len(ok), min(ok),
                 statistics.median(ok), max(ok), sd)
    if lats:
        log.info("[%s] trigger->echo latency: median %d us (min %d, max %d)",
                 tag, int(statistics.median(lats)), min(lats), max(lats))

    hints = []
    if idle == "floating":
        hints.append(f"ECHO {pin(echo)} floats: check the ECHO wire, the "
                     f"divider (1k from ECHO to the GPIO, 2k from the GPIO "
                     f"to GND), and that VCC=5 V and GND reach the sensor")
    if counts.get("NO_ECHO", 0) == len(results):
        hints.append(f"never answered: check VCC (5 V) and GND on the sensor, "
                     f"then the TRIG wire to {pin(trig)}; a 3.3 V trigger "
                     f"works on nearly every module")
    if counts.get("NO_FALL", 0) or counts.get("BUSY", 0):
        hints.append("ECHO rises but does not fall in time: a clone holding "
                     "ECHO long with nothing in range, or ECHO shorted high")
    if counts.get("NO_OBSTACLE", 0) and not ok:
        hints.append("answers but sees nothing in range: point it at a wall "
                     "20-100 cm away and run again")
    if cross:
        hints.append(f"triggering {tag} produced echoes on "
                     f"{', '.join(cross)}: TRIG wires swapped between those "
                     f"sonars, or they face each other / a shared wall")
    if ok and len(ok) > 1 and statistics.pstdev(ok) > NOISY_CM:
        hints.append(f"noisy (stdev > {NOISY_CM:g} cm on a still target): a "
                     f"soft or angled target, or a loose connection")

    good = (len(ok) >= 0.8 * len(results) and not cross
            and idle == "driven-low")
    if good:
        log.info("[%s] VERDICT: OK", tag)
    else:
        log.error("[%s] VERDICT: CHECK WIRING", tag)
    for h in hints:
        log.warning("[%s]   -> %s", tag, h)
    return good


# ---------------------------------------------------------------------------
# Optional modes
# ---------------------------------------------------------------------------
def live(pi, rec: EdgeRecorder, tags: List[str], timeout_s: float) -> None:
    log.info("LIVE -- round-robin pings, Ctrl+C to stop")
    root = logging.getLogger()
    quiet = root.level
    n = 0
    try:
        while True:
            n += 1
            root.setLevel(logging.ERROR)       # one summary line per round
            row = {t: ping(pi, rec, t, n, timeout_s) for t in tags}
            root.setLevel(quiet)
            parts = []
            for t in tags:
                r = row[t]
                if r["status"] == "OK":
                    bar = "#" * min(40, int(r["cm"] / 5))
                    parts.append(f"{t}:{r['cm']:6.1f} cm {bar:<40}")
                else:
                    parts.append(f"{t}:{r['status']:>9}    {'':<40}")
            log.info(" | ".join(parts))
    except KeyboardInterrupt:
        root.setLevel(quiet)
        log.info("live stopped")


def driver(pi, tags: List[str], seconds: float) -> None:
    """The same path as the server: UltrasonicPigpio, staggered like
    proc_sensors does it."""
    from ultrasonic_pigpio import UltrasonicPigpio
    ping_s = 0.06
    sensors = {}
    for i, tag in enumerate(tags):
        trig, echo, _ = SONAR_PINS[tag]
        sensors[tag] = UltrasonicPigpio(
            trigger=trig, echo=echo, pi=pi, name=tag, ping_interval=ping_s,
            start_delay=ping_s * i / len(tags))
        log.info("[%s] UltrasonicPigpio started (start delay %.0f ms)", tag,
                 ping_s * i / len(tags) * 1000)
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            log.info("driver: " + "  ".join(
                f"{t}={s.get_distance():4d} cm"
                f"{'' if s.healthy else ' (FAULTED)'}"
                for t, s in sensors.items()))
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        for s in sensors.values():
            s.stop()


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("-s", "--sensor", action="append",
                    choices=sorted(SONAR_PINS),
                    help="sonar to test (repeatable; default all)")
    ap.add_argument("-n", "--pings", type=int, default=20,
                    help="pings per sonar (default 20)")
    ap.add_argument("--timeout", type=float, default=ECHO_TIMEOUT_S,
                    help=f"echo timeout, s (default {ECHO_TIMEOUT_S:g})")
    ap.add_argument("--live", action="store_true",
                    help="after the test, ping round-robin until Ctrl+C")
    ap.add_argument("--driver", type=float, metavar="S",
                    help="after the test, read through UltrasonicPigpio for S s")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="log every raw edge too")
    ap.add_argument("--log", metavar="FILE", help="also write the log here")
    ap.add_argument("--force", action="store_true",
                    help=f"run even while {SERVICE} is active")
    args = ap.parse_args()

    handlers = [logging.StreamHandler(sys.stdout)]
    if args.log:
        handlers.append(logging.FileHandler(args.log, encoding="utf-8"))
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s.%(msecs)03d %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S", handlers=handlers)

    if pigpio is None:
        log.error("pigpio is not installed (pip install pigpio / apt install "
                  "python3-pigpio) -- run this on the Pi")
        return 2
    active = service_active()
    if active:
        log.error("%s is ACTIVE: P_sensors is pinging the same pins. Stop it "
                  "(sudo systemctl stop %s) or pass --force", SERVICE, SERVICE)
        if not args.force:
            return 2
    elif active is None:
        log.info("could not ask systemd about %s; make sure the server is "
                 "stopped", SERVICE)
    else:
        log.info("%s is not running -- good", SERVICE)

    pi = pigpio.pi()
    if not pi.connected:
        log.error("pigpiod not reachable -- start it: sudo pigpiod")
        return 2

    tags = args.sensor or list(SONAR_PINS)
    pins = sorted({p for t in SONAR_PINS for p in SONAR_PINS[t][:2]})
    rec = None
    verdicts = {}
    try:
        check_environment(pi, tags)

        log.info("=" * 70)
        log.info("STAGE 2 -- idle levels")
        log.info("=" * 70)
        # Every sonar's pins are set up, tested or not, so crosstalk onto an
        # untested one is still seen.
        idle = {t: check_idle(pi, t) for t in SONAR_PINS}
        rec = EdgeRecorder(pi, pins)

        log.info("=" * 70)
        log.info("STAGE 3 -- pings (%d per sonar, one sonar at a time)",
                 args.pings)
        log.info("=" * 70)
        results = {}
        for tag in tags:
            log.info("[%s] pinging ...", tag)
            results[tag] = [ping(pi, rec, tag, i + 1, args.timeout)
                            for i in range(args.pings)]

        log.info("=" * 70)
        log.info("STAGE 4 -- summary")
        log.info("=" * 70)
        for tag in tags:
            verdicts[tag] = summarize(tag, idle[tag], results[tag])
        log.info("-" * 70)
        log.info("RESULT: " + "  ".join(
            f"{t}={'OK' if v else 'FAIL'}" for t, v in verdicts.items()))

        if args.live:
            live(pi, rec, tags, args.timeout)
        if args.driver:
            rec.cancel()
            rec = None
            driver(pi, tags, args.driver)
    except KeyboardInterrupt:
        log.info("interrupted")
    finally:
        if rec is not None:
            rec.cancel()
        for t in SONAR_PINS:
            trig, echo, _ = SONAR_PINS[t]
            try:
                pi.write(trig, 0)
                pi.set_pull_up_down(echo, pigpio.PUD_OFF)
            except Exception:                                   # noqa: BLE001
                pass
        pi.stop()
    return 0 if verdicts and all(verdicts.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
