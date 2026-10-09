#!/usr/bin/python3
"""
Inter-process plumbing for the multiprocess robot stack.

Why processes at all
-------------------
Very little in this codebase is cpu-bound in the classic sense: the PID, the
odometry and the kinematics cost microseconds per tick. What IS expensive, and
what the control loop was losing its deadlines to, is:

  * the pigpio quadrature callbacks -- at 0.6 m/s the four motors produce on the
    order of 24k Python callback invocations per second (2340 counts/rev,
    0.204 m circumference), all of them holding the GIL;
  * ``Ultrasonic.pulseIn``, a pure Python busy-wait of up to 18 ms, five times
    per reading, in the RPi.GPIO fallback path;
  * the legacy autonomous modes, whose loops had no sleep at all;
  * the telemetry broadcaster, which was re-serialising the same 20 Hz snapshot
    500 times a second.

So the goal of the split is NOT throughput, it is determinism: give the control
loop its own GIL, its own core and a real-time scheduling class. Measure it as
control-loop jitter and dropped encoder counts, not as CPU%.

The two rules that shape everything here
----------------------------------------
1. **Queues carry events; shared memory carries state.** A queue in the encoder
   edge path would be catastrophic (24k pickles/s). A queue for "turn 90
   degrees" is exactly right. State that is latest-value-wins (yaw, distance,
   encoder totals) goes in shared arrays.

2. **Exactly one process owns each device.** The per-process singleton in
   Motor.py does nothing across a fork, so P_control is the sole owner of the
   PCA9685 (motors AND servos), P_sensors of pigpio/MPU6050/ADC, P_aux of the
   LED strip and buzzer, P_camera of Picamera2.

Staleness
---------
Inside one process, "the object exists" implies "the data is live". Across
processes it does not: a wedged or SIGKILLed producer leaves its last value in
shared memory forever, and a heading PID closing on a frozen yaw will spin the
kart until its safety timeout. Every block therefore carries a publish timestamp
and every reader checks it. Skipping that would make this refactor a safety
regression, not an improvement.
"""
from __future__ import annotations

import multiprocessing as mp
import os
import queue as _queue
import sys
import time
from typing import Dict, Iterable, Optional, Sequence

from config import CONFIG, ENCODER_TAGS, RobotConfig

# Shared-array layouts. Index constants beat magic numbers here because two
# processes have to agree on them.
ENC_TS = len(ENCODER_TAGS)          # encoder array: N totals then a timestamp

GYRO_YAW, GYRO_SAMPLE_TS, GYRO_CONNECTED, GYRO_TS = 0, 1, 2, 3
DIST_CM, DIST_GUARD, DIST_HEALTHY, DIST_TS = 0, 1, 2, 3
ADC_BATTERY, ADC_LIGHT_L, ADC_LIGHT_R, ADC_TS = 0, 1, 2, 3
LINE_L, LINE_M, LINE_R, LINE_VALID = 0, 1, 2, 3
STILL_MOVING, STILL_EVENTS, STILL_FROZEN_S, STILL_TS = 0, 1, 2, 3
HB_TS, HB_DT, HB_STEPS = 0, 1, 2

_NAN = float("nan")


# ---------------------------------------------------------------------------
# Context / process tuning
# ---------------------------------------------------------------------------
def mp_context():
    """The multiprocessing context to build everything with.

    ``fork`` is the right choice on the Pi: children inherit the already-built
    CONFIG and the shared arrays with no re-import cost, and none of our shared
    objects need to survive pickling. It also imposes the ordering constraint
    that shapes startup -- see :func:`assert_fork_safe`.
    """
    try:
        return mp.get_context("fork")
    except ValueError:              # Windows / no fork: dev box, tests only
        return mp.get_context()


def has_fork() -> bool:
    return hasattr(os, "fork")


def assert_fork_safe() -> None:
    """Warn if we are about to fork from a process that already has threads.

    Forking a threaded process copies only the calling thread but keeps every
    lock's state, so a child can deadlock on a mutex that was held at fork time
    by a thread that no longer exists. All children must therefore be spawned
    BEFORE the aiohttp loop, the telemetry threads or any Timer starts. This is
    also why P_aux is started eagerly instead of on demand.
    """
    import threading
    extra = [t for t in threading.enumerate() if t is not threading.main_thread()]
    if extra:
        names = ", ".join(t.name for t in extra)
        print(f"[ipc] WARNING: forking with live threads ({names}); "
              f"children must be spawned before any thread starts",
              file=sys.stderr)


def apply_process_tuning(name: str, *, cpu=None, nice: int = 0,
                         rt_priority: int = 0) -> None:
    """Pin and prioritise the calling process. Every step degrades gracefully.

    This is the payoff that threads could never give: SCHED_FIFO on the control
    loop means the kernel runs it the moment its period elapses, instead of
    whenever the GIL happens to come free.
    """
    if cpu is not None and hasattr(os, "sched_setaffinity"):
        cpus = {cpu} if isinstance(cpu, int) else set(cpu)
        try:
            available = set(range(os.cpu_count() or 1))
            cpus &= available
            if cpus:
                os.sched_setaffinity(0, cpus)
        except OSError as exc:
            print(f"[{name}] affinity {cpu} refused ({exc})", file=sys.stderr)

    if rt_priority > 0 and hasattr(os, "sched_setscheduler"):
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO,
                                  os.sched_param(rt_priority))  # type: ignore[attr-defined]
            return                          # RT beats nice; don't also renice
        except (OSError, PermissionError, AttributeError) as exc:
            print(f"[{name}] SCHED_FIFO {rt_priority} refused ({exc}); "
                  f"falling back to nice {nice}", file=sys.stderr)

    if nice and hasattr(os, "nice"):
        try:
            os.nice(nice)
        except OSError as exc:
            print(f"[{name}] nice {nice} refused ({exc})", file=sys.stderr)


# ---------------------------------------------------------------------------
# Queue helpers
# ---------------------------------------------------------------------------
def put_drop_oldest(q, item, drop_timeout: float = 0.02) -> bool:
    """Enqueue without ever blocking the producer for long.

    A full queue means the consumer is stalled. Blocking there would propagate a
    stalled control loop into the web loop (or a stalled web loop into the
    control loop), so the oldest item is discarded instead. For commands that is
    the right trade: the newest steering input is the one that matters.

    The retry loop is not paranoia. ``multiprocessing.Queue`` is a pipe with a
    feeder thread in front of it, so an item that has been *put* is not
    immediately *gettable*: on a queue whose semaphore says Full, ``get_nowait``
    can still raise Empty because the feeder has not flushed yet. Dropping out
    there would discard the NEW item -- exactly backwards. A brief bounded wait
    makes room instead, and only a consumer that is truly wedged loses anything.
    """
    try:
        q.put_nowait(item)
        return True
    except _queue.Full:
        pass
    except (OSError, ValueError):        # queue closed during shutdown
        return False

    for _ in range(3):
        try:
            # drop_timeout=0 asks for a strictly non-blocking drop. Callers on
            # an event loop pass that: up to 3 x 20 ms of blocking is nothing
            # on a worker thread but is a visible stall on the thread that also
            # answers WebSocket commands, which is exactly where a stalled
            # command path starts.
            if drop_timeout > 0.0:
                q.get(timeout=drop_timeout)
            else:
                q.get_nowait()           # drop oldest
        except _queue.Empty:
            pass                         # feeder still in flight; try the put
        except (OSError, ValueError):
            return False
        try:
            q.put_nowait(item)
            return True
        except _queue.Full:
            continue
        except (OSError, ValueError):
            return False
    return False


def drain(q, limit: int = 256) -> list:
    """Non-blocking: everything currently *readable*, oldest first.

    Note "readable", not "queued". ``multiprocessing.Queue`` hands items to a
    feeder thread that writes them to a pipe, so an item put microseconds ago may
    not be gettable yet and will not appear here. That is deliberately not worked
    around: the control loop must never block waiting for a command, and at 20 Hz
    a command that misses one drain is applied 50 ms later. Callers that need the
    item *now* (tests) have to poll.
    """
    items = []
    for _ in range(limit):
        try:
            items.append(q.get_nowait())
        except (_queue.Empty, OSError, ValueError):
            break
    return items


def drain_latest(q, limit: int = 64):
    """Non-blocking: only the most recent item (for latest-value-wins data)."""
    items = drain(q, limit)
    return items[-1] if items else None


# ---------------------------------------------------------------------------
# Camera frame ring
# ---------------------------------------------------------------------------
class FrameRing:
    """Fixed-slot ring of JPEG frames in shared memory.

    A ``Queue`` of frames would pickle and copy every ~30 KB frame through a
    pipe; at 20 fps with two viewers that is pure waste and it adds latency to
    the newest frame while older ones are still in flight. Here the writer
    memcpy's into a slot and bumps a sequence number, and readers copy out the
    newest slot. With ``slots`` buffers a reader has (slots-1) frame periods to
    finish its copy before the writer comes back around.

    Anonymous shared memory (RawArray) rather than ``shared_memory.SharedMemory``
    on purpose: it is inherited through the fork, so there is no name to manage,
    no /dev/shm leak and no resource-tracker interaction on shutdown.
    """

    def __init__(self, slots: int, slot_bytes: int, ctx=None):
        ctx = ctx or mp_context()
        self.slots = int(slots)
        self.slot_bytes = int(slot_bytes)
        self._buf = ctx.RawArray("B", self.slots * self.slot_bytes)
        self._meta = ctx.RawArray("q", self.slots * 2)   # (seq, length) per slot
        self._seq = ctx.RawValue("q", 0)
        self._dropped = ctx.RawValue("q", 0)
        self._torn = ctx.RawValue("q", 0)                # reads lost to a reuse
        # The condition guards the BOOKKEEPING (seq/meta) and wakes waiters. It
        # is deliberately NOT held while a reader copies a frame out -- see
        # `read` for why that difference is the whole ballgame with more than
        # one viewer.
        self._cond = ctx.Condition()
        self._view = None                                # lazily, per process

    def _mem(self):
        # memoryview, not ctypes slicing: buf[a:b] on a c_ubyte array builds a
        # list of 30k Python ints per frame, while a memoryview slice assignment
        # is one memcpy.
        #
        # .cast("B") is required, not cosmetic: a memoryview over a ctypes
        # c_ubyte array reports its format as "<B", and slice-assigning bytes to
        # a view with a byte-order-prefixed format raises
        # "NotImplementedError: memoryview: unsupported format <B". Casting
        # normalises it to plain unsigned bytes.
        if self._view is None:
            self._view = memoryview(self._buf).cast("B")
        return self._view

    # -- writer (P_camera) -------------------------------------------------
    def publish(self, data) -> bool:
        n = len(data)
        if n > self.slot_bytes:
            self._dropped.value += 1     # frame larger than a slot: raise config
            return False
        with self._cond:
            seq = self._seq.value + 1
            slot = seq % self.slots
            # Invalidate the slot BEFORE touching its bytes. A reader that is
            # mid-copy out of this slot re-checks this field afterwards (see
            # `read`); if it still held the old sequence while the bytes were
            # being overwritten, the reader would re-check against a stale
            # match and hand out a spliced frame. -1 matches no sequence.
            self._meta[2 * slot] = -1

        off = slot * self.slot_bytes
        self._mem()[off:off + n] = data

        with self._cond:
            self._meta[2 * slot] = seq
            self._meta[2 * slot + 1] = n
            self._seq.value = seq
            self._cond.notify_all()
        return True

    @property
    def torn(self) -> int:
        """Reads abandoned because the writer recycled the slot mid-copy. A
        steadily rising count means the ring is too short for how slowly
        viewers are draining it -- raise `frame_slots`."""
        return self._torn.value

    # -- readers (P_web, legacy TCP video) ---------------------------------
    @property
    def seq(self) -> int:
        return self._seq.value

    @property
    def dropped(self) -> int:
        return self._dropped.value

    def read(self, last_seq: int = 0, timeout: float = 2.0):
        """Block for a frame newer than ``last_seq``.

        Returns ``(jpeg_bytes, seq)``, or ``(None, last_seq)`` on timeout so the
        caller can re-check its own liveness (a client that went away, a camera
        that never started) instead of hanging forever.

        The copy happens OUTSIDE the lock -- a seqlock, not a mutex around the
        memcpy. The earlier version held the condition for the whole read, which
        worked with one viewer and deadlocked the web process with two: both
        readers wait on the same condition, ``notify_all`` wakes both, and
        ``wait_for`` only returns once it has REACQUIRED the lock. The two then
        take turns holding it to copy ~30 KB each while the publisher in
        P_camera blocks trying to acquire the very same lock to write the next
        frame. Those readers run in asyncio's executor, blocked in an OS
        semaphore that never yields cooperatively, so the stall propagated
        straight into the WebSocket command path.

        The protocol instead: take the bookkeeping under the lock, release,
        copy, then re-check that the slot still belongs to the sequence that was
        read. With `slots` buffers at the camera's frame rate a reader has
        (slots-1) frame periods to finish its copy, so the re-check virtually
        never fails -- and when it does, the answer is to skip that frame rather
        than to serve half of one spliced with half of another.
        """
        with self._cond:
            if self._seq.value <= last_seq:
                self._cond.wait_for(lambda: self._seq.value > last_seq,
                                    timeout=timeout)
            seq = self._seq.value
            if seq <= last_seq:
                return None, last_seq
            slot = seq % self.slots
            if self._meta[2 * slot] != seq:      # already recycled: skip it
                return None, seq
            n = self._meta[2 * slot + 1]

        # --- lock released: the publisher is free to run while we copy ---
        off = slot * self.slot_bytes
        data = bytes(self._mem()[off:off + n])

        # Did the writer lap us mid-copy? Then `data` may splice two frames.
        # Report the skip instead; the caller asks again and gets a whole one.
        if self._meta[2 * slot] != seq:
            self._torn.value += 1
            return None, seq
        return data, seq


# ---------------------------------------------------------------------------
# The shared-state bundle
# ---------------------------------------------------------------------------
class IPC:
    """Every queue and shared block, created in the parent BEFORE any fork.

    Children receive this whole object by inheritance, so nothing in it has to
    be picklable and the layout can never drift between processes.
    """

    def __init__(self, config: RobotConfig = CONFIG, ctx=None):
        ctx = ctx or mp_context()
        self.ctx = ctx
        self.config = config
        p = config.process

        # -- commands: P_web -> subsystem ---------------------------------
        self.control_q = ctx.Queue(maxsize=p.command_queue_size)
        self.sensors_q = ctx.Queue(maxsize=p.command_queue_size)
        self.aux_q = ctx.Queue(maxsize=p.command_queue_size)
        self.camera_q = ctx.Queue(maxsize=p.command_queue_size)

        # -- upstream: children -> P_web ----------------------------------
        self.telemetry_q = ctx.Queue(maxsize=p.telemetry_queue_size)
        self.event_q = ctx.Queue(maxsize=p.event_queue_size)

        # -- state: shared arrays (latest value wins) ---------------------
        # Each Array carries its own lock, which makes a whole block's publish
        # and read atomic. That matters on 32-bit userland, where a 64-bit store
        # is not atomic and a torn encoder total would read as a huge bogus
        # delta -- i.e. as a phantom lurch in the odometry.
        self.enc = ctx.Array("q", len(ENCODER_TAGS) + 1)
        self.gyro = ctx.Array("d", 4)
        self.dist = ctx.Array("d", 4)
        self.adc = ctx.Array("d", 4)
        self.line = ctx.Array("i", 4)
        self.standstill = ctx.Array("d", 4)
        self.control_hb = ctx.Array("d", 3)
        self.camera_state = ctx.Array("i", 2)      # (running, refcount)

        with self.dist:
            self.dist[DIST_CM] = _NAN
            self.dist[DIST_HEALTHY] = 1.0
        with self.gyro:
            self.gyro[GYRO_YAW] = 0.0
        with self.standstill:
            # Assume motion until P_sensors says otherwise: a consumer that saw
            # "not moving" before the detector had any data would draw exactly
            # the wrong conclusion.
            self.standstill[STILL_MOVING] = 1.0

        # -- camera frames -------------------------------------------------
        self.frames = FrameRing(p.frame_slots, p.frame_slot_bytes, ctx=ctx)

        # -- lifecycle -----------------------------------------------------
        self.stop_evt = ctx.Event()            # global: shut everything down
        self.aux_stop_evt = ctx.Event()        # per-mode: stop the legacy loops

    # -- event/log channel -------------------------------------------------
    def log(self, source: str, message: str) -> None:
        """Send a line to the parent's logger instead of printing from a child.

        Concurrent writes to one inherited stderr interleave mid-line; routing
        through the parent keeps the log readable and lets it be redirected once.
        """
        put_drop_oldest(self.event_q,
                        {"kind": "log", "source": source, "message": message,
                         "ts": time.time()})

    def event(self, kind: str, **fields) -> None:
        payload = {"kind": kind, "ts": time.time()}
        payload.update(fields)
        put_drop_oldest(self.event_q, payload)

    # -- publishers (P_sensors) -------------------------------------------
    def publish_encoders(self, totals: Dict[str, int]) -> None:
        now = time.monotonic()
        with self.enc:
            for i, tag in enumerate(ENCODER_TAGS):
                self.enc[i] = int(totals.get(tag, 0))
            self.enc[ENC_TS] = int(now * 1e9)

    def publish_gyro(self, yaw_deg: float, connected: bool,
                     sample_ts: float) -> None:
        with self.gyro:
            self.gyro[GYRO_YAW] = float(yaw_deg)
            self.gyro[GYRO_SAMPLE_TS] = float(sample_ts)
            self.gyro[GYRO_CONNECTED] = 1.0 if connected else 0.0
            self.gyro[GYRO_TS] = time.monotonic()

    def publish_distance(self, cm, guard: bool, healthy: bool) -> None:
        with self.dist:
            self.dist[DIST_CM] = _NAN if cm is None else float(cm)
            self.dist[DIST_GUARD] = 1.0 if guard else 0.0
            self.dist[DIST_HEALTHY] = 1.0 if healthy else 0.0
            self.dist[DIST_TS] = time.monotonic()

    def publish_adc(self, battery: float, light_l: float, light_r: float) -> None:
        with self.adc:
            self.adc[ADC_BATTERY] = float(battery)
            self.adc[ADC_LIGHT_L] = float(light_l)
            self.adc[ADC_LIGHT_R] = float(light_r)
            self.adc[ADC_TS] = time.monotonic()

    def publish_standstill(self, state: dict) -> None:
        """Standstill state, for telemetry only.

        Nothing in the control path reads this -- the gate is applied inside
        GyroMPU, in P_sensors, via a direct callback. This block exists so an
        operator can SEE that the yaw integration is frozen, which is otherwise
        invisible and would look identical to a dead gyro.
        """
        with self.standstill:
            self.standstill[STILL_MOVING] = 1.0 if state.get("moving") else 0.0
            self.standstill[STILL_EVENTS] = float(state.get("frozen_events", 0))
            self.standstill[STILL_FROZEN_S] = float(state.get("frozen_s", 0.0))
            self.standstill[STILL_TS] = time.monotonic()

    def read_standstill(self):
        with self.standstill:
            if self.standstill[STILL_TS] == 0.0:
                return None                     # nothing published yet
            return {"moving": self.standstill[STILL_MOVING] >= 0.5,
                    "frozen_events": int(self.standstill[STILL_EVENTS]),
                    "frozen_s": round(self.standstill[STILL_FROZEN_S], 2),
                    "age": round(time.monotonic() - self.standstill[STILL_TS], 2)}

    def publish_line(self, bits: Sequence[int]) -> None:
        with self.line:
            self.line[LINE_L], self.line[LINE_M], self.line[LINE_R] = \
                int(bits[0]), int(bits[1]), int(bits[2])
            self.line[LINE_VALID] = 1

    # -- publisher (P_control) --------------------------------------------
    def beat(self, dt: float) -> None:
        """Heartbeat for the parent's watchdog. A control loop that stops
        beating while still holding the last PWM duty is the dangerous failure
        mode this exists to catch."""
        with self.control_hb:
            self.control_hb[HB_TS] = time.monotonic()
            self.control_hb[HB_DT] = float(dt)
            self.control_hb[HB_STEPS] += 1

    # -- readers -----------------------------------------------------------
    def read_encoder_totals(self):
        with self.enc:
            return ({tag: int(self.enc[i]) for i, tag in enumerate(ENCODER_TAGS)},
                    self.enc[ENC_TS] / 1e9)

    def read_gyro(self):
        with self.gyro:
            return (self.gyro[GYRO_YAW], self.gyro[GYRO_SAMPLE_TS],
                    self.gyro[GYRO_CONNECTED] >= 0.5, self.gyro[GYRO_TS])

    def read_distance(self):
        with self.dist:
            cm = self.dist[DIST_CM]
            return (None if cm != cm else cm,          # NaN check
                    self.dist[DIST_GUARD] >= 0.5,
                    self.dist[DIST_HEALTHY] >= 0.5,
                    self.dist[DIST_TS])

    def read_adc(self):
        with self.adc:
            return (self.adc[ADC_BATTERY], self.adc[ADC_LIGHT_L],
                    self.adc[ADC_LIGHT_R], self.adc[ADC_TS])

    def read_line(self):
        with self.line:
            if not self.line[LINE_VALID]:
                return None
            return (self.line[LINE_L], self.line[LINE_M], self.line[LINE_R])

    def read_heartbeat(self):
        with self.control_hb:
            return (self.control_hb[HB_TS], self.control_hb[HB_DT],
                    int(self.control_hb[HB_STEPS]))


# ---------------------------------------------------------------------------
# Adapters: make shared memory look like the in-process objects
# ---------------------------------------------------------------------------
# These exist so DriveController does not learn about IPC at all. It keeps taking
# an "encoders" and a "gyro", keeps being constructible with the simulated ones
# in tests, and step() stays a pure function of its inputs.
# ---------------------------------------------------------------------------
class SharedEncoderReader:
    """Duck-types :class:`WheelEncoders` for a consumer in another process.

    The cross-process feedback path deliberately switched from *read-and-reset*
    to *differencing monotonic totals*. Read-and-reset would need the writer to
    take a shared lock on every single edge (24k/s), and any lost handshake
    would lose counts irrecoverably -- a silently short odometry. With totals,
    the producer only ever increments and the consumer subtracts its own
    previous copy, so a missed publication self-corrects on the next tick.
    """

    def __init__(self, ipc: IPC, sides, modes: Optional[Dict[str, dict]] = None,
                 config: RobotConfig = CONFIG):
        from config import SINGLE_PHASE_ENCODERS
        self._ipc = ipc
        self.sides = sides
        self.modes = dict(SINGLE_PHASE_ENCODERS if modes is None else modes)
        self.using_hardware = True
        self._prev: Optional[Dict[str, int]] = None
        self._stale_s = config.process.gyro_stale_s

    # lifecycle is P_sensors' job
    def begin(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def raw_totals(self) -> Dict[str, int]:
        totals, _ts = self._ipc.read_encoder_totals()
        return totals

    def read_reset_sides(self):
        from encoders import side_means
        totals, _ts = self._ipc.read_encoder_totals()
        prev, self._prev = self._prev, totals
        if prev is None:
            # First tick: adopt the current totals as the baseline rather than
            # integrating however far the kart moved before we attached.
            return 0.0, 0.0
        delta = {tag: totals.get(tag, 0) - prev.get(tag, 0) for tag in totals}
        return side_means(delta, self.sides, self.modes)

    def read_reset_detailed(self):
        left, right = self.read_reset_sides()
        return left, right, self.raw_totals()


class SharedGyroReader:
    """Duck-types :class:`GyroMPU` for P_control.

    ``is_connected`` deliberately folds in a freshness test. In one process the
    flag alone was enough, because a dead GyroMPU object could not answer at all.
    Across processes a killed P_sensors leaves the last yaw in shared memory
    forever, and turn_in_place would then close its heading PID on a frozen
    number: the kart spins until the 20 s safety timeout. Two timestamps are
    checked -- when the sensor last produced a sample (catches a thread wedged in
    an I2C read) and when P_sensors last published (catches a dead process).
    """

    def __init__(self, ipc: IPC, config: RobotConfig = CONFIG):
        self._ipc = ipc
        self._stale = config.process.gyro_stale_s
        self._warned = False

    def is_connected(self) -> bool:
        yaw, sample_ts, connected, publish_ts = self._ipc.read_gyro()
        if not connected:
            return False
        now = time.monotonic()
        fresh = (now - sample_ts) <= self._stale and \
                (now - publish_ts) <= self._stale
        if not fresh and not self._warned:
            self._warned = True
            self._ipc.log("control", "gyro data is stale; heading falls back "
                                     "to encoders")
        elif fresh:
            self._warned = False
        return fresh

    def get_yaw(self) -> float:
        return self._ipc.read_gyro()[0]

    def get_angles_gyro(self):
        return {"x": 0.0, "y": 0.0, "z": self.get_yaw()}


class SharedFrontGuard:
    """The front collision guard, as decided by P_sensors.

    Replaces ``DriveController._dist_guard`` plus the ``dist_guard_lock.locked()``
    idiom. Using a lock's held-state as a boolean never survived a process
    boundary, and it conflated "an obstacle is close" with "somebody is holding a
    mutex". The hysteresis now lives with the sensor that feeds it, and what
    crosses the boundary is one already-decided flag.

    A stale publication reads as NOT engaged: the guard only ever vetoes motion,
    so failing open keeps the kart drivable (and the operator informed through
    telemetry) rather than bricking it when P_sensors restarts. A sensor that is
    genuinely faulted already reports max range upstream.
    """

    def __init__(self, ipc: IPC, config: RobotConfig = CONFIG):
        self._ipc = ipc
        self._stale = config.process.guard_stale_s

    def engaged(self) -> bool:
        _cm, guard, _healthy, ts = self._ipc.read_distance()
        if (time.monotonic() - ts) > self._stale:
            return False
        return guard

    @property
    def distance_cm(self):
        cm, _guard, _healthy, ts = self._ipc.read_distance()
        if (time.monotonic() - ts) > self._stale:
            return None
        return cm


class RemoteMotor:
    """A ``Motor``-shaped object that forwards duties to P_control.

    P_aux runs the legacy autonomous modes (Light / Line_Tracking / Ultrasonic),
    which all drive the motors directly. They cannot construct ``Motor()``
    there -- that would put a second PCA9685 client on the same chip from a
    second process, and ``setMotorModel`` is four separate I2C register writes
    that would interleave. So they are injected with this instead, and P_control
    stays the only writer.
    """

    def __init__(self, control_q):
        self._q = control_q

    def setMotorModel(self, duty1, duty2, duty3, duty4) -> None:
        from protocol import Command
        put_drop_oldest(self._q, Command(
            name="motor",
            kwargs={"duty": [int(duty1), int(duty2), int(duty3), int(duty4)]}))

    def stop(self) -> None:
        self.setMotorModel(0, 0, 0, 0)


class RemoteServo:
    """A ``Servo``-shaped object that forwards angles to P_control (see
    :class:`RemoteMotor`: the servos are on the same PCA9685 as the motors)."""

    def __init__(self, control_q):
        self._q = control_q
        self.angles: Dict[str, int] = {}

    def setServoPwm(self, channel, angle, error=10) -> None:
        from protocol import Command
        self.angles[str(channel)] = int(angle)
        put_drop_oldest(self._q, Command(
            name="servo",
            kwargs={"channel": str(channel), "angle": int(angle)}))


class RemoteAdc:
    """An ``Adc``-shaped read of the values P_sensors already publishes, so P_aux
    does not open a second smbus client for the photoresistors."""

    def __init__(self, ipc: IPC):
        self._ipc = ipc

    def recvADC(self, channel: int) -> float:
        battery, light_l, light_r, _ts = self._ipc.read_adc()
        if channel == 0:
            return light_l
        if channel == 1:
            return light_r
        if channel == 2:
            return battery / 5.0        # Adc.recvADC(2)*5 is the battery volts
        return 0.0


# ---------------------------------------------------------------------------
# Emergency stop
# ---------------------------------------------------------------------------
def emergency_motor_stop() -> bool:
    """Zero every motor channel by talking to the PCA9685 directly.

    Last-resort path for the supervisor, for the case that makes a crashed
    control process dangerous: the PCA9685 latches its last duty in hardware, so
    a SIGKILLed P_control leaves the kart driving at whatever it was doing. The
    "one writer per device" rule is broken here on purpose, and only once
    P_control is confirmed dead -- at which point there is provably no other
    writer.

    Deliberately does not import Motor: setPWMFreq would re-run the init
    sequence, and we want the narrowest possible write.
    """
    try:
        from PCA9685 import PCA9685
        pwm = PCA9685(0x40, debug=False)
        for channel in range(8):            # the four motors' H-bridge inputs
            pwm.setMotorPwm(channel, 4095)  # both inputs high = brake, as
        return True                         # Motor.setMotorModel(0,...) does
    except Exception as exc:                                    # noqa: BLE001
        print(f"[supervisor] emergency motor stop FAILED: {exc}",
              file=sys.stderr)
        return False
