# Lafvin PiKart — Server

Web-controlled 4-wheel **skid-steer** robot car running on a Raspberry Pi.
`web.py` is the entry point: it serves a mobile control page over HTTP, streams
the camera as MJPEG, and exchanges commands + telemetry over a WebSocket.

This document describes the **refactor**: a hardware-independent control stack
(PID / kinematics / odometry / encoders), an extensible command protocol, and a
clean split between pure logic and hardware so the maths can be unit-tested on
any machine — no Pi required.

---

## Status

| Area | State |
|------|-------|
| Control stack (config, PID, kinematics, odometry, encoders, drive controller) | ✅ implemented + unit-tested off-Pi |
| Wire protocol (JSON + legacy, command router, telemetry) | ✅ implemented + unit-tested |
| Multiprocess split (`ipc.py`, `supervisor.py`, `proc_*.py`) | ✅ implemented + unit-tested off-Pi |
| Unit tests (`tests/`) | 92 tests; 87 pass, 5 pre-existing `TestGyroHeadingTurn` failures |
| `server.py` facade → queues → per-process appliers | ✅ done |
| `web.py` telemetry broadcast + `/health` + shared-ring MJPEG | ✅ done |
| `static/index.html` odometry display + closed-loop toggle | ✅ unchanged (telemetry keys preserved) |

> ⚠️ **Not yet run on the robot.** Everything byte-compiles, the hardware-free
> stack is unit-tested, and `proc_control` + `proc_sensors` have been executed
> end-to-end against stubbed hardware (queues, shared memory, telemetry envelope,
> heartbeat, command routing and shutdown all verified). But `fork()`, pigpio,
> real I2C timing, `picamera2` and the SCHED_FIFO/affinity calls can only be
> exercised on the Pi. **Put the kart on blocks for the first run.**

> ⚠️ The 5 failing `TestGyroHeadingTurn` tests **pre-date this work** — the
> heading gains are tuned on hardware and the simulated plant does not reach the
> target. The failure set is identical before and after the refactor.

---

## Architecture

```
```
                                    P_web  (parent, aiohttp)
   browser ◀── WS / MJPEG ──▶ ┌───────────────────────────────┐
                              │ web.py   HTTP + WS + video    │
                              │ server.py Server facade       │
                              │   parse -> route -> enqueue   │
                              │ supervisor.py  watchdog       │
                              └──┬────────┬─────────┬─────────┘
            control_q / telemetry_q│  sensors_q│   camera_q│ aux_q
                   ┌──────────────▼──┐  ┌──────▼───────┐  ┌▼──────────────┐
                   │   P_control     │  │  P_sensors   │  │ P_camera      │
                   │ DriveController │  │ encoders     │  │ Picamera2     │
                   │  .step() 20 Hz  │  │  (pigpio)    │  │ JpegEncoder   │
                   │ Motor + Servo   │  │ GyroMPU      │  │  -> FrameRing │
                   │  (PCA9685: SOLE │  │ Ultrasonic   │  └───────────────┘
                   │   PWM owner)    │  │  + guard     │  ┌───────────────┐
                   │ SCHED_FIFO,     │  │ ADC, IR line │  │ P_aux (nice+10)│
                   │  own core       │  │              │  │ Led, Buzzer   │
                   └────────▲────────┘  └──────┬───────┘  │ legacy modes  │
                            │                  │          └───────┬───────┘
                            └── shared memory ─┘                  │
                               enc totals, yaw,          RemoteMotor/RemoteServo
                               distance+guard, adc        (duties -> control_q)
```

**Queues carry events; shared memory carries state.** A queue in the encoder edge
path would mean pickling ~24k times a second; a queue for "turn 90 degrees" is
exactly right. Latest-value-wins signals (yaw, distance, encoder totals) live in
shared arrays, each with a publish timestamp.

### Why processes

Almost nothing here is cpu-bound in the classic sense — the PID, the odometry and
the kinematics cost microseconds per tick. What the control loop was losing its
deadlines to was GIL contention:

| Load | Nature |
|------|--------|
| pigpio quadrature callbacks | ~24k Python callbacks/s at 0.6 m/s, all holding the GIL |
| `Ultrasonic.pulseIn` (RPi.GPIO fallback) | pure Python busy-wait, up to 18 ms × 5 per reading |
| `Line_Tracking.run` / `Light.run` | `while True` with **no sleep** — a pinned core per active mode |
| telemetry broadcaster | 500 Hz `json.dumps` of a snapshot that changes at 20 Hz |
| LED animations | numpy + SPI transfer per frame, in tight loops |

So the goal of the split is **determinism, not throughput**. Judge it by
control-loop jitter and dropped encoder counts, not by CPU%. The 500 Hz
telemetry rate and the sleepless loops were fixed outright; the rest is
isolation.

### Process model rules

1. **One owner per device.** `Motor`'s singleton is per-process and does nothing
   across a fork, so P_control alone constructs `Motor`/`Servo` (both on the same
   PCA9685). P_aux's legacy modes reach them through `ipc.RemoteMotor` /
   `RemoteServo`.
2. **Hardware handles open in the child, after the fork.** Inheriting a pigpio
   socket or an smbus fd gives two processes one connection.
3. **Fork before any thread.** A forked child inherits every lock in whatever
   state it was in, so it can deadlock on a mutex whose owner does not exist in
   it. `Server.__init__` forks first and starts threads after;
   `ipc.assert_fork_safe()` warns if that order is ever broken. This is also why
   P_aux is started eagerly rather than on demand.
4. **Every shared-memory reader checks staleness.** In one process "the object
   exists" implied "the data is live". It no longer does: a SIGKILLed P_sensors
   leaves its last yaw in shared memory forever, and a heading PID closing on a
   frozen yaw spins until its safety timeout. See `ipc.SharedGyroReader`.
5. **P_web imports no hardware module at all** — verify with
   `tests/test_wiring.py` and the import check in its docstring.

### Failure modes the split introduces

Partial death is new: P_control can die while the UI still looks healthy, and the
PCA9685 **latches its last duty in hardware**, so a kart that was driving forward
keeps driving forward. `supervisor.py` escalates:

| Condition | Action |
|-----------|--------|
| heartbeat older than `control_heartbeat_s` (1.0 s) | log it |
| P_control not alive | stop the motors directly (`ipc.emergency_motor_stop`) |
| heartbeat older than `control_kill_after_s` (2.0 s) | SIGTERM (its handler brakes), then stop the motors as a backstop |

`GET /health` reports per-process pids, liveness and heartbeat age. Telemetry
carries `drive.stale`, so a UI can tell "stopped" from "not reporting".

### Design principles applied by the refactor
- **Hardware behind a fallback.** Encoders use **pigpio**; when pigpio/pigpiod
  is unavailable (laptop/CI) `WheelEncoders` transparently falls back to
  `SimulatedEncoder`, so every pure-logic module imports and runs anywhere.
- **Dependency injection.** `DriveController` receives its `motor` and
  `encoders` instead of constructing them, so the same object runs against real
  hardware or a `SimulatedDrivePlant`.
- **No import-time side effects, anywhere.** The legacy modules used to build
  hardware at import (`PWM = Motor()`, `led = Led()`, `ultrasonic = Ultrasonic()`,
  `infrared = Line_Tracking()`, and Buzzer's GPIO/PWM setup), which made them
  unimportable from any process that is not that device's owner. All lazy now —
  this was the prerequisite for splitting processes at all.
- **Cooperative stops only.** `Thread.stop_thread()` (ctypes-injecting
  `SystemExit` seven times into a running thread) is gone; it could land
  mid-I2C-transaction. Loops take a stop `Event`; subsystems take
  `Process.terminate()`, whose SIGTERM handler brakes first.
- **One place to tune.** All geometry, gains, pins and ports live in
  `config.py`.
- **Extensible control surface.** New commands are a `router.register(...)`
  call, not another branch in a 150-line `if/elif`.

---

## New modules

| File | Responsibility |
|------|----------------|
| `config.py` | Dataclasses for wheel geometry, PID gains, encoder pins, motor channels, side mapping, control/network settings. Exposes a ready `CONFIG`. |
| `pid.py` | Reusable, time-aware `PID` with anti-windup, output clamp, feed-forward, `reset()`. |
| `kinematics.py` | `SkidSteerKinematics` forward/inverse, `Twist`, `WheelSpeeds`. |
| `odometry.py` | `SkidSteerOdometry` pose integration, `Pose`, `wrap_angle`. |
| `encoders.py` | pigpio quadrature `Encoder` (x4, glitch-filtered), `SimulatedEncoder`, `WheelEncoders` per-side aggregator + raw-count diagnostics. |
| `drive_controller.py` | `DriveController` — velocity + position PID loops, `SimulatedDrivePlant`. |
| `protocol.py` | `parse()`, `Command`, `CommandRouter`, telemetry/sensor JSON builders. |
| `ipc.py` | The process boundary: bounded queues, shared-state blocks, `FrameRing`, the `SharedEncoderReader`/`SharedGyroReader`/`SharedFrontGuard` adapters that duck-type the in-process objects, `RemoteMotor`/`RemoteServo`/`RemoteAdc`, `emergency_motor_stop`. |
| `supervisor.py` | Forks the children, watchdogs the control heartbeat, drains their logs, fails the motors safe. |
| `proc_control.py` | P_control body: `ControlApplier` (the receiving half of the command router) + `DriveController.run_loop` under SCHED_FIFO. |
| `proc_sensors.py` | P_sensors body: encoders, gyro, `FrontGuardMonitor` (was `DriveController._dist_guard`), `AdcMonitor`, IR line. |
| `proc_camera.py` | P_camera body: Picamera2 → `RingOutput` → `FrameRing` (replaces `server.StreamingOutput`). |
| `proc_aux.py` | P_aux body: LED animations, buzzer, legacy autonomous modes, all cooperatively stoppable. |
| `tests/test_core.py` | Unit tests for the control stack. |
| `tests/test_ipc.py` | Encoder-total differencing, staleness contracts, guard hysteresis, frame ring, queue policy, and a closed-loop run whose feedback crosses shared memory. |
| `tests/test_wiring.py` | Every command P_web enqueues has a handler on the receiving side (read from server.py's AST), and vice versa — the silent-failure mode of a queue design. |

---

## Encoder → Odometry → PID

The two left wheels move as one virtual left wheel, the two right wheels as one
virtual right wheel, so the platform is a differential drive with track width
equal to the lateral spacing between sides.

**Wheel geometry** (`config.WheelGeometry`, from the reference `TiredWheel`):
`diameter = 0.065 m`, `track = 0.151 m`. Distance per encoder count is
`π·diameter / counts_per_rev`.
> ⚠️ `counts_per_rev` defaults to `2340` (13 PPR × 45:1 × 4) — **calibrate it** for
> your motors (spin one wheel exactly N turns, read the count).

**Quadrature decoding** (`encoders.py`, **pigpio**). Both edges of both phases
are counted (x4). The transition delta is looked up by
`(prev_state << 2) | new_state` where `state = (A << 1) | B`:

```
        new →  00   01   10   11
   prev 00:     0,  -1,   1,   0
   prev 01:     1,   0,   0,  -1
   prev 10:    -1,   0,   0,   1
   prev 11:     0,   1,  -1,   0
```

Counting is serviced by the **pigpio daemon (pigpiod)** in C. 
This matters: at 2340 counts/rev (13 PPR × 45:1 × 4) the edge rate
at speed is far too high for RPi.GPIO's Python callbacks, which silently under-count. 
Each phase gets a 100 µs hardware glitch filter for
debounce. Off-Pi (no pigpio/pigpiod), `WheelEncoders` falls back to
`SimulatedEncoder` so the stack still imports and the tests run.

> Requires the daemon:  **`sudo pigpiod`** (start on boot with
> `sudo systemctl enable pigpiod`).

Each encoder also keeps a **lifetime raw count** exposed in telemetry
(`drive.encoders`) — the ground truth for calibrating signs and
`counts_per_rev` (see *Calibration*).

**Odometry** (`odometry.py`) integrates per-side distance deltas — identical
update to the reference, but using the **midpoint heading** for better arcs:

```
d_center = (d_right + d_left) / 2
d_theta  = (d_right - d_left) / track
mid      = theta + d_theta/2
x     += d_center · cos(mid)
y     += d_center · sin(mid)
theta  = wrap(theta + d_theta)
```

**Inverse kinematics** (`kinematics.py`) turns a commanded body twist into
target side speeds:

```
v_left  = v − w · track/2
v_right = v + w · track/2
```

**Two control modes** (`drive_controller.py`), once per control tick:

*Velocity mode* (teleop / `drive`): inverse-kinematics the target `Twist` →
per-side target speed → per-side velocity `PID` (+feed-forward) → duty.

*Position mode* (`drive_distance` / `turn`): the move sets a per-side **distance
target** and a per-side **position PID** drives duty from the *distance* error
(error in metres, not m/s). Both sides share the same target for a straight
line (so they stay equal → straight) or equal-and-opposite for a turn. The move
finishes when both sides are within `tolerance` and stopped. This is the
"PID on distance, not velocity" approach — more robust for point-to-point moves
because it doesn't rely on differentiating encoder counts into a velocity.

Both modes then `Motor.setMotorModel(left, left, right, right)` and publish a
telemetry snapshot (pose, twist, wheel speeds, duties, raw encoder counts, and
— during a move — per-side target/traveled/remaining).

**Saturation & anti-windup** (`pid.py`). The output is clamped to
`±output_limit` (4095, the Motor duty range — `Motor.duty_range` clamps again,
so it's defense-in-depth). Integral windup is handled two ways:
- **Conditional integration** — while the output is saturated *and* the error
  would drive it further into saturation, integration is frozen, so the
  integrator can't run away while the actuator is pinned. When the error
  reverses, integration resumes so the term can unwind.
- **Integral-term clamp** — `integral_limit` bounds the *contribution*
  `ki·integral` (in duty units, not the raw accumulator), reflected back into
  the stored state as a backstop.

**High-level moves** (`drive_distance`, `turn_in_place`) use the position PID
above: each side servos to its distance target and stops within `tolerance`.
In the slip-free simulated plant, `drive_distance(3.0)` stops within ~1 cm.
> Real-world accuracy depends on **`counts_per_rev` calibration**, correct
> **encoder signs**, and — for skid-steer — **wheel slip** (worst in turns).
> A 1 cm/side arrival tolerance on the 15 cm track maps to ~0.13 rad of heading,
> so treat turn angles as approximate; precise turns want a heading sensor.

## Calibration (do this first, on the robot)

Everything downstream assumes the encoders read *forward → positive, equal on
both sides*. Verify with the raw counts in telemetry:

1. Start the daemon: `sudo pigpiod`, then run the server.
2. Watch raw counts (`python ws_probe.py 8` reads `drive.encoders`), or the
   `/status`/WebSocket telemetry.
3. **Push the robot straight forward by hand ~1 m.** Every one of M1–M4 should
   increase (positive). If a motor goes **negative**, set its entry in
   `SideMapping.signs` (`config.py`) to `-1`. If a whole side is negative, flip
   both of that side's signs.
4. **Calibrate `counts_per_rev`:** spin one wheel exactly N full turns by hand
   and read that motor's raw count; `counts_per_rev = raw_count / N`. (For the
   13 PPR × 45:1 motor with x4 decoding that's 13·45·4 = **2340**.)
5. Re-check: a hand-pushed 1 m forward should now read pose `x ≈ 1.0`,
   `y ≈ 0`, `θ ≈ 0`. Only then run closed-loop moves.

> ⚠️ Do **not** run `drive_distance` before this: a position loop with an
> under-counting encoder keeps driving until it *thinks* it arrived, so it
> overshoots badly.

Engagement: the controller is **released** by default (odometry keeps running,
motors untouched so raw `CMD_MOTOR` duty still works). A velocity command
**engages** it (PID takes over). A command timeout (`command_timeout`) forces a
safety stop while engaged.

`SimulatedDrivePlant` is a first-order motor model that feeds the simulated
encoders, letting the *entire* loop close on a laptop — that's what the drive
tests exercise.

---

## WebSocket protocol

Both encodings are accepted on the same socket; replies/telemetry are JSON.

### Client → robot (commands)
| JSON | Legacy text | Effect |
|------|-------------|--------|
| `{"type":"drive","linear":0.3,"angular":0.5}` | — | closed-loop velocity (m/s, rad/s) |
| `{"type":"drive_distance","distance":3.0,"speed":0.3}` | — | drive straight N m (odometry-closed) and stop |
| `{"type":"turn","angle":90,"speed":1.0}` | — | turn in place N° (odometry-closed) and stop |
| `{"type":"motor","duty":[f,b,f,b]}` | `CMD_MOTOR#f#b#f#b` | raw skid duty (bypasses PID) |
| `{"type":"mecanum",...}` | `CMD_M_MOTOR#a#m#a#m` | legacy mecanum mix |
| `{"type":"servo","channel":"0","angle":90}` | `CMD_SERVO#0#90` | pan/tilt |
| `{"type":"led","index":255,"r":..,"g":..,"b":..}` | `CMD_LED#255#r#g#b` | LEDs |
| `{"type":"led_mode","mode":"2"}` | `CMD_LED_MOD#2` | LED animation |
| `{"type":"buzzer","on":true}` | `CMD_BUZZER#1` | buzzer |
| `{"type":"mode","mode":"one"}` | `CMD_MODE#one` | switch autonomous mode |
| `{"type":"reset_odometry"}` | — | zero the pose |
| `{"type":"power"}` | `CMD_POWER` | request battery |

### Robot → client (telemetry, JSON lines)
```json
{"type":"telemetry","ts":1720.5,"battery":7.9,"mode":"one",
 "drive":{"pose":{"x":0.42,"y":0.01,"theta":0.05,"theta_deg":2.9},
          "twist":{"linear":0.20,"angular":0.01},
          "wheel_speed":{"left":0.20,"right":0.20},
          "duty":{"left":2600,"right":2610},"engaged":true}}
```
Plus `{"type":"sensor","sensor":"ultrasonic","value":42}` etc.

> The legacy Android TCP client keeps its `CMD_*#...` text protocol (unchanged);
> the WebSocket path uses JSON.

---

## Configuration & calibration

Everything tunable is in `config.py`. Common knobs:

- `WheelGeometry.counts_per_rev` — **calibrate first** (see above).
- `SideMapping.signs` — flip a `-1`/`+1` if a wheel counts backwards, or
  `left`/`right` tag groups if a side is mirrored. No logic changes needed.
- `PIDGains` — velocity loop: `kp/ki/kd`, `feedforward` (duty to hold 1 m/s),
  `output_limit` (saturation, ±4095) and `integral_limit` (bounds the
  `ki·integral` contribution, in duty units).
- `PositionGains` — distance loop (`drive_distance`/`turn`): `kp` (duty per
  metre), `kd` (damping), `output_limit` (gentle move-speed cap), `tolerance`
  (arrival, m), `stop_speed`, `max_time` (safety timeout). **Tune on hardware.**
  Note: `ki` defaults to 0; if moves stall short due to stiction, add a small
  `ki` (with the built-in anti-windup) to close the last bit.
- `ControlConfig` — `loop_hz`, `telemetry_hz`, `command_timeout`,
  `max_linear`, `max_angular`.
- `NetworkConfig` — ports and the network interface name.

---

## Testing

The control/protocol stack is fully testable without a Pi (uses the GPIO mock
and the simulated plant):

```bash
cd Server
python -m unittest discover -s tests -v
```

Coverage (`tests/test_core.py`):
- **PID** — output sign, saturation clamp, feed-forward, integral-term clamp,
  **no windup during saturation** (unreachable setpoint pins the output, then
  the setpoint drops and the output must recover immediately), and that the
  integrator still unwinds when the error reverses.
- **Kinematics** — forward/inverse round-trip, straight, spin-in-place.
- **Odometry** — straight line, spin-in-place, and an **arc compared against
  the analytic circle** `R = v/w`.
- **Encoders** — x4 direction decoding, side aggregation with sign correction.
- **DriveController** — closed-loop forward velocity **converges** to target,
  spin changes heading, `release()` stops actuation.
- **Protocol** — legacy/JSON parsing, garbage rejection, router dispatch,
  telemetry serialisation.

Coverage (`tests/test_ipc.py`) — the code the process split added:
- **Encoder totals differencing** — the first read adopts a baseline instead of
  integrating everything P_sensors counted before P_control attached; deltas match
  the in-process aggregation exactly; a **skipped publication self-corrects**
  (the reason totals beat a cross-process read-and-reset handshake); the
  single-phase M3 rule survives the hop.
- **Staleness contracts** — a stale *sample* timestamp (sensor thread wedged in an
  I2C read) and a stale *publish* timestamp (P_sensors dead) both read as
  disconnected; the front guard **fails open**, so a dead sensor process cannot
  brick the kart.
- **Front guard hysteresis / TTL**, lifted out of `DriveController._dist_guard`,
  including that the 255 "no echo" sentinel is not mistaken for a clear road and
  that the *reported* distance expires while the guard's history does not.
- **Frame ring** — roundtrip, newest-frame-wins for a slow viewer, independent
  cursors, timeout, oversize drop.
- **Closed loop across shared memory** — the real controller converging with its
  feedback arriving only through shared arrays. This is what would catch a sign,
  scale or differencing error in the new feedback path.

Coverage (`tests/test_wiring.py`) — the silent-failure mode of a queue design:
every command name `server.py` enqueues (read from its AST) has a handler on the
receiving side, and every handler is reachable; plus the applier driving a real
`DriveController` through a real queue, and the ingress-timestamp dead-man logic.

---

## Running (on the Pi)

```bash
sudo pigpiod                     # start the GPIO daemon (needed for encoders)
sudo python3 web.py              # web only (port 8080)
sudo python3 web.py --with-tcp   # web + legacy TCP (5000/8000) + power monitor
sudo python3 web.py --no-camera  # skip the camera process
```

`sudo` matters for more than GPIO now: `SCHED_FIFO` on the control loop and the
negative `nice` values need it. Without root they degrade (the log says which)
and you lose the determinism the split was for.

Each child prints its pid on startup. Check the split is live with:

```bash
curl -s localhost:8080/health
top -H -p $(pgrep -d, -f web.py)
```

**What to measure**, before and after — this is a determinism change, so CPU% is
the wrong metric:
- control-loop jitter: `control_loop_dt` from `/health` against 1/`loop_hz`;
- dropped encoder counts: drive a known distance and compare the raw totals in
  `drive.encoders` against `counts_per_rev`;
- `vcgencmd get_throttled` — spreading load over 4 cores raises total power, and
  the 3B+ throttles at 60 °C.

Individual processes run standalone for bring-up:

```bash
sudo python3 proc_sensors.py     # publish sensors and print them, nothing else
sudo python3 server.py           # bring the whole stack up, print telemetry
```

Dependencies: `aiohttp`, `pigpio` (+ `sudo pigpiod` running), `picamera2`,
`smbus`, `RPi.GPIO`, `rpi_ws281x`. Linux only — the stack needs `fork()`.

---

## Bugs fixed in this refactor

- **`Motor` is now a singleton.** `Motor.py`, `Ultrasonic.py`,
  `Line_Tracking.py` etc. each did `Motor()`, creating several `PCA9685`/`Adc`
  objects for one physical board and re-running `setPWMFreq`. `Motor.__new__`
  now returns one shared instance.
- **`Motor.Rotate()`** used the module-global `PWM` instead of `self`, and
  `bat_compensate` could divide by zero on a 0 ADC reading. Both fixed; it now
  also stops **cooperatively** (`stop_rotate()` / `Event`) instead of relying
  on `Thread.stop_thread()` injecting an async exception.
- **`/video` camera ref-leak.** `video_handler` acquired the camera on every
  request but never released it; it now releases in a `finally`.
- **God-object dispatch replaced** by a `CommandRouter` registry.

## Bugs found and fixed while splitting the processes

- **`telemetry_hz` was 500.0** for a snapshot that only changes at `loop_hz` (20):
  500 `json.dumps` + WebSocket sends per second, on a core the control loop
  needed. Now 20. This one change may be worth more than the rest of the split.
- **Three self-rearming `threading.Timer` chains** (`sendUltrasonic`, `sendLight`,
  `sendLine`) created a **brand new thread every 0.17–0.23 s** each, for as long
  as their sensor was enabled. Replaced by one publisher reading shared memory.
- **`Line_Tracking.run` and `Light.run` had no sleep at all** — a pinned core per
  active mode.
- **`main_UI.py` headless mode was `while True: pass`** — another pinned core.
- **`main_UI.close()` called `os._exit(0)`**, which with the process split would
  orphan the children — and the PCA9685 latches its last duty, so that leaves the
  kart driving. It shuts down properly first now.
- **`FrameRing` memoryview format.** A `memoryview` over a ctypes `c_ubyte` array
  reports its format as `"<B"`, and slice-assigning `bytes` to it raises
  `NotImplementedError: memoryview: unsupported format <B`. Caught by
  `tests/test_ipc.py`; it would have broken the camera on the Pi. Fixed with
  `.cast("B")`.
- **`put_drop_oldest` dropped the *newest* item** under burst load.
  `multiprocessing.Queue` is a pipe with a feeder thread, so on a queue whose
  semaphore says Full, `get_nowait()` can still raise Empty — and the first
  version gave up there, discarding the item it was trying to add. Now retries
  with a bounded wait.
- **A dead `mecanum` handler** in `ControlApplier`: the joystick mix is computed
  in P_web and crosses the queue as plain duties, so nothing could reach it.
  Caught by `tests/test_wiring.py`.

## Remaining / by design

- **`command_timeout` (0.1 s) now spans the queue hop.** Commands are stamped at
  ingress in P_web and that stamp is credited in `_apply_target`, so the dead-man
  switch measures the age of the operator's input rather than restarting on
  arrival. An implausible stamp (clock skew) falls back to "now" rather than
  wedging teleop.
- **A raw `CMD_MOTOR` duty has no dead-man timeout** — unchanged behaviour, and
  the web UI depends on it (one "forward" press must hold). It does mean a P_aux
  crash mid-mode leaves the last duty applied; `AuxWorker._guarded` zeroes the
  motors when a mode thread dies, which covers the likely case but not SIGKILL.
- **`emergency_motor_stop` deliberately breaks the one-writer rule**, writing the
  PCA9685 from the supervisor — but only once P_control is confirmed dead, at
  which point there is provably no other writer.
- **No auto-restart of a dead child.** Re-forking from a threaded parent is the
  hazard rule 3 exists to avoid, so the supervisor fails safe and reports
  `degraded` instead. Restart the service.
- **`Scripts/gray_regression.py` still needs the server stopped first** — it owns
  the encoders directly, and it now contends with P_sensors' pigpio callbacks
  rather than P_control's. Its existing warning still applies.
- **On-device verification pending** — see the *Status* note.
