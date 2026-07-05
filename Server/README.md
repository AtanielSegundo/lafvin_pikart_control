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
| Unit tests (`tests/test_core.py`) | ✅ 23 tests, all passing (see *Testing*) |
| `server.py` dispatch → `CommandRouter` + drive controller | ✅ done |
| `web.py` telemetry broadcast + JSON WS handling | ✅ done |
| `static/index.html` odometry display + closed-loop toggle | ✅ done |
| Motor singleton + `Rotate()` bug fixes | ✅ done |

> ⚠️ The `server.py` / `web.py` integration was written and byte-compiles
> cleanly, but could **not** be executed here — it imports Pi-only libraries
> (`RPi.GPIO`, `picamera2`, `smbus`). Run it on the Pi to verify end-to-end.
> The hardware-independent stack (everything the tests cover) is verified.

---

## Architecture

```
                       ┌──────────────┐
   browser  ◀── WS ──▶ │   web.py     │  aiohttp: HTTP + WebSocket + MJPEG
                       └──────┬───────┘
                              │ dispatch(raw) / telemetry()
                       ┌──────▼───────┐
                       │  protocol    │  parse (JSON | legacy) + CommandRouter
                       └──────┬───────┘
                              │ registered handlers
                       ┌──────▼───────────────────────────┐
                       │   Server (facade)                 │  peripherals + modes
                       └──┬───────────────┬────────────┬───┘
                          │               │            │
                 ┌────────▼──────┐  ┌─────▼─────┐  ┌───▼────┐
                 │DriveController│  │  Servo    │  │  Led   │ ...
                 └──┬────────┬───┘  └───────────┘  └────────┘
        inverse kin │        │ PID + feedforward
             ┌──────▼─┐   ┌──▼────────┐
             │odometry│   │  Motor    │  PCA9685 PWM
             └────▲───┘   └───────────┘
                  │ count deltas
             ┌────┴─────────┐
             │ WheelEncoders│  quadrature x4 (GPIO edge IRQ) / simulated
             └──────────────┘
```

### Design principles applied by the refactor
- **Hardware behind a shim.** `gpio_backend.py` returns real `RPi.GPIO` on the
  Pi and a no-op mock elsewhere, so every pure-logic module imports and runs on
  a laptop/CI.
- **Dependency injection.** `DriveController` receives its `motor` and
  `encoders` instead of constructing them, so the same object runs against real
  hardware or a `SimulatedDrivePlant`.
- **No import-time side effects** in the new modules (the legacy modules still
  instantiate hardware at import — see *Known issues*).
- **One place to tune.** All geometry, gains, pins and ports live in
  `config.py`.
- **Extensible control surface.** New commands are a `router.register(...)`
  call, not another branch in a 150-line `if/elif`.

---

## New modules

| File | Responsibility |
|------|----------------|
| `config.py` | Dataclasses for wheel geometry, PID gains, encoder pins, motor channels, side mapping, control/network settings. Exposes a ready `CONFIG`. |
| `gpio_backend.py` | `GPIO` + `ON_HARDWARE` — real `RPi.GPIO` or mock fallback. |
| `pid.py` | Reusable, time-aware `PID` with anti-windup, output clamp, feed-forward, `reset()`. |
| `kinematics.py` | `SkidSteerKinematics` forward/inverse, `Twist`, `WheelSpeeds`. |
| `odometry.py` | `SkidSteerOdometry` pose integration, `Pose`, `wrap_angle`. |
| `encoders.py` | Quadrature `Encoder` (IRQ-driven, x4), `SimulatedEncoder`, `WheelEncoders` per-side aggregator. |
| `drive_controller.py` | `DriveController` closed loop + `SimulatedDrivePlant`. |
| `protocol.py` | `parse()`, `Command`, `CommandRouter`, telemetry/sensor JSON builders. |
| `tests/test_core.py` | Unit tests for all of the above. |

---

## Encoder → Odometry → PID (the skid-steer model)

The kinematics mirror the CoppeliaSim `DiffDrive` reference used for analysis.
The two left wheels move as one virtual left wheel, the two right wheels as one
virtual right wheel, so the platform is a differential drive with track width
equal to the lateral spacing between sides.

**Wheel geometry** (`config.WheelGeometry`, from the reference `TiredWheel`):
`diameter = 0.065 m`, `track = 0.151 m`. Distance per encoder count is
`π·diameter / counts_per_rev`.
> ⚠️ `counts_per_rev` defaults to `1560` as a placeholder — **calibrate it** for
> your motors (spin one wheel exactly N turns, read the count).

**Quadrature decoding** (`encoders.py`). Both edges of both phases are counted
(x4). The transition delta is looked up by `(prev_state << 2) | new_state` where
`state = (A << 1) | B`:

```
        new →  00   01   10   11
   prev 00:     0,  -1,   1,   0
   prev 01:     1,   0,   0,  -1
   prev 10:    -1,   0,   0,   1
   prev 11:     0,   1,  -1,   0
```

Counting is interrupt-driven on the Pi (GPIO `add_event_detect`). Off-Pi,
`SimulatedEncoder` accepts injected ticks so tests are deterministic.

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

**Closed loop** (`drive_controller.py`), once per control tick (default 50 Hz):
1. read encoder count deltas → distances → update odometry + measured speeds;
2. inverse-kinematics the target `Twist` → target side speeds;
3. per-side `PID` (with static feed-forward) → PWM duty;
4. `Motor.setMotorModel(left, left, right, right)`;
5. publish a telemetry snapshot (pose, twist, wheel speeds/targets, duties).

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

**High-level moves** (`drive_distance`, `turn_in_place`). These are motion
*goals* the control loop services each tick, closed on the odometry: it drives
the target twist, tapers speed near the end to limit overshoot (a heading term
holds the line straight), and stops itself when the measured travel reaches the
target. In the simulated plant, `drive_distance(3.0)` stops within ~0.5 cm.
> Real-world accuracy depends on **`counts_per_rev` calibration** and, for
> skid-steer, **wheel slip** (worse in turns). Expect good straight-line
> accuracy once calibrated; treat turn angles as approximate.

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
- `PIDGains` — `kp/ki/kd`, plus `feedforward` (duty to hold 1 m/s),
  `output_limit` (saturation, ±4095) and `integral_limit` (bounds the
  `ki·integral` contribution, in duty units).
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

Coverage (`tests/test_core.py`, 23 tests):
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

> On the first run 19/20 passed; the 20th was a bad assertion (commanding
> 1 rad/s for 4 s yields ~4 rad, which correctly wraps to −2.28 rad in
> (−π, π]). The test now spins for <½ turn so it doesn't wrap.

---

## Running (on the Pi)

```bash
sudo python3 web.py              # web only (port 8080)
sudo python3 web.py --with-tcp   # web + legacy TCP (5000/8000) + power monitor
```

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

## Remaining / by design

- **Legacy import-time hardware init** still exists in `Ultrasonic.py`,
  `Line_Tracking.py`, `Led.py` (`ultrasonic = Ultrasonic()` at import, etc.).
  It's harmless now that `Motor` is shared, but ideally these become lazy too.
- **`Thread.stop_thread()`** is still used to stop the autonomous *mode* threads
  (light/ultrasonic/line). It kills threads via injected async exceptions; the
  new `DriveController` deliberately avoids it with a cooperative stop.
- **`server.py` / `web.py` need on-device testing** — see the *Status* note.
