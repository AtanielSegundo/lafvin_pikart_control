"""
Gray-box identification rig for the PiKart drivetrain.

Collects the (u, v) data needed to fit

    tau * v_dot + v = K * (u - u0)          per side, u = PWM duty

over three experiments, each writing one CSV row per sample tick:

  static        -- lowest duty that breaks the kart away FROM REST -> u0_static
  kinetic       -- lowest duty that SUSTAINS motion once rolling   -> u0_kinetic
  prbs          -- pseudo-random excitation, driving forward       -> K, tau
  prbs_heading  -- the same, spinning in place, output = yaw rate  -> K_w, tau

  tau * w_dot + w = K_w * (u - u0_turn)      (the heading plant)

Y is v_center = (d_left + d_right) / (2*dt), NOT odom.pose.x: pose.x is the
WORLD-frame coordinate (x = integral of d_center*cos(theta)), so any heading
drift -- and a skid-steer always drifts -- shrinks it by cos(theta) for reasons
that have nothing to do with the motor. pose.theta is logged as a DISCARD
criterion instead: a run that curved is contaminated, not fittable.

The ultrasonic is a SAFETY INTERLOCK ONLY: it aborts a run at WALL_STOP_CM and
never enters the CSV.

Alongside v_center the raw count deltas go in too, because meters_per_count
depends on counts_per_rev = 2340, itself a value being calibrated -- keeping the
counts lets a corrected constant be applied offline without re-running.

STOP THE SERVER SERVICE FIRST. read_reset_sides() consumes the counters, so the
DriveController loop and this script would steal each other's counts, and both
processes would drive PCA9685 0x40 over the same I2C bus.

    sudo systemctl stop <servico>
    sudo pigpiod                       # required by encoders + ultrasonic
    sudo python3 Scripts/gray_regression.py static

THE BINDING CONSTRAINT IS THE ENCODER, NOT THE SAMPLE RATE
----------------------------------------------------------
Every edge from all four encoders is delivered to a PYTHON callback over
pigpio's notification socket. At 2340 counts/rev x 4 motors, 0.7 m/s is already
~34 000 callbacks/s, and past roughly ENC_MAX_CPS that path -- not the daemon --
gives out: the loop period stretches in step with the edge rate, counts arrive
in bursts, the quadrature state machine desynchronises and counts BACKWARDS, and
whole ticks return zero with the kart at full power. config.py says the same
thing from the other side, at PositionGains.output_limit: "at high duty (~3000)
the wheel spins faster than the quadrature decoder tracks".

So the PRBS operating point is bounded from ABOVE by the encoder, not only from
below by the dead zone. u_center=2500 +-600 reaches duty 3100, which is inside
the region config.py already documents as broken. Each run now prints an
encoder verdict at the end (see _sample_run) -- read it before repositioning
the kart, because that is when redoing the run is still cheap.

  --lean-encoders   decode only M1 and M4: half the callbacks, and it drops
                    M3's single-phase hack (borrowed direction, x2 fudge).
  --fs              loop rate. Raising it does NOT buy resolution -- the edge
                    rate is set by wheel speed -- it only slices the same edges
                    into noisier buckets.

Then fit with Scripts/regressors.py, which discretises each sample with its own
dt and reports what is wrong with the data before it reports any gains.
"""
import argparse
import csv
import math
import os
import signal
import sys
import threading
import time

from collections import deque
from datetime import datetime

__HERE   = os.path.dirname(os.path.abspath(__file__))
__PARENT = os.path.dirname(__HERE)
__SERVER = os.path.join(__PARENT,"Server")

# Server/ is not a package -- its modules import each other flat ("from config
# import ...", "from PCA9685 import ..."), so Server/ itself goes on the path and
# the imports below stay unprefixed. Importing them as "Server.x" instead loads
# every shared module twice, under two names, with two CONFIG singletons.
if __SERVER not in sys.path:
    sys.path.insert(0, __SERVER)

print(f"[PATH ADDED] {__SERVER}")

from odometry   import SkidSteerOdometry
from config     import CONFIG, ENCODER_PINS, SideMapping
from encoders   import WheelEncoders
from Motor      import Motor
from Ultrasonic import Ultrasonic

# ---------------------------------------------------------------------------
# Experiment constants
# ---------------------------------------------------------------------------
# 50 Hz, not CONFIG.control.loop_hz (20): with tau ~ 0.2 s, 20 Hz gives only 4
# samples per time constant -- too few to identify tau. The trade is velocity
# quantisation (mpc/Ts: 1.7 mm/s at 20 Hz vs 4.4 mm/s at 50 Hz), but you can
# always filter offline and never resample upwards.
#
# Overridable with --fs. Raising it does NOT buy resolution: the encoder edge
# rate is set by wheel speed, not by this, so a shorter window only slices the
# same edges into noisier buckets.
FS            = 50.0
TS            = 1.0 / FS

# Encoder acquisition ceiling, counts/s summed over all decoded motors.
#
# pigpio services edges in the C daemon, but every edge is then delivered to a
# PYTHON callback over the notification socket, where it takes the GIL and a
# lock. That path -- not the daemon -- is the ceiling. Past it the loop period
# stretches in step with the edge rate, counts arrive in bursts (a 65 ms tick
# reporting 1718 counts = 2.4 m/s on a 0.6 m/s kart), the quadrature state
# machine desynchronises and starts counting BACKWARDS under forward duty, and
# whole ticks come back zero with the kart at full power. All four were present
# in the reference run.
#
# 2340 counts/rev x 4 motors means 0.7 m/s is already ~34 kHz of callbacks.
# This is a WARNING threshold, calibrated from the runs where the pathologies
# start: treat a run that exceeds it as unfittable, not as merely noisy.
#
# Note also GLITCH_FILTER_US = 100 in encoders.py: it discards any level that
# does not persist 100 us. At 0.7 m/s the mean edge spacing on one phase is
# 249 us, so an encoder with an asymmetric duty cycle loses real edges there
# too -- a second, independent undercount that grows with speed.
ENC_MAX_CPS   = 20000.0

# The ultrasonic is a SAFETY INTERLOCK ONLY -- it aborts a run when the wall gets
# this close and never appears in the data. Y is v_center.
WALL_STOP_CM  = 20
# A v_center above this is not the kart, it is the counting: CONFIG saturates
# drive commands at max_linear and the wheels cannot outrun that by much even
# open-loop. Reference run: 49% of samples above it, peaking at 2.0 m/s.
MAX_PLAUSIBLE_MPS = 1.5 * CONFIG.control.max_linear

MOVE_EPS_MPS  = 0.03          # |v| below this counts as "not moving"
VEL_WIN_S     = 0.10          # moving-average window for stall decisions

# EVERYTHING is on I2C bus 1: PCA9685 (0x40), the ADC, and the MPU6050 --
# MPU6050_CFG.I2C_BUS is 1 despite the "/dev/i2c-0" comment next to it. So gyro
# rate is not free: it competes with the motor writes for the same bus, and one
# setMotorModel costs 8 setPWM * 4 byte-writes = 32 I2C transactions.
#
# Running the gyro at 200 Hz saturated the bus: the loop overran its period
# (dt wandered from 0.02 to 0.16 s), MPU reads failed, GyroMPU dropped
# `connected`, and each _reconnect backed off 300 ms -- 15 blank ticks of
# w_gyro every time. 100 Hz, with the traffic cuts below, leaves headroom.
# Lower it to 50 (what server.py uses and proves works) if gaps persist.
GYRO_HZ       = 100.0

# Battery sag is a slow signal; reading the ADC every tick just adds I2C traffic
# to the bus the gyro needs. Sampled at this period and held between reads.
VBAT_PERIOD_S = 0.5

# MPU6050 gyro full scale, deg/s. heading.py defaults to GYRO_RANGE_250DEG, i.e.
# +-4.36 rad/s -- which an in-place spin blows straight past: the encoders put
# this kart at 6-15 rad/s (340-860 deg/s) at PRBS_heading duties. Beyond full
# scale the 16-bit reading saturates and wraps, so w_gyro comes back erratic and
# 2-3x BELOW the encoder rate, with occasional values above the range itself.
# +-1000 deg/s = 17.5 rad/s leaves headroom; the resolution it costs (5.3e-4
# rad/s per LSB) is nothing against this signal.
#
# The server has the same exposure: at HeadingGains duties (min_turn_duty 2200,
# output_limit 3200) a turn_in_place spins fast enough to clip, so the heading
# PID closes on corrupted feedback.
GYRO_FS_DEG   = 1000

DATA_DIR = os.path.join(__HERE, "data")

# The per-motor columns (c_M1..c_M4) are RAW signed deltas, before the side
# mean. A side is an average of two motors, so one encoder dying reads as a
# halved side -- i.e. as a gentle curve -- and is invisible in dc_left/dc_right
# alone. These columns are how that failure becomes findable after the fact.
CSV_HEADER = ["t", "dt", "run_id", "phase", "duty",
              "v_center", "v_left", "v_right",
              "w_gyro", "yaw",
              "dc_left", "dc_right", "vbat", "theta",
              "c_M1", "c_M2", "c_M3", "c_M4"]

stop_evt = threading.Event()
PAUSE_BETWEEN_RUNS = True


# ---------------------------------------------------------------------------
# Hardware helpers
# ---------------------------------------------------------------------------
def set_foward_motors_duty(m:Motor,duty_pwm:int):
    m.setMotorModel(duty_pwm,duty_pwm,duty_pwm,duty_pwm)


def set_turn_motors_duty(m:Motor, duty_pwm:int):
    """In-place spin. POSITIVE duty = CCW (left side back, right side forward),
    matching DriveController._set_raw_turn and _heading_duty's (-duty, +duty)
    and the +ccw convention used throughout the server."""
    d = int(duty_pwm)
    m.setMotorModel(-d, -d, d, d)


def get_ultrasonic_handler():
    try:
        from ultrasonic_pigpio import UltrasonicPigpio
        print("[ultrasonic] using pigpio (hardware-timed echo)")
        return UltrasonicPigpio()
    except Exception as e:
        print(f"[ultrasonic] pigpio unavailable ({e}); RPi.GPIO fallback")
        return Ultrasonic()


def get_gyro_handler(sample_rate=GYRO_HZ, fs_deg=GYRO_FS_DEG):
    try:
        from heading import GyroMPU
        from mpu6050 import mpu6050 as _mpu
        rng = {250:  _mpu.GYRO_RANGE_250DEG,  500:  _mpu.GYRO_RANGE_500DEG,
               1000: _mpu.GYRO_RANGE_1000DEG, 2000: _mpu.GYRO_RANGE_2000DEG}[fs_deg]
        # set_configs applies the range in __init__, before calibrate() below,
        # so the bias is measured on the range that will actually be used.
        gyro = GyroMPU(sample_rate=sample_rate,    # __init__ already start()s it
                       gyro_range=rng)
        print(f"[gyro] fundo de escala +-{fs_deg} deg/s "
              f"(+-{math.radians(fs_deg):.1f} rad/s)")
        if gyro.is_connected():
            print("[gyro] calibrating bias -- keep the kart STILL...")
            gyro.calibrate()
        else:
            print("[gyro] MPU6050 not detected; heading falls back to encoders")
        return gyro
    except Exception as e:                      # noqa: BLE001
        print(f"[gyro] unavailable ({e}); heading falls back to encoders")
        return None


def read_gyro_yaw(gyro):
    """Latest gyro yaw in radians (mounting sign/axis applied by GyroMPU),
    or None if no gyro / disconnected."""
    if gyro is None or not gyro.is_connected():
        return None
    try:
        if hasattr(gyro, "get_yaw"):
            return math.radians(gyro.get_yaw())
        return math.radians(gyro.get_angles_gyro()["z"])
    except Exception:                                       # noqa: BLE001
        return None


def dist_ok(cm):
    """Is this ultrasonic reading a MEASUREMENT rather than a sentinel?

    Used only to decide whether the wall interlock may trust a reading.
    UltrasonicPigpio returns max_distance_cm (300) for far/clear, stale AND
    faulted alike; the RPi.GPIO Ultrasonic returns 255 for a failed read. Both
    are large, so an unfiltered sentinel fails in the "keep driving" direction
    -- exactly the direction that matters for a safety stop.
    """
    return True


def test_motors(rig, duty=2000, seconds=1.0):
    """Wiring smoke test. duty defaults ABOVE min_move_duty (1200 in config) --
    a lower default would simply fail to break away and look like dead motors."""
    set_foward_motors_duty(rig.motor, duty)
    stop_evt.wait(seconds)
    set_foward_motors_duty(rig.motor, 0)


def lean_encoder_setup():
    """(sides, pins) decoding ONE motor per side instead of all four.

    Halves the pigpio callback rate, which is the acquisition ceiling (see
    ENC_MAX_CPS). Picks M1 (left) and M4 (right): both are healthy x4
    quadrature, so this also drops M3 entirely -- the single-phase motor whose
    DIRECTION is borrowed from its partner and whose count is doubled to fake
    x4 resolution. That borrowed direction is a guess, and when the partner
    reads exactly zero it always guesses FORWARD, biasing the side.

    The cost is that a side is no longer an average of two motors, so a single
    slipping wheel is no longer smoothed. For identification that is a good
    trade: a clean measurement of one wheel beats a corrupted mean of two.

    Uses CONFIG's own signs so a wiring change stays in one place.
    """
    signs = dict(CONFIG.sides.signs)
    sides = SideMapping(left=("M1",), right=("M4",), signs=signs)
    pins  = {t: ENCODER_PINS[t] for t in ("M1", "M4")}
    return sides, pins


class Rig:
    """Everything the experiments touch, built once."""

    def __init__(self, lean_encoders=False):
        self.motor    = Motor()
        self.ultra    = get_ultrasonic_handler()
        self.gyro     = get_gyro_handler()
        if lean_encoders:
            sides, pins = lean_encoder_setup()
            # modes={} -> no single-phase handling; M3 is not decoded at all.
            self.encoders = WheelEncoders(sides, pins=pins, modes={})
            print("[encoders] modo enxuto: so M1 (esq) e M4 (dir) -- metade "
                  "das callbacks, e sem o remendo de fase unica do M3")
        else:
            self.encoders = WheelEncoders(CONFIG.sides)
        self.enc_tags = tuple(self.encoders.encoders)
        self.encoders.begin()          # registers the pigpio edge callbacks --
                                       # without it every count stays 0 forever
        self.odom     = SkidSteerOdometry(CONFIG.wheel)
        self.mpc      = CONFIG.wheel.meters_per_count
        self._prev_yaw = None
        self._prev_yaw_t = 0.0
        self._vbat = ""
        self._vbat_t = -1e9
        self.gyro_gaps = 0             # ticks with no usable gyro sample
        self.gyro_stale = 0            # ticks where the gyro had not updated

    def vbat(self):
        """Battery volts, sampled at VBAT_PERIOD_S and held in between.

        Logged because pack sag is the largest systematic error across a
        session; with it you can fit the proper gray-box form
        v = K*(u*V/V_nom - u0) instead of watching K drift for no visible
        reason. It does NOT need 50 Hz -- and at 50 Hz the extra I2C read per
        tick was starving the gyro on the same bus.
        """
        now = time.monotonic()
        if now - self._vbat_t < VBAT_PERIOD_S:
            return self._vbat
        self._vbat_t = now
        try:
            self._vbat = round(self.motor.adc.recvADC(2) * 3, 3)
        except Exception:                                   # noqa: BLE001
            self._vbat = ""
        return self._vbat

    def distance_cm(self):
        try:
            return self.ultra.get_distance()
        except Exception:                                   # noqa: BLE001
            return None

    def yaw_sample(self):
        """(yaw_rad, dtheta_rad, w_rad_s) -- the last two None when the gyro has
        produced no NEW sample since the previous tick.

        w is differenced over the interval between two DISTINCT yaw readings,
        not over the control tick. GyroMPU integrates on its own thread, so a
        tick can read back the same yaw twice; dividing an unchanged yaw by the
        tick dt reports w = 0, a measurement the kart never made, and those
        false zeros drag the fitted K_w down. Skipping those ticks and dividing
        by the true elapsed time on the ones that did update keeps every logged
        w honest -- and removes the reason the gyro had to outrun the loop.

        A dropout resets the reference rather than differencing across it:
        while GyroMPU is disconnected _update_loop stops integrating, so yaw
        FREEZES. Differencing across an outage would report far less rotation
        than actually happened.

        No wrap handling: angles["z"] is free-running ("yaw: pure bias-corrected
        gyro"), so it never jumps at +/-pi even after several full turns.
        """
        now = time.monotonic()
        yaw = read_gyro_yaw(self.gyro)
        if yaw is None:
            self._prev_yaw = None
            self.gyro_gaps += 1
            return None, None, None
        if self._prev_yaw is None:
            self._prev_yaw, self._prev_yaw_t = yaw, now
            return yaw, None, None
        if yaw == self._prev_yaw:              # thread has not ticked yet
            self.gyro_stale += 1
            return yaw, None, None
        dtheta = yaw - self._prev_yaw
        span = now - self._prev_yaw_t
        self._prev_yaw, self._prev_yaw_t = yaw, now
        return yaw, dtheta, (dtheta / span if span > 0.0 else None)

    def teardown(self):
        set_foward_motors_duty(self.motor, 0)      # motors FIRST, always
        for fn in (getattr(self.encoders, "stop", None),
                   getattr(self.gyro,     "stop", None),
                   getattr(self.ultra,    "stop", None)):   # RPi.GPIO fallback
            if fn is not None:                              # has no stop()
                try:
                    fn()
                except Exception:                           # noqa: BLE001
                    pass


class RunState:
    """Live state handed to a duty schedule on every tick."""
    __slots__ = ("t", "dt", "phase", "duty", "d_center",
                 "v_center", "v_avg", "w_gyro", "dist_cm", "aborted")

    def __init__(self):
        self.t = self.dt = self.d_center = 0.0
        self.v_center = self.v_avg = 0.0
        self.phase = ""
        self.duty = 0
        self.w_gyro = None
        self.dist_cm = None
        self.aborted = None


# ---------------------------------------------------------------------------
# Core sampling loop
# ---------------------------------------------------------------------------
def _sample_run(rig, writer, run_id, duty_fn, max_time,
                apply_fn=set_foward_motors_duty, wall_guard=True):
    """Drive `duty_fn` at FS and log one row per tick.

    `apply_fn(motor, duty)` is how a duty reaches the wheels: forward drive for
    the translational experiments, in-place spin for the heading one.

    ORDER MATTERS: read -> log(duty_prev) -> decide -> apply.

    read_reset_sides() returns the counts accrued SINCE THE PREVIOUS READ, i.e.
    under the duty already in force -- duty_prev -- not the one about to be
    written. Log the new duty on that row and the series sits one sample early;
    at 50 Hz that is 20 ms, which against tau ~ 200 ms is a 10% error the ARX
    fit absorbs silently as time constant. No fit diagnostic will flag it.

    duty_fn(st) sets st.phase and returns the duty for the NEXT interval, or
    None to end the run.
    """
    st  = RunState()
    win = deque(maxlen=max(1, int(VEL_WIN_S * FS)))
    rig.gyro_gaps = rig.gyro_stale = 0
    ticks = 0
    dt_max = 0.0
    w_peak = 0.0
    # Acquisition pathologies, counted live so a bad run is known BEFORE the
    # kart is repositioned for the next one -- not weeks later in the fit.
    cps_peak = 0.0
    over_cps = impossible = stalled = reversed_ = lopsided = 0
    translational = apply_fn is set_foward_motors_duty

    duty = duty_fn(st)
    if duty is None:
        return st
    apply_fn(rig.motor, duty)
    st.duty = duty_prev = duty
    phase_prev = st.phase

    rig.encoders.read_reset_sides()      # discard anything accrued before t0
    t0 = last = time.monotonic()
    stop_evt.wait(TS)                    # so the first interval is a real one

    while not stop_evt.is_set():
        # -- 1. read FIRST, and take the timestamp FROM the read ------------
        # dt has to be the window the COUNTS accrued over. read_reset_sides
        # returns everything since the PREVIOUS read, so the counting window
        # runs read-to-read -- while the old code measured loop-top to
        # loop-top. Everything in between (writerow, the ultrasonic ping, the
        # 32-byte I2C duty write, the sleep) has variable cost, so v =
        # counts*mpc/dt was dividing a count from one interval by the length
        # of a slightly different one. Timestamping at the read removes that
        # by construction; it costs nothing.
        dc_l, dc_r, raw = rig.encoders.read_reset_detailed()
        now = time.monotonic()
        dt  = now - last
        last = now
        if dt <= 0.0:
            dt = TS
        st.t, st.dt = now - t0, dt
        ticks += 1
        dt_max = max(dt_max, dt)

        d_left   = dc_l * rig.mpc
        d_right  = dc_r * rig.mpc
        d_center = (d_left + d_right) / 2.0

        # Acquisition health, judged per tick and reported once at the end.
        cps = sum(abs(v) for v in raw.values()) / dt
        cps_peak = max(cps_peak, cps)
        if cps > ENC_MAX_CPS:
            over_cps += 1
        if abs(d_center / dt) > MAX_PLAUSIBLE_MPS:
            impossible += 1
        if translational and duty_prev > CONFIG.position.min_move_duty:
            if abs(dc_l) < 2.0 and abs(dc_r) < 2.0:
                stalled += 1        # full duty, zero counts: stream died
            if dc_l < -5.0 or dc_r < -5.0:
                reversed_ += 1      # backwards under forward duty: state lost
        vals = [abs(v) for v in raw.values()]
        if len(vals) > 1 and max(vals) > 80.0 and min(vals) < 0.1 * max(vals):
            lopsided += 1           # one motor running, its partner silent

        yaw, gyro_dtheta, w_gyro = rig.yaw_sample()
        # For the translational runs theta is a diagnostic only -- it decides
        # whether to KEEP the run. For the heading run w_gyro IS the output.
        rig.odom.update_from_distances(d_left, d_right, dt, d_theta=gyro_dtheta)

        st.d_center += d_center
        st.v_center  = d_center / dt
        win.append(st.v_center)
        st.v_avg   = sum(win) / len(win)
        st.w_gyro   = w_gyro
        if w_gyro is not None:
            w_peak = max(w_peak, abs(w_gyro))

        # -- 2. log: the row describes the interval that just ENDED ---------
        writer.writerow([f"{st.t:.4f}", f"{dt:.5f}", run_id, phase_prev,
                         duty_prev, f"{st.v_center:.5f}",
                         f"{d_left / dt:.5f}", f"{d_right / dt:.5f}",
                         "" if st.w_gyro is None else f"{st.w_gyro:.5f}",
                         "" if yaw is None else f"{yaw:.5f}",
                         f"{dc_l:.1f}", f"{dc_r:.1f}",
                         rig.vbat(), f"{rig.odom.pose.theta:.4f}"]
                        + [raw.get(t, "") for t in ("M1", "M2", "M3", "M4")])

        # -- 3. aborts: safety interlock, not data --------------------------
        if wall_guard:
            st.dist_cm = rig.distance_cm()
            if dist_ok(st.dist_cm) and st.dist_cm < WALL_STOP_CM:
                st.aborted = "wall"
                break
        if st.t > max_time:
            st.aborted = "timeout"
            break

        # -- 4. decide the NEXT interval, then apply ------------------------
        duty = duty_fn(st)
        if duty is None:
            break
        if duty != duty_prev:
            # NOT set_foward_motors_duty: the heading run drives this same loop
            # through set_turn_motors_duty. And only on a CHANGE -- one
            # setMotorModel is 32 I2C byte writes, so rewriting an unchanged
            # duty at 50 Hz burned 1600 transactions/s on the bus the gyro and
            # the ADC share, for a duty that only moves every T_switch.
            apply_fn(rig.motor, duty)
        st.duty = duty_prev = duty
        phase_prev = st.phase

        sleep = TS - (time.monotonic() - now)
        if sleep > 0.0:
            stop_evt.wait(sleep)

    if ticks:
        # Loop health. dt_max far above TS, or many gyro gaps, means the I2C bus
        # is saturated and the run is not trustworthy -- see the GYRO_HZ note.
        fs = math.radians(GYRO_FS_DEG)
        print(f"  [loop] {ticks} ticks  dt_max {dt_max * 1000:.0f} ms "
              f"(TS {TS * 1000:.0f})  gyro: {rig.gyro_gaps} lacunas, "
              f"{rig.gyro_stale} repetidos, |w|max {w_peak:.2f} rad/s "
              f"({100 * w_peak / fs:.0f}% do fundo de escala)")
        if dt_max > 2.0 * TS or rig.gyro_gaps > ticks // 10:
            print("  [loop] AVISO: laco estourando o periodo ou gyro caindo -- "
                  "baixe GYRO_HZ (ou FS) antes de confiar nestes dados")
        if w_peak > 0.8 * fs:
            # Past full scale the reading saturates and wraps, and nothing else
            # flags it: w just comes back low and erratic, and the fit reads
            # that as a small, noisy K_w.
            print(f"  [loop] AVISO: w chegou a {100 * w_peak / fs:.0f}% do fundo "
                  f"de escala do gyro -- suba GYRO_FS_DEG (hoje {GYRO_FS_DEG}) "
                  f"ou baixe o duty; acima da faixa a leitura satura e os dados "
                  f"de heading nao valem")

        # -- encoder acquisition verdict ---------------------------------
        # Said HERE, at the end of the run, because that is when it is still
        # cheap to act on: the kart has not been repositioned yet and the run
        # can simply be redone slower. The same faults found offline cost a
        # whole session.
        print(f"  [enc]  pico {cps_peak:.0f} contagens/s "
              f"(teto {ENC_MAX_CPS:.0f}); acima do teto em {over_cps}/{ticks} "
              f"ticks")
        faults = []
        if over_cps > ticks // 20:
            faults.append(f"{over_cps} ticks acima do teto de contagem")
        if impossible:
            faults.append(f"{impossible} ticks com |v| > {MAX_PLAUSIBLE_MPS:.2f} "
                          f"m/s (impossivel)")
        if stalled:
            faults.append(f"{stalled} ticks com ZERO contagem e duty de avanco")
        if reversed_:
            faults.append(f"{reversed_} ticks contando para TRAS com duty de "
                          f"avanco")
        if lopsided:
            faults.append(f"{lopsided} ticks com um motor parado e o parceiro "
                          f"girando")
        if faults:
            print("  [enc]  AVISO: aquisicao de encoder degradada -- "
                  + "; ".join(faults))
            print("  [enc]  Esta corrida provavelmente NAO e ajustavel. As "
                  "bordas chegam por callback Python do pigpio: acima de ~"
                  f"{ENC_MAX_CPS:.0f} contagens/s elas atrasam, chegam em "
                  "rajada e a quadratura perde o estado. Reduza o duty, use "
                  "--lean-encoders (metade das callbacks), ou ambos.")
    return st


def _brake_and_settle(rig, seconds=0.8):
    """Duty 0 is a BRAKE, not coast: Motor.py writes 4095 to BOTH channels,
    shorting the motor terminals. Drain the counters afterwards so the braking
    transient does not leak into the next run."""
    set_foward_motors_duty(rig.motor, 0)
    stop_evt.wait(seconds)
    rig.encoders.read_reset_sides()


def _pause(msg):
    if not PAUSE_BETWEEN_RUNS or stop_evt.is_set():
        return
    try:
        input(f"[pausa] {msg} -- ENTER para seguir (Ctrl+C aborta): ")
    except (EOFError, KeyboardInterrupt):
        stop_evt.set()


# ---------------------------------------------------------------------------
# Experiment 1: static (breakaway) threshold
# ---------------------------------------------------------------------------
def get_static_threshold(rig, writer, lo=400, hi=2200, step=100,
                         pulse_s=0.6, repeats=3, move_min_m=0.01):
    """Lowest duty that breaks the kart away FROM REST.

    Each level is a short pulse from a standstill. Below threshold the kart does
    not move at all, so the sweep costs almost no floor -- and it stops at the
    first level that passes, before the kart starts covering real ground.

    Breakaway is stochastic (where the gear teeth happen to sit, etc.), so a
    level must move on ALL `repeats` pulses to count. One lucky breakaway is not
    a threshold.

    Returns the duty, or None if the sweep aborted / found nothing.
    """
    print(f"[static] varrendo {lo}..{hi} passo {step}, {repeats}x por nivel")
    for duty in range(lo, hi + 1, step):
        moved = 0
        for r in range(repeats):
            if stop_evt.is_set():
                return None
            _brake_and_settle(rig, 0.8)

            def sched(st, _d=duty):
                st.phase = "static_pulse"
                return _d if st.t < pulse_s else None

            st = _sample_run(rig, writer, f"static_u{duty}_r{r}",
                             sched, max_time=pulse_s + 0.5)
            if st.aborted == "wall":
                print("[static] abortado: parede")
                return None
            print(f"[static]   duty={duty:5d} r={r}  "
                  f"d_center={st.d_center * 1000:7.1f} mm")
            if st.d_center > move_min_m:
                moved += 1

        if moved == repeats:
            _brake_and_settle(rig, 0.5)
            print(f"[static] THRESHOLD ESTATICO = {duty}")
            return duty

    print(f"[static] nenhum nivel ate {hi} moveu o kart")
    return None


# ---------------------------------------------------------------------------
# Experiment 2: kinetic (dropout) threshold
# ---------------------------------------------------------------------------
def get_cinematic_threshold(rig, writer, u_start=3000, step=150, hold_s=0.35,
                            warmup_s=1.0, u_floor=300, stall_ticks=4):
    """Lowest duty that SUSTAINS motion once the kart is already rolling.

    A DIFFERENT experiment from the static one, not a variation: static friction
    is only measurable from rest, kinetic only while moving. So this run starts
    moving and steps DOWN, and must never pass through duty 0 (which brakes,
    injecting a nonlinear transient into the middle of the data).

    Descending monotonically correlates duty with time, which normally biases
    the fit through battery sag -- acceptable here only because the whole run
    lasts a few seconds.

    hold_s ~ 1.75*tau: settled enough to judge, short enough to fit the floor.

    Returns the lowest duty that still sustained motion, or None if aborted.
    """
    state = {"duty": u_start, "t_step": warmup_s,
             "stall": 0, "last_alive": u_start}

    def sched(st):
        if st.t < warmup_s:
            st.phase = "warmup"
            return u_start

        st.phase = "descend"
        if st.t - state["t_step"] >= hold_s:
            state["duty"] -= step
            state["t_step"] = st.t
            state["stall"] = 0
            if state["duty"] < u_floor:
                return None

        # Judge only after the step has had time to act, else you read the
        # previous level's velocity and the threshold comes out too high.
        if st.t - state["t_step"] > hold_s * 0.5:
            if abs(st.v_avg) < MOVE_EPS_MPS:
                state["stall"] += 1
                if state["stall"] >= stall_ticks:
                    return None
            else:
                state["stall"] = 0
                state["last_alive"] = state["duty"]
        return state["duty"]

    max_t = warmup_s + ((u_start - u_floor) / float(step)) * hold_s + 1.0
    st = _sample_run(rig, writer, f"kinetic_from{u_start}", sched,
                     max_time=max_t)
    _brake_and_settle(rig, 0.8)

    if st.aborted == "wall":
        print("[kinetic] abortado: parede (resultado nao confiavel)")
        return None
    print(f"[kinetic] THRESHOLD CINEMATICO = {state['last_alive']} "
          f"(travou em {state['duty']}, {st.d_center:.2f} m de chao)")
    return state["last_alive"]


# ---------------------------------------------------------------------------
# Experiment 3: PRBS excitation -> K and tau
# ---------------------------------------------------------------------------
# Maximum-length Galois LFSR tap masks, verified by brute force to give the full
# 2^n - 1 period for every non-zero seed. Galois rather than Fibonacci because
# the shift-and-conditional-xor form cannot collapse to the all-zero state.
PRBS_TAPS = {6: 0x021, 7: 0x041, 9: 0x108}


def prbs_bits(n_bits=6, seed=1):
    """Galois LFSR, maximum-length sequence (period 2^n - 1).

    Preferred over random.choice(): the sequence is balanced, its autocorrelation
    is near-white (exactly the persistent-excitation property the fit needs), and
    it is REPRODUCIBLE -- the same seed replays the same experiment.
    """
    mask = PRBS_TAPS[n_bits]
    reg  = (seed & ((1 << n_bits) - 1)) or 1
    while True:
        lsb = reg & 1
        reg >>= 1
        if lsb:
            reg ^= mask
        yield lsb


def PRBS(rig, writer, u_center=2500, u_amp=600, t_switch=0.10,
         n_bits=6, seed=1, warmup_s=1.0, u_kinetic=None):
    """Pseudo-random binary excitation at one operating point -- the run that
    actually identifies K and tau.

    A staircase spends nearly all its time on plateaus, which only re-measure K;
    tau lives in the transients, so N steps give N looks at tau. This gives
    2^n - 1 of them over the SAME stretch of floor, the binding constraint here.
    n_bits=6 -> 63 bits * 0.10 s = 6.3 s, about 4 m.

    t_switch belongs between tau/3 and tau: slower and the excitation is
    quasi-static, so tau stops being identifiable (the same reason a ramp is
    useless); faster and the plant cannot respond at all.

    Both levels MUST clear the kinetic threshold. If the kart stalls mid-run the
    record picks up static breakaway transients -- a different, nonlinear
    dynamic than the one being fitted. Pass `u_kinetic` and this is checked.

    Warm-up rows are logged but tagged phase="warmup": drop them offline. Their
    job is to have the kart already AT the operating point when the prbs rows
    begin, so no breakaway contaminates the identification data.
    """
    u_lo, u_hi = u_center - u_amp, u_center + u_amp
    if u_kinetic is not None and u_lo <= u_kinetic:
        raise ValueError(
            f"u_center - u_amp = {u_lo} <= limiar cinetico {u_kinetic}: "
            f"o kart trava no meio da corrida e contamina o ajuste")

    gen    = prbs_bits(n_bits, seed)
    t_prbs = ((1 << n_bits) - 1) * t_switch
    state  = {"duty": u_center, "t_switch": warmup_s}

    def sched(st):
        if st.t < warmup_s:
            st.phase = "warmup"
            return u_center
        if st.t > warmup_s + t_prbs:
            return None
        st.phase = "prbs"
        if st.t - state["t_switch"] >= t_switch:
            state["duty"] = u_hi if next(gen) else u_lo
            state["t_switch"] = st.t
        return state["duty"]

    print(f"[prbs] u={u_center} +-{u_amp} ({u_lo}/{u_hi})  "
          f"T_sw={t_switch * 1000:.0f} ms  {t_prbs:.1f} s")
    st = _sample_run(rig, writer, f"prbs_u{u_center}_a{u_amp}_s{seed}",
                     sched, max_time=warmup_s + t_prbs + 1.0)
    _brake_and_settle(rig, 0.8)
    print(f"[prbs]   t={st.t:.1f} s  chao={st.d_center:.2f} m  "
          f"theta={math.degrees(rig.odom.pose.theta):+.1f} deg  "
          f"abort={st.aborted}")
    return st


# ---------------------------------------------------------------------------
# Experiment 4: PRBS on the heading axis -> K_w (and tau again)
# ---------------------------------------------------------------------------
def PRBS_heading(rig, writer, u_center=2800, u_amp=400, t_switch=0.10,
                 n_bits=7, seed=1, warmup_s=1.0, u_turn_threshold=None):
    """PRBS excitation of the ROTATION axis, in place.

    Same identification as PRBS(), different plant:

        tau * w_dot + w = K_w * (u - u0_turn)

    with u applied as an in-place spin (left = -u, right = +u) and the output
    w = yaw rate from the gyro, not v_center.

    tau is EXPECTED to come out near the translational one and the gain to
    change -- but fit it freely rather than pinning it: rotational inertia is
    not mass, and an in-place skid turn is dominated by lateral tyre scrub
    (Coulomb), which loads the motors differently than rolling does. Letting tau
    float means the run CONFIRMS the assumption instead of hiding a violation
    of it inside K_w.

    Three things differ from the translational run:

    1. NO FLOOR BUDGET. The kart spins in place instead of driving away, so the
       binding constraint that forced n_bits=6 on PRBS() is gone. n_bits=7 ->
       127 bits * 0.1 s = 12.7 s in the same square metre, which is the cheapest
       variance reduction available anywhere in this rig. Raise it further if
       the gyro bias holds.

    2. The wall interlock is OFF by default. A spinning kart sweeps its front
       sensor across the whole room, so seeing something at 20 cm is expected
       and means nothing about collision -- leaving the guard on would abort
       almost every run. The timeout and Ctrl+C remain.

    3. Both levels must clear the TURN threshold, which is much higher than the
       drive one: min_turn_duty is 2200 in HeadingGains against min_move_duty
       1200, because an in-place skid must break lateral scrub on four tyres,
       not just roll. Hence the 2800 +/- 400 default -- inside HeadingGains'
       output_limit of 3200 and clear of 2200.

    Unipolar (both levels the same sign), like the translational run: alternating
    +u/-u would keep the heading near its start but cross the dead zone on every
    single transition, injecting the exact nonlinearity being factored out. The
    kart just keeps spinning instead, which costs nothing here.
    """
    if rig.gyro is None or not rig.gyro.is_connected():
        raise RuntimeError(
            "PRBS_heading precisa do gyro: w e a saida do modelo. O w derivado "
            "dos encoders nao serve -- numa rotacao no lugar as quatro rodas "
            "escorregam lateralmente, que e justamente por que o servidor trata "
            "o gyro como ground truth para d_theta.")

    u_lo, u_hi = u_center - u_amp, u_center + u_amp
    if u_turn_threshold is not None and u_lo <= u_turn_threshold:
        raise ValueError(
            f"u_center - u_amp = {u_lo} <= limiar de giro {u_turn_threshold}: "
            f"o kart para de girar no meio da corrida e contamina o ajuste")

    gen    = prbs_bits(n_bits, seed)
    t_prbs = ((1 << n_bits) - 1) * t_switch
    state  = {"duty": u_center, "t_switch": warmup_s}

    def sched(st):
        if st.t < warmup_s:
            st.phase = "warmup"
            return u_center
        if st.t > warmup_s + t_prbs:
            return None
        st.phase = "prbs_head"
        if st.t - state["t_switch"] >= t_switch:
            state["duty"] = u_hi if next(gen) else u_lo
            state["t_switch"] = st.t
        return state["duty"]

    print(f"[prbs_head] u={u_center} +-{u_amp} ({u_lo}/{u_hi}) CCW  "
          f"T_sw={t_switch * 1000:.0f} ms  {t_prbs:.1f} s")
    st = _sample_run(rig, writer, f"prbs_head_u{u_center}_a{u_amp}_s{seed}",
                     sched, max_time=warmup_s + t_prbs + 1.0,
                     apply_fn=set_turn_motors_duty, wall_guard=False)
    _brake_and_settle(rig, 1.0)
    print(f"[prbs_head]   t={st.t:.1f} s  "
          f"giro total={math.degrees(rig.odom.pose.theta):+.0f} deg  "
          f"deriva={st.d_center:.2f} m  abort={st.aborted}")
    return st


# ---------------------------------------------------------------------------
# CSV / entry point
# ---------------------------------------------------------------------------
def _open_csv(tag):
    os.makedirs(DATA_DIR, exist_ok=True)
    path = os.path.join(DATA_DIR,
                        f"gray_{tag}_{datetime.now():%Y%m%d_%H%M%S}.csv")
    fh = open(path, "w", newline="")
    w = csv.writer(fh)
    w.writerow(CSV_HEADER)
    print(f"[csv] {path}")
    return fh, w, path


def check_levels(kind, u_lo, threshold, measured):
    """Warn (or refuse) when the PRBS low level sits in the dead zone.

    Below the threshold the kart is not a first-order plant, it is static
    friction: no tau explains those samples, and they are typically half the
    record. A MEASURED threshold is a fact, so violating it aborts; a threshold
    taken from config is an assumption, so it only warns -- refusing on a guess
    would block legitimate runs on a re-tuned build.
    """
    if threshold is None or u_lo > threshold:
        return
    msg = (f"o nivel BAIXO do PRBS ({u_lo}) esta em ou abaixo do limiar de "
           f"{kind} ({threshold}): o kart cai na zona morta no meio da corrida "
           f"e contamina o ajuste")
    if measured:
        raise ValueError(msg)
    print()
    print(f"  !! AVISO: {msg}.")
    print( "     O limiar veio do config.py, nao de uma medicao -- rode o "
           "experimento static/kinetic para saber")
    print( "     o valor real desta montagem, ou suba --u-center / baixe "
           "--u-amp.")
    print()


def _install_signal_handlers():
    def _on_signal(signum, frame):
        stop_evt.set()
        # Installing a handler stops Ctrl+C raising KeyboardInterrupt, so a
        # wedged loop would be unkillable -- with motors running. Restore the
        # default so the SECOND Ctrl+C always works.
        signal.signal(signum, signal.SIG_DFL)
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, _on_signal)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Coleta de dados para identificacao cinza do PiKart.")
    ap.add_argument("mode", choices=("static", "kinetic", "prbs",
                                     "prbs_heading", "all", "smoke"))
    ap.add_argument("--u-center", type=int, action="append",
                    help="ponto de operacao do PRBS (repetivel; "
                         "padrao 1500 2500 3500 no linear, 2800 no heading)")
    # Per-mode default, NOT a shared 600. The old code read
    #   u_amp = args.u_amp if args.u_center else 400
    # so the heading default of 400 applied ONLY when --u-center was absent;
    # passing --u-center 1250 silently took the translational 600 instead and
    # put the low level at 650 -- a third of min_turn_duty, i.e. the kart
    # sitting in stiction for half the record. Left as None it now resolves
    # per experiment, so choosing an operating point cannot change the
    # amplitude behind your back.
    ap.add_argument("--u-amp",     type=int,   default=None,
                    help="amplitude do PRBS (padrao: 600 no linear, "
                         "400 no heading)")
    ap.add_argument("--t-switch",  type=float, default=0.10)
    ap.add_argument("--fs",        type=float, default=FS,
                    help=f"taxa de amostragem do laco em Hz (padrao {FS:g}). "
                         f"Subir isto NAO melhora a resolucao: a taxa de "
                         f"bordas do encoder depende da velocidade da roda, "
                         f"nao daqui.")
    ap.add_argument("--lean-encoders", action="store_true",
                    help="decodifica so M1 (esq) e M4 (dir) em vez dos quatro "
                         "motores: metade das callbacks do pigpio, e sem o "
                         "remendo de fase unica do M3")
    ap.add_argument("--u-kinetic", type=int,   default=None,
                    help="limiar cinetico ja medido; valida os niveis do PRBS")
    ap.add_argument("--u-turn",    type=int,   default=None,
                    help="limiar de giro ja medido; valida o PRBS de heading")
    ap.add_argument("--n-bits",    type=int,   default=None,
                    choices=(6, 7, 9),
                    help="comprimento da sequencia PRBS (2^n - 1 bits)")
    ap.add_argument("--seed",      type=int,   default=1)
    ap.add_argument("--no-pause",  action="store_true",
                    help="nao espera o operador reposicionar entre corridas")
    args = ap.parse_args()

    PAUSE_BETWEEN_RUNS = not args.no_pause
    FS = float(args.fs)
    TS = 1.0 / FS
    _install_signal_handlers()

    rig = Rig(lean_encoders=args.lean_encoders)
    fh, writer, path = _open_csv(args.mode)
    u_static = u_kin = None
    try:
        if args.mode == "smoke":
            test_motors(rig)

        if args.mode in ("static", "all"):
            _pause("posicione o kart com ~1 m livre a frente")
            u_static = get_static_threshold(rig, writer)

        if args.mode in ("kinetic", "all"):
            _pause("reposicione o kart com ~5 m livres a frente")
            u_kin = get_cinematic_threshold(rig, writer)

        if args.mode in ("prbs", "all"):
            # Provenance matters: a threshold this session MEASURED (or the
            # operator passed) is a fact and aborts the run; the config value is
            # an assumption and only warns.
            u_kin_check = args.u_kinetic if args.u_kinetic is not None else u_kin
            measured = u_kin_check is not None
            if u_kin_check is None:
                u_kin_check = CONFIG.position.min_move_duty
            amp = args.u_amp if args.u_amp is not None else 600
            for u in (args.u_center or [1500, 2500, 3500]):
                if stop_evt.is_set():
                    break
                check_levels("movimento", u - amp, u_kin_check, measured)
                _pause(f"reposicione o kart com ~5 m livres (PRBS u={u})")
                PRBS(rig, writer, u_center=u, u_amp=amp,
                     t_switch=args.t_switch, seed=args.seed,
                     n_bits=args.n_bits or 6,
                     u_kinetic=u_kin_check if measured else None)

        if args.mode in ("prbs_heading", "all"):
            u_turn_check = args.u_turn
            measured = u_turn_check is not None
            if u_turn_check is None:
                u_turn_check = CONFIG.heading.min_turn_duty
            amp = args.u_amp if args.u_amp is not None else 400
            for u in (args.u_center or [2800]):
                if stop_evt.is_set():
                    break
                check_levels("giro", u - amp, u_turn_check, measured)
                _pause(f"deixe ~1 m livre em volta do kart (giro u={u})")
                PRBS_heading(rig, writer, u_center=u, u_amp=amp,
                             t_switch=args.t_switch, seed=args.seed,
                             n_bits=args.n_bits or 7,
                             u_turn_threshold=args.u_turn)
    finally:
        rig.teardown()
        fh.close()
        print(f"\n[resumo] estatico={u_static}  cinetico={u_kin}")
        print(f"[csv] {path}")

# THRESHOLD ESTATICO = 800
# TRESHOLF CINEMATICO = 1200