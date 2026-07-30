#!/usr/bin/python3
"""
MPU6050 heading estimator.

A background thread samples the gyroscope at `sample_rate` and integrates the
angular rates into per-axis angles (x, y, z in DEGREES). The accelerometer's
gravity vector corrects the drift-prone gyro on roll/pitch (x, y) via a
complementary filter; z (yaw) is pure bias-corrected gyro integration -- there
is no gravity reference for yaw, so only the calibrated bias keeps it steady.

Fail-safe: if a read fails (MPU6050 unplugged / I2C glitch), the thread flips
`connected` to False, keeps the last cached calibration, and retries the
connection + re-applies the configs until it comes back.

Calibration (kart AT REST): averages N gyro samples for the per-axis bias and N
accel samples for the resting pose, seeds x/y from the accel-derived roll/pitch
and z to 0, and caches the bias to disk so a disconnect (or restart) does not
force a re-calibration.

External agents read `get_angles_gyro()` (non-blocking, returns the latest
integrated angles) and `connected` / `is_connected()` for the health flag.

Wired to GPIO0 (SDA) / GPIO1 (SCL) = the secondary I2C bus, /dev/i2c-0.
Deps:  pip install mpu6050-raspberrypi
"""
import json
import math
import os
import threading
import time
from dataclasses import dataclass

from mpu6050 import mpu6050

MPU6050_ADDR = 0x68        # 0x69 if AD0 is tied high
MPU6050_SDA  = 0           # GPIO0 (physical pin 27) -> SDA0
MPU6050_SCL  = 1           # GPIO1 (physical pin 28) -> SCL0
I2C_BUS      = 0           # /dev/i2c-0  (GPIO0/1)
SAMPLE_HZ    = 2.0         # readings per second

GRAVITY_MS2  = 9.80665     # accel magnitude at rest; used to reject motion
CAL_CACHE    = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            ".mpu6050_cal.json")


@dataclass(frozen=True)
class MPU6050_CFG:
    ADDR : int = 0x68
    SDA  : int = 0  # GPIO0 -> SDA0
    SCL  : int = 1  # GPIO1 -> SCL0
    I2C_BUS : int = 0  # /dev/i2c-0  (GPIO0/1)


class GyroMPU:
    def __init__(self,
                 sample_rate: float = SAMPLE_HZ,
                 accel_range: int = mpu6050.ACCEL_RANGE_2G,
                 gyro_range:  int = mpu6050.GYRO_RANGE_250DEG,
                 filter_bw:   int = mpu6050.FILTER_BW_256,
                 comp_alpha:  float = 0.98,      # gyro weight in the x/y filter
                 cache_path:  str = CAL_CACHE):
        
        self.sample_rate = max(1.0, float(sample_rate))
        self._period = 1.0 / self.sample_rate
        self.comp_alpha = comp_alpha
        self._cache_path = cache_path

        # Desired sensor config, kept so a reconnect can re-apply it.
        self._accel_range = accel_range
        self._gyro_range = gyro_range
        self._filter_bw = filter_bw

        self.port = None
        self.connected = False          # <-- health flag for external consumers
        self.calibrated = False

        # Integrated angles (degrees) + gyro bias (deg/s). `heading` aliases yaw.
        self.angles = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.gyro_bias = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.heading = 0.0

        self._lock = threading.Lock()       # guards angles / bias / heading
        self._io_lock = threading.Lock()    # serialises I2C access to the port
        self._stop_evt = threading.Event()
        self._thread = None

        self._load_cache()                  # restore last calibration if any
        self.init_mpu6050()
        if self.connected:
            self.set_configs(accel_range, gyro_range, filter_bw)
            self._seed_from_accel()         # initial roll/pitch from gravity

        self.start()

    # ------------------------------------------------------------------ #
    # Connection
    # ------------------------------------------------------------------ #
    def init_mpu6050(self):
        try:
            port = mpu6050(MPU6050_CFG.ADDR, bus=MPU6050_CFG.I2C_BUS)
            port.get_gyro_data()            # probe: confirm the device answers
            self.port = port
            self.connected = True
            print(f"[GyroMPU] MPU6050 initialized at 0x{MPU6050_ADDR:02x} on i2c-{I2C_BUS}")
        except Exception as e:                                  # noqa: BLE001
            self.port = None
            self.connected = False
            print(f"[GyroMPU] MPU6050 not found at 0x{MPU6050_ADDR:02x} on i2c-{I2C_BUS} ({e})")

    def set_configs(self, accel_range, gyro_range, filter_bw):
        self._accel_range = accel_range
        self._gyro_range = gyro_range
        self._filter_bw = filter_bw
        if not self.port:
            return False
        try:
            with self._io_lock:
                self.port.set_accel_range(accel_range)
                self.port.set_gyro_range(gyro_range)
                self.port.set_filter_range(filter_bw)
            return True
        except Exception as e:                                  # noqa: BLE001
            print(f"[GyroMPU] set_configs failed ({e})")
            self.port = None
            self.connected = False
            return False

    def _reconnect(self):
        """Re-open the device and re-apply configs; keep the cached bias and
        re-seed roll/pitch from gravity so angles don't jump on return."""
        self.init_mpu6050()
        if self.connected:
            self.set_configs(self._accel_range, self._gyro_range, self._filter_bw)
            self._seed_from_accel()
        return self.connected

    def is_connected(self) -> bool:
        return self.connected

    # ------------------------------------------------------------------ #
    # Sensor I/O (serialised so the loop and calibrate never share the bus)
    # ------------------------------------------------------------------ #
    def _read_raw(self):
        """Return (gyro, accel) dicts. Raises on I2C failure."""
        with self._io_lock:
            gyro = self.port.get_gyro_data()
            accel = self.port.get_accel_data()
        return gyro, accel

    @staticmethod
    def _roll_pitch(accel):
        """Accel gravity vector -> (roll about x, pitch about y) in degrees."""
        ax, ay, az = accel["x"], accel["y"], accel["z"]
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        return roll, pitch

    def _seed_from_accel(self):
        """Set x/y to the current gravity-derived roll/pitch (absolute ref)."""
        if not self.port:
            return
        try:
            _, accel = self._read_raw()
        except Exception:                                       # noqa: BLE001
            self.port = None
            self.connected = False
            return
        roll, pitch = self._roll_pitch(accel)
        with self._lock:
            self.angles["x"] = roll
            self.angles["y"] = pitch

    # ------------------------------------------------------------------ #
    # Calibration (kart AT REST)
    # ------------------------------------------------------------------ #
    def calibrate(self, samples: int = 200, sample_delay: float = 0.005) -> bool:
        """Average `samples` readings while stationary to find the gyro bias and
        resting pose. Seeds x/y from the accel roll/pitch, z to 0, and caches the
        bias so a later disconnect/restart reuses it instead of re-calibrating.
        """
        if not self.connected and not self._reconnect():
            print("[GyroMPU] calibrate: sensor unavailable")
            return False

        gsum = {"x": 0.0, "y": 0.0, "z": 0.0}
        asum = {"x": 0.0, "y": 0.0, "z": 0.0}
        n = 0
        for _ in range(max(1, samples)):
            try:
                gyro, accel = self._read_raw()
            except Exception:                                   # noqa: BLE001
                continue                                        # skip glitches
            for k in gsum:
                gsum[k] += gyro[k]
                asum[k] += accel[k]
            n += 1
            time.sleep(sample_delay)

        if n == 0:
            self.port = None
            self.connected = False
            print("[GyroMPU] calibrate: no samples read (sensor lost)")
            return False

        bias = {k: gsum[k] / n for k in gsum}
        accel_avg = {k: asum[k] / n for k in asum}
        roll, pitch = self._roll_pitch(accel_avg)

        with self._lock:
            self.gyro_bias = bias
            self.angles = {"x": roll, "y": pitch, "z": 0.0}
            self.heading = 0.0
            self.calibrated = True
        self._save_cache()
        print(f"[GyroMPU] calibrated over {n} samples: "
              f"bias=({bias['x']:+.3f},{bias['y']:+.3f},{bias['z']:+.3f}) deg/s, "
              f"rest pose roll={roll:+.2f} pitch={pitch:+.2f} deg")
        return True

    def _load_cache(self):
        try:
            with open(self._cache_path) as f:
                data = json.load(f)
            self.gyro_bias = {k: float(data["gyro_bias"][k]) for k in ("x", "y", "z")}
            self.calibrated = True
            print(f"[GyroMPU] loaded cached calibration: bias={self.gyro_bias}")
        except (OSError, KeyError, ValueError, TypeError):
            pass                                                # no/invalid cache

    def _save_cache(self):
        try:
            with open(self._cache_path, "w") as f:
                json.dump({"gyro_bias": self.gyro_bias,
                           "calibrated_at": time.time()}, f)
        except OSError as e:
            print(f"[GyroMPU] could not cache calibration ({e})")

    # ------------------------------------------------------------------ #
    # Background integration thread
    # ------------------------------------------------------------------ #
    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._update_loop, daemon=True,
                                        name="GyroMPU")
        self._thread.start()

    def stop(self):
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _update_loop(self):
        last = time.monotonic()
        while not self._stop_evt.is_set():
            start = time.monotonic()

            if not self.port:                       # (re)connect and re-config
                self._reconnect()
                if not self.connected:
                    self._stop_evt.wait(1.0)        # backoff, then retry
                    last = time.monotonic()
                    continue

            try:
                gyro, accel = self._read_raw()
            except Exception as e:                  # noqa: BLE001 -- lost sensor
                print(f"[GyroMPU] read failed ({e}); reconnecting...")
                self.port = None
                self.connected = False
                self._stop_evt.wait(0.5)
                last = time.monotonic()
                continue

            now = time.monotonic()
            dt = min(now - last, 5 * self._period)  # clamp after a stall
            last = now
            self._integrate(gyro, accel, dt)

            elapsed = time.monotonic() - start
            self._stop_evt.wait(max(0.0, self._period - elapsed))

    def _integrate(self, gyro, accel, dt):
        # Gyro integration with the calibrated bias removed.
        with self._lock:
            bx, by, bz = self.gyro_bias["x"], self.gyro_bias["y"], self.gyro_bias["z"]
            gx = (gyro["x"] - bx) * dt + self.angles["x"]
            gy = (gyro["y"] - by) * dt + self.angles["y"]
            gz = (gyro["z"] - bz) * dt + self.angles["z"]

        # Accel gives an absolute roll/pitch reference -- but only trust it when
        # the kart isn't accelerating (|a| ~ 1 g), else linear accel corrupts it.
        amag = math.sqrt(accel["x"] ** 2 + accel["y"] ** 2 + accel["z"] ** 2)
        a = self.comp_alpha
        if abs(amag - GRAVITY_MS2) < 1.5:
            roll, pitch = self._roll_pitch(accel)
            gx = a * gx + (1.0 - a) * roll
            gy = a * gy + (1.0 - a) * pitch

        with self._lock:
            self.angles["x"] = gx
            self.angles["y"] = gy
            self.angles["z"] = gz           # yaw: pure (bias-corrected) gyro
            self.heading = gz
        self.connected = True

    # ------------------------------------------------------------------ #
    # External API
    # ------------------------------------------------------------------ #
    def get_angles_gyro(self):
        """Latest integrated angles {'x','y','z'} in degrees (non-blocking).
        Consumed by external agents; the thread keeps it fresh."""
        with self._lock:
            return dict(self.angles)

    def reset(self):
        """Zero the angles; re-seed roll/pitch from gravity if available."""
        with self._lock:
            self.angles = {"x": 0.0, "y": 0.0, "z": 0.0}
            self.heading = 0.0
        self._seed_from_accel()


if __name__ == "__main__":
    imu = GyroMPU(sample_rate=50.0)
    print("Calibrating -- keep the kart still...")
    imu.calibrate(samples=200)
    try:
        while True:
            ang = imu.get_angles_gyro()
            flag = "OK  " if imu.is_connected() else "DISC"
            print(f"[{flag}] x={ang['x']:+7.2f} y={ang['y']:+7.2f} z={ang['z']:+7.2f} deg")
            time.sleep(0.2)
    except KeyboardInterrupt:
        imu.stop()
        print("\n[INFO] stopped")
