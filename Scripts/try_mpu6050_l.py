#!/usr/bin/env python3
"""
Server-identical MPU6050 check: drives Server/heading.py's GyroMPU the same way
ServerNode does, rather than talking to the chip directly like try_mpu6050.py.

Use this one when the raw check passes but the server still logs
    [gyro] unavailable (No module named 'mpu6050')
or when a gyro-closed turn misbehaves. try_mpu6050.py proves the SENSOR works;
this proves the SERVER'S PATH to it works -- same import, same construction,
same calibration, same read call the drive controller makes -- and adds the two
checks that decide whether turn_in_place will behave:

  * MOUNTING: resting roll near +/-180 deg means the board is upside down.
  * YAW SIGN: whether a counter-clockwise turn reads POSITIVE. If it reads
    negative while heading.py says YAW_SIGN = 1, the heading PID becomes
    positive feedback -- error grows as the kart turns, so it spins at full duty
    until HeadingGains.max_time expires. Check this BEFORE commanding a turn.

Run it as the same user the service does. The service is `User=root`, so the
interesting case is:
    sudo python3 Scripts/try_mpu6050_l.py

Examples:
    python3 Scripts/try_mpu6050_l.py                  # calibrate, then monitor
    python3 Scripts/try_mpu6050_l.py --sign-check     # decide YAW_SIGN
    python3 Scripts/try_mpu6050_l.py --no-calibrate   # reuse the cached bias

Deps:  pip install mpu6050-raspberrypi   (must be visible to THIS interpreter)
"""
import argparse
import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SERVER_DIR = os.path.join(os.path.dirname(_HERE), "Server")


def _euid():
    """Effective uid, or -1 where the OS has no such concept (running this on a
    dev box rather than the Pi)."""
    getter = getattr(os, "geteuid", None)
    return getter() if getter else -1


def _who(uid):
    if uid < 0:
        return "non-POSIX host -- not the Pi, so this only checks the import"
    return "root -- same as the service" if uid == 0 else f"uid {uid}"


def import_gyro():
    """Import GyroMPU as the server does, and explain a failure the way the
    journal cannot. `heading` lives in Server/ -- the service's WorkingDirectory
    but not this script's -- so it goes on the path first."""
    if _SERVER_DIR not in sys.path:
        sys.path.insert(0, _SERVER_DIR)
    try:
        from heading import GyroMPU, YAW_AXIS, YAW_SIGN
        return GyroMPU, YAW_AXIS, YAW_SIGN
    except ImportError as e:
        uid = _euid()
        print(f"[FATAL] cannot import GyroMPU: {e}\n")
        print(f"  interpreter : {sys.executable}")
        print(f"  running as  : {_who(uid)}")
        print(f"  Server dir  : {_SERVER_DIR}")
        # Either name means the same thing: mpu6050-raspberrypi imports smbus, so
        # a split install (one at user level, one system-wide) fails on smbus
        # even though the module the server names is mpu6050.
        missing = next((m for m in ("mpu6050", "smbus") if m in str(e)), None)
        if missing:
            print(
                f"\n  Same class of failure the service logs. '{missing}' is visible to\n"
                "  your login shell but not to this interpreter -- nearly always a\n"
                "  --user install that root cannot see. Compare the two:\n"
                f"      python3      -c 'import {missing}; print({missing}.__file__)'\n"
                f"      sudo python3 -c 'import {missing}; print({missing}.__file__)'\n"
                "  If only the first prints a path, install where root can see it:\n"
                "      sudo apt install -y python3-smbus\n"
                "      sudo pip3 install --break-system-packages mpu6050-raspberrypi")
        raise SystemExit(1)


def start_like_the_server(GyroMPU, rate, calibrate, samples):
    """Reproduce ServerNode.__init__'s gyro block verbatim, so a failure here is
    a failure there. The server swallows any exception into a printed warning
    and falls back to encoder heading -- do the same, but say so loudly."""
    print(f"[sim] GyroMPU(sample_rate={rate})")
    try:
        imu = GyroMPU(sample_rate=rate)
    except Exception as e:                                      # noqa: BLE001
        print(f"[gyro] unavailable ({e}); heading falls back to encoders")
        print("[sim] -> the server would run WITHOUT a gyro: turn_in_place() "
              "takes the encoder-distance fallback, not the heading PID.")
        raise SystemExit(1)

    if imu.is_connected():
        if calibrate:
            print("[gyro] calibrating bias -- keep the kart STILL...")
            imu.calibrate(samples=samples)
        else:
            print(f"[sim] --no-calibrate: reusing cached bias {imu.gyro_bias}")
    else:
        print("[gyro] MPU6050 not detected; heading falls back to encoders")
    return imu


def report_mounting(imu, yaw_axis, yaw_sign):
    """Resting roll/pitch -> is the board mounted upside down? The complementary
    filter seeds x/y from gravity, so at rest they ARE the mounting angles."""
    time.sleep(0.3)                       # let the loop take a few samples
    ang = imu.get_angles_gyro()
    roll, pitch = ang["x"], ang["y"]
    print(f"\n[mount] resting roll={roll:+.1f} pitch={pitch:+.1f} deg   "
          f"(yaw axis '{yaw_axis}', YAW_SIGN {yaw_sign:+d})")

    if abs(abs(roll) - 180.0) < 30.0:
        print("[mount] roll is near +/-180: the board is UPSIDE DOWN.")
        print("        Yaw is pure gyro so it still reads, but its sign is very")
        print("        likely flipped -- run --sign-check before any turn.")
        print("        (Inverted, roll also sits on the atan2 wrap, so the x/y")
        print("        angles will look like nonsense. That part is cosmetic;")
        print("        nothing in the drive controller reads them.)")
    elif abs(roll) < 30.0 and abs(pitch) < 30.0:
        print("[mount] board is level and right-way-up.")
    else:
        print("[mount] board is tilted or on its side. If it is mounted on edge,")
        print(f"        yaw may not be on '{yaw_axis}' at all -- check which axis")
        print("        moves when you rotate the kart, and set YAW_AXIS to it.")


def sign_check(imu, yaw_sign):
    """Decide YAW_SIGN by hand. This is the only authoritative test: it works
    whatever the mounting, because it asks the sensor what it reports for a
    rotation whose true direction you control."""
    print("\n" + "=" * 68)
    print("YAW SIGN CHECK")
    print("  1. Put the kart flat on the floor and let go of it.")
    print("  2. Press Enter, rotate it ~90 deg COUNTER-CLOCKWISE (viewed from")
    print("     ABOVE, i.e. to its left), then press Enter again.")
    print("=" * 68)
    input("  Enter to start... ")
    start = imu.get_yaw()
    input("  Now rotate CCW, then press Enter... ")
    delta = imu.get_yaw() - start

    print(f"\n  yaw moved {delta:+.1f} deg for a counter-clockwise rotation")
    if abs(delta) < 20.0:
        print("  [INCONCLUSIVE] too small to judge -- is the sensor reading at all?")
        print("                 Turn further (~90 deg) and rerun.")
        return
    if delta > 0.0:
        print(f"  [OK] CCW reads POSITIVE. YAW_SIGN = {yaw_sign:+d} is correct.")
    else:
        print(f"  [WRONG SIGN] CCW reads NEGATIVE, but heading.py has "
              f"YAW_SIGN = {yaw_sign:+d}.")
        print(f"               Set YAW_SIGN = {-yaw_sign:+d} in Server/heading.py.")
        print("               Until you do, DO NOT command a turn: the heading PID")
        print("               will drive the error the wrong way and spin at full")
        print("               duty until HeadingGains.max_time expires.")


def monitor(imu, hz):
    """Live readout of what the drive controller actually consumes.

    `yaw` is GyroMPU.get_yaw() -- axis and sign already applied -- which is what
    _read_gyro_yaw() converts to radians and feeds to the heading PID and to the
    odometry's d_theta. `drift` is the yaw wander while stationary: yaw has no
    absolute reference (no gravity vector for heading), so bias error integrates
    forever. A few deg/min is normal; tens means the bias calibration is stale
    or the kart was not still when it ran.
    """
    period = 1.0 / max(0.1, hz)
    print(f"\n[monitor] {hz:g} Hz. Ctrl+C to stop.\n")
    t0 = time.monotonic()
    yaw0 = imu.get_yaw()
    try:
        while True:
            elapsed = time.monotonic() - t0
            yaw = imu.get_yaw()
            ang = imu.get_angles_gyro()
            drift = (yaw - yaw0) / elapsed * 60.0 if elapsed > 1.0 else 0.0

            if imu.stoped:
                flag = "DEAD"          # gave up after max_reconect_tries
            elif imu.is_connected():
                flag = "OK  "
            else:
                flag = "DISC"
            print(f"[{flag}] t={elapsed:6.1f}s  yaw={yaw:+8.2f} deg "
                  f"({math.radians(yaw):+7.4f} rad) | "
                  f"x={ang['x']:+7.1f} y={ang['y']:+7.1f} z={ang['z']:+8.2f} | "
                  f"drift={drift:+6.1f} deg/min")

            if imu.stoped:
                print("\n[monitor] GyroMPU gave up reconnecting "
                      f"(max_reconect_tries={imu.max_reconect_tries}). Its thread has")
                print("          exited -- in the server this is silent, and every")
                print("          later turn falls back to the encoder path.")
                return
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[INFO] stopped")


def main():
    ap = argparse.ArgumentParser(
        description="Exercise Server/heading.py's GyroMPU the way the server does.")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="GyroMPU sample_rate in Hz (server uses 50)")
    ap.add_argument("--hz", type=float, default=5.0,
                    help="print rate for the live monitor")
    ap.add_argument("--samples", type=int, default=200,
                    help="calibration samples (server uses the 200 default)")
    ap.add_argument("--no-calibrate", action="store_true",
                    help="skip calibration and reuse the cached bias")
    ap.add_argument("--sign-check", action="store_true",
                    help="interactively determine YAW_SIGN, then monitor")
    args = ap.parse_args()

    GyroMPU, yaw_axis, yaw_sign = import_gyro()
    print(f"[env] {sys.executable}  ({_who(_euid())})")

    imu = start_like_the_server(GyroMPU, args.rate, not args.no_calibrate,
                                args.samples)
    try:
        report_mounting(imu, yaw_axis, yaw_sign)
        if args.sign_check:
            sign_check(imu, yaw_sign)
        monitor(imu, args.hz)
    finally:
        imu.stop()


if __name__ == "__main__":
    main()
