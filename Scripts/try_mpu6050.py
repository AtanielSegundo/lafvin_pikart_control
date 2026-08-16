#!/usr/bin/env python3
"""
Quick hardware check for the MPU6050 IMU.

Reads accelerometer (m/s^2), gyroscope (deg/s) and temperature (C) over I2C and
prints them at a fixed rate until Ctrl+C.

Wiring (Raspberry Pi): VCC->3V3, GND->GND, SDA->GPIO0, SCL->GPIO1. That is the
SECONDARY I2C bus, /dev/i2c-0 -- the primary bus 1 (GPIO2/3) is already used by
other peripherals here. Bus 0 is normally reserved for HAT ID EEPROM, so enable
it first: add `dtparam=i2c_vc=on` to /boot/firmware/config.txt and reboot, then
    i2cdetect -y 0
The MPU6050 shows at 0x68 (or 0x69 if the AD0 pin is pulled high). That address
is the same for the MPU6500/9250-family dies that many "MPU6050" boards actually
carry, so the script probes which register map the part uses and reports it.

Deps:  pip install mpu6050-raspberrypi
"""
import datetime
import time

from mpu6050 import mpu6050

MPU6050_ADDR = 0x68        # 0x69 if AD0 is tied high
# Wired to GPIO0 (SDA) / GPIO1 (SCL) = the secondary I2C bus, /dev/i2c-0.
MPU6050_SDA  = 2           # GPIO0 (physical pin 27) -> SDA0
MPU6050_SCL  = 3           # GPIO1 (physical pin 28) -> SCL0
I2C_BUS      = 1           # /dev/i2c-0  (GPIO0/1)
SAMPLE_HZ    = 2.0         # readings per second

WHO_AM_I      = 0x75
TEMP_OUT0     = 0x41
CONFIG        = 0x1A   # DLPF_CFG[2:0]
ACCEL_CONFIG2 = 0x1D   # A_DLPF_CFG[2:0] -- 6500/9250 only, FF_THRESHOLD on a 6050

# Low-pass filter setting, 0-6 (None leaves the chip at its default).
# Approximate -3 dB bandwidth per code, accel / gyro:
#   0: 260/256 Hz (6050) or 460/250 Hz (6500)   4: 21/20 Hz
#   1: 184/188 Hz                               5: 10/10 Hz
#   2: 94/98 Hz                                 6: 5/5 Hz
#   3: 44/42 Hz
DLPF_CFG = 3

# GY-521 boards ship with several register-compatible dies. Accel/gyro registers
# and scale factors are identical, so the mpu6050 library reads them correctly on
# all of them, but the temperature constants and the filter registers are not the
# same. WHO_AM_I is only a hint here -- IDs like 0x72 are reported both by real
# 9250 silicon and by MPU6050 knock-offs, so the register map gets probed.
PART_NAMES = {
    0x68: "MPU6050/9150", 0x69: "MPU6050 (AD0 in ID)",
    0x70: "MPU6500", 0x71: "MPU9250", 0x72: "MPU6050 clone", 0x73: "MPU9255",
    0x74: "MPU6515", 0x7C: "MPU6555", 0x98: "ICM-20689",
    0xAC: "ICM-20608-G", 0xAF: "ICM-20608-D",
}
XA_OFFSET_H = 0x77   # exists only on the 6500 map; the 6050 map ends at 0x75


def read_word(sensor, register):
    """Signed 16-bit big-endian read, straight off the smbus handle."""
    high = sensor.bus.read_byte_data(sensor.address, register)
    low = sensor.bus.read_byte_data(sensor.address, register + 1)
    value = (high << 8) + low
    return value - 65536 if value >= 0x8000 else value


def probe_6500_register_map(sensor):
    """Which register map the part uses, decided by probing rather than by ID.

    XA_OFFSET_H (0x77) only exists on the 6500 family -- the 6050 map ends at
    WHO_AM_I (0x75), so writes above it are dropped and reads come back 0.
    The original value is restored, since on a real 6500 it is factory trim.
    """
    saved = [sensor.bus.read_byte_data(sensor.address, XA_OFFSET_H + i) for i in (0, 1)]
    pattern = [0x5A, 0xA4]        # bit 0 of the low byte is reserved, keep it 0

    try:
        for i, value in enumerate(pattern):
            sensor.bus.write_byte_data(sensor.address, XA_OFFSET_H + i, value)
        readback = [sensor.bus.read_byte_data(sensor.address, XA_OFFSET_H + i)
                    for i in (0, 1)]
    finally:
        # Must happen even if the readback raises: on a real 6500 these hold
        # per-die factory calibration, and leaving the probe pattern in place
        # would bias every subsequent accelerometer reading.
        for i, value in enumerate(saved):
            sensor.bus.write_byte_data(sensor.address, XA_OFFSET_H + i, value)

    return readback == pattern


def set_dlpf(sensor, cfg, is_6500):
    """Apply the low-pass filter to accel and gyro alike.

    On the 6050, CONFIG[2:0] filters both. On the 6500/9250 die it filters
    gyro and temperature only -- the accelerometer has its own filter in
    ACCEL_CONFIG2, and leaving it alone parks the accel at ~460 Hz no matter
    what CONFIG says. The 0x1D write is gated on the probed register map
    because that register is the free-fall threshold on a 6050.
    """
    if not 0 <= cfg <= 6:
        raise ValueError(f"DLPF_CFG must be 0-6, got {cfg}")

    value = sensor.bus.read_byte_data(sensor.address, CONFIG)
    sensor.bus.write_byte_data(sensor.address, CONFIG, (value & 0xF8) | cfg)

    if is_6500:
        value = sensor.bus.read_byte_data(sensor.address, ACCEL_CONFIG2)
        # Clearing bit 3 (ACCEL_FCHOICE_B) is what enables the filter at all.
        sensor.bus.write_byte_data(sensor.address, ACCEL_CONFIG2, (value & 0xF0) | cfg)


def read_temp_c(sensor, is_6500):
    """The library hardcodes the 6050 formula; the 6500 die uses different
    constants (~15 C too high across the whole useful range if you use the
    6050 formula on a 6500)."""
    raw = read_word(sensor, TEMP_OUT0)
    if is_6500:
        return raw / 333.87 + 21.0
    return raw / 340.0 + 36.53


def main():
    try:
        sensor = mpu6050(MPU6050_ADDR, bus=I2C_BUS)
    except Exception as e:                                     # noqa: BLE001
        print(f"[ERROR] MPU6050 not found at 0x{MPU6050_ADDR:02x} on i2c-{I2C_BUS} ({e})")
        print(f"        Check wiring and `i2cdetect -y {I2C_BUS}`; enable bus 0 with "
              "`dtparam=i2c_vc=on` in config.txt.")
        return

    # The library never checks WHO_AM_I, so a wrong or half-dead chip reads as
    # plausible garbage rather than an error. Do it here.
    try:
        who = sensor.bus.read_byte_data(sensor.address, WHO_AM_I)
        if who in (0x00, 0xFF):
            print(f"[ERROR] WHO_AM_I read back 0x{who:02x} - nothing is driving the bus")
            return
        is_6500 = probe_6500_register_map(sensor)
        if DLPF_CFG is not None:
            set_dlpf(sensor, DLPF_CFG, is_6500)
    except OSError as e:
        print(f"[ERROR] I2C failed during setup ({e})")
        return

    part = PART_NAMES.get(who, "unrecognised part (likely a clone)")

    period = 1.0 / max(0.1, SAMPLE_HZ)
    print(f"{part}, WHO_AM_I 0x{who:02x}, {'6500/9250' if is_6500 else '6050'} register map")
    print(f"0x{MPU6050_ADDR:02x} (i2c-{I2C_BUS}) - {SAMPLE_HZ:g} Hz, DLPF={DLPF_CFG}. "
          "Ctrl+C to stop.")

    try:
        while True:
            try:
                accel, gyro, _ = sensor.get_all_data()
                temp = read_temp_c(sensor, is_6500)
            except OSError as e:                # transient I2C read glitch
                print(f"[WARN] read failed ({e}); retrying...")
                time.sleep(period)
                continue

            ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"[{ts}] "
                  f"accel(m/s^2) x={accel['x']:+7.3f} y={accel['y']:+7.3f} z={accel['z']:+7.3f} | "
                  f"gyro(deg/s) x={gyro['x']:+8.3f} y={gyro['y']:+8.3f} z={gyro['z']:+8.3f} | "
                  f"temp={temp:5.1f}C")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[INFO] stopped")


if __name__ == "__main__":
    main()
