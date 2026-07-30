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
The MPU6050 shows at 0x68 (or 0x69 if the AD0 pin is pulled high).

Deps:  pip install mpu6050-raspberrypi
"""
import datetime
import time

from mpu6050 import mpu6050

MPU6050_ADDR = 0x68        # 0x69 if AD0 is tied high
# Wired to GPIO0 (SDA) / GPIO1 (SCL) = the secondary I2C bus, /dev/i2c-0.
MPU6050_SDA  = 0           # GPIO0 (physical pin 27) -> SDA0
MPU6050_SCL  = 1           # GPIO1 (physical pin 28) -> SCL0
I2C_BUS      = 0           # /dev/i2c-0  (GPIO0/1)
SAMPLE_HZ    = 2.0         # readings per second


def main():
    try:
        sensor = mpu6050(MPU6050_ADDR, bus=I2C_BUS)
    except Exception as e:                                     # noqa: BLE001
        print(f"[ERROR] MPU6050 not found at 0x{MPU6050_ADDR:02x} on i2c-{I2C_BUS} ({e})")
        print(f"        Check wiring and `i2cdetect -y {I2C_BUS}`; enable bus 0 with "
              "`dtparam=i2c_vc=on` in config.txt.")
        return

    period = 1.0 / max(0.1, SAMPLE_HZ)
    print(f"MPU6050 @ 0x{MPU6050_ADDR:02x} (i2c-{I2C_BUS}) - {SAMPLE_HZ:g} Hz. Ctrl+C to stop.")

    try:
        while True:
            try:
                accel, gyro, temp = sensor.get_all_data()
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
