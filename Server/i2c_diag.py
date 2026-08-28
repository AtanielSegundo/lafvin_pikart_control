#!/usr/bin/python3
"""
I2C / PCA9685 diagnostic script. Run directly on the Pi:

    python3 i2c_diag.py [bus] [address]

Defaults: bus=1, address=0x40 (PCA9685).
Does not require the rest of this codebase - only smbus/smbus2.
"""

import sys
import time
import errno
import subprocess

try:
    import smbus2 as smbus
except ImportError:
    import smbus


def hexaddr(a):
    return "0x{:02X}".format(a)


def run_i2cdetect(bus_num):
    print("== i2cdetect -y {} ==".format(bus_num))
    try:
        out = subprocess.run(
            ["i2cdetect", "-y", str(bus_num)],
            capture_output=True, text=True, timeout=10,
        )
        print(out.stdout.strip() or out.stderr.strip())
    except FileNotFoundError:
        print("i2cdetect not found. Install with: sudo apt install i2c-tools")
    except Exception as e:
        print("i2cdetect failed: {}".format(e))
    print()


def software_scan(bus_num):
    """Fallback bus scan using smbus directly, in case i2c-tools isn't installed."""
    print("== software scan (smbus) ==")
    try:
        bus = smbus.SMBus(bus_num)
    except Exception as e:
        print("Could not open /dev/i2c-{}: {}".format(bus_num, e))
        print("Check: is I2C enabled? (raspi-config -> Interface Options -> I2C)")
        return []

    found = []
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            found.append(addr)
        except OSError as e:
            if e.errno == errno.EREMOTEIO or e.errno == 121:
                pass  # NACK, nothing there - expected for empty addresses
            elif e.errno == errno.ETIMEDOUT or e.errno == 110:
                print("  {} -> TIMEOUT (bus may be hung/stuck)".format(hexaddr(addr)))
            else:
                pass
    bus.close()
    if found:
        print("Devices found: {}".format(", ".join(hexaddr(a) for a in found)))
    else:
        print("No devices responded (ACK) on this bus.")
    print()
    return found


def test_target(bus_num, address):
    print("== targeted test: bus {} address {} ==".format(bus_num, hexaddr(address)))
    try:
        bus = smbus.SMBus(bus_num)
    except Exception as e:
        print("FAIL: could not open /dev/i2c-{}: {}".format(bus_num, e))
        return

    # 1) Quick presence check (read)
    try:
        val = bus.read_byte(address)
        print("PASS: quick read ok, got byte 0x{:02X}".format(val))
    except OSError as e:
        diagnose_oserror("quick read", e)

    # 2) MODE1 register read (0x00) - PCA9685 specific
    try:
        val = bus.read_byte_data(address, 0x00)
        print("PASS: read MODE1 register (0x00) = 0x{:02X}".format(val))
    except OSError as e:
        diagnose_oserror("read MODE1 register", e)

    # 3) Write to MODE1 (what the failing code path does: write(0x00, 0x00))
    try:
        bus.write_byte_data(address, 0x00, 0x00)
        print("PASS: write to MODE1 register succeeded")
    except OSError as e:
        diagnose_oserror("write MODE1 register", e)

    bus.close()
    print()


def diagnose_oserror(step, e):
    code = e.errno
    print("FAIL: {} -> errno {} ({})".format(step, code, e.strerror))
    if code in (errno.ETIMEDOUT, 110):
        print(
            "  -> Connection timed out: the Pi got no response at all from the device.\n"
            "     Most likely causes:\n"
            "       - PCA9685 not powered (check VCC logic pin AND V+ servo rail)\n"
            "       - SDA/SCL wiring loose, swapped, or disconnected\n"
            "       - I2C bus stuck (a device left SDA held low) - try powering\n"
            "         everything off (Pi + PCA9685) for ~10s, then back on\n"
            "       - Service started before power rail was stable at boot"
        )
    elif code in (errno.EREMOTEIO, 121):
        print(
            "  -> Remote I/O error: something answered on the bus but rejected\n"
            "     the transaction (wrong address, or device in a bad state)."
        )
    elif code == errno.ENXIO:
        print("  -> No such device or address: nothing at this address.")
    else:
        print("  -> Unrecognized errno, see strerror above.")


def main():
    bus_num = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    address = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0x40

    print("I2C diagnostic - bus={} target={}\n".format(bus_num, hexaddr(address)))

    run_i2cdetect(bus_num)
    found = software_scan(bus_num)
    test_target(bus_num, address)

    print("== summary ==")
    if address in found:
        print("{} responded during scan - if the targeted write test above also".format(hexaddr(address)))
        print("passed, the bus/device is currently healthy (issue may have been transient/boot-time).")
    else:
        print("{} did NOT respond during scan.".format(hexaddr(address)))
        print("Check power to the PCA9685 (VCC + V+) and the SDA/SCL wiring before anything else.")


if __name__ == "__main__":
    main()
