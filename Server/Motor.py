import math
import threading
from PCA9685 import PCA9685
from ADC import *
import time


class Motor:
    """Skid-steer motor driver over the PCA9685.

    Implemented as a process-wide singleton: there is exactly one PCA9685 board
    and one ADC, so every ``Motor()`` call (here, and in Light/Ultrasonic/
    Line_Tracking) shares the same hardware instance instead of re-initialising
    the I2C driver several times.
    """

    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
            return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)
        self.time_proportion = 2.5  # Depend on your own car. If you want to get
        # the best out of the rotation mode, tune this by experimenting.
        self.adc = Adc()
        self._rotate_stop = threading.Event()
        self._initialized = True

    @staticmethod
    def duty_range(duty1, duty2, duty3, duty4):
        def clamp(d):
            if d > 4095:
                return 4095
            if d < -4095:
                return -4095
            return d
        return clamp(duty1), clamp(duty2), clamp(duty3), clamp(duty4)

    def left_Upper_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(0, 0)
            self.pwm.setMotorPwm(1, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(1, 0)
            self.pwm.setMotorPwm(0, abs(duty))
        else:
            self.pwm.setMotorPwm(0, 4095)
            self.pwm.setMotorPwm(1, 4095)

    def left_Lower_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(3, 0)
            self.pwm.setMotorPwm(2, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(2, 0)
            self.pwm.setMotorPwm(3, abs(duty))
        else:
            self.pwm.setMotorPwm(2, 4095)
            self.pwm.setMotorPwm(3, 4095)

    def right_Upper_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(7, 0)
            self.pwm.setMotorPwm(6, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(6, 0)
            self.pwm.setMotorPwm(7, abs(duty))
        else:
            self.pwm.setMotorPwm(6, 4095)
            self.pwm.setMotorPwm(7, 4095)

    def right_Lower_Wheel(self, duty):
        if duty > 0:
            self.pwm.setMotorPwm(4, 0)
            self.pwm.setMotorPwm(5, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(5, 0)
            self.pwm.setMotorPwm(4, abs(duty))
        else:
            self.pwm.setMotorPwm(4, 4095)
            self.pwm.setMotorPwm(5, 4095)

    def setMotorModel(self, duty1, duty2, duty3, duty4):
        duty1, duty2, duty3, duty4 = self.duty_range(duty1, duty2, duty3, duty4)
        self.left_Upper_Wheel(duty1)
        self.left_Lower_Wheel(duty2)
        self.right_Upper_Wheel(duty3)
        self.right_Lower_Wheel(duty4)

    def stop(self):
        self.setMotorModel(0, 0, 0, 0)

    def stop_rotate(self):
        """Cooperatively stop a running :meth:`Rotate` loop."""
        self._rotate_stop.set()

    def Rotate(self, n):
        """Spin the car continuously, sweeping the heading by 5 degrees/step.

        Runs until :meth:`stop_rotate` is called (cooperative, unlike the old
        version that relied on injecting an async exception into the thread).
        """
        self._rotate_stop.clear()
        angle = n
        adc_reading = self.adc.recvADC(2) * 3
        # Guard against a zero/low ADC reading (was a divide-by-zero).
        bat_compensate = 7.5 / adc_reading if adc_reading > 0.1 else 1.0
        while not self._rotate_stop.is_set():
            W = 2000
            VY = int(2000 * math.cos(math.radians(angle)))
            VX = -int(2000 * math.sin(math.radians(angle)))

            FR = VY - VX + W
            FL = VY + VX - W
            BL = VY - VX - W
            BR = VY + VX + W

            self.setMotorModel(FL, BL, FR, BR)   # use this instance, not a global
            # Interruptible sleep so stop_rotate() takes effect promptly.
            self._rotate_stop.wait(5 * self.time_proportion * bat_compensate / 1000)
            angle -= 5
        self.setMotorModel(0, 0, 0, 0)


PWM = Motor()


def loop():
    PWM.setMotorModel(2000, 2000, 2000, 2000)  # Forward
    time.sleep(3)
    PWM.setMotorModel(-2000, -2000, -2000, -2000)  # Back
    time.sleep(3)
    PWM.setMotorModel(-500, -500, 2000, 2000)  # Left
    time.sleep(3)
    PWM.setMotorModel(2000, 2000, -500, -500)  # Right
    time.sleep(3)
    PWM.setMotorModel(0, 0, 0, 0)  # Stop


def destroy():
    PWM.setMotorModel(0, 0, 0, 0)


if __name__ == '__main__':
    try:
        loop()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, destroy() is executed.
        destroy()
