import time
import RPi.GPIO as GPIO
from Command import COMMAND as cmd

Buzzer_Pin = 17
PWM_MAX = 40

# The GPIO setup and ``GPIO.PWM(...).start(0)`` used to run at module level, so
# merely importing Buzzer claimed pin 17 and started a PWM generator. With the
# process split that import happens in P_web (via server.py) as well as in the
# process that actually owns the buzzer (P_aux), and two owners of one pin fight.
# It is now created lazily, on first Buzzer() -- only P_aux does that.
_buzzer_pwm = None
_lock_pin_done = False


def _pwm():
    global _buzzer_pwm, _lock_pin_done
    if _buzzer_pwm is None:
        if not _lock_pin_done:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(Buzzer_Pin, GPIO.OUT)
            _lock_pin_done = True
        _buzzer_pwm = GPIO.PWM(Buzzer_Pin, 1000)
        _buzzer_pwm.start(0)
    return _buzzer_pwm


class Buzzer:
    def run(self, command):
        if command != "0":
            _pwm().ChangeDutyCycle(PWM_MAX)
        else:
            _pwm().ChangeDutyCycle(0)


if __name__ == '__main__':
    B = Buzzer()
    B.run('1')
    time.sleep(3)
    B.run('0')
