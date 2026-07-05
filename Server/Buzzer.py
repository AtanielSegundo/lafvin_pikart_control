import time
import RPi.GPIO as GPIO
from Command import COMMAND as cmd

GPIO.setwarnings(False)
Buzzer_Pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(Buzzer_Pin,GPIO.OUT)

buzzer_pwm = GPIO.PWM(Buzzer_Pin, 1000)
buzzer_pwm.start(0)

PWM_MAX = 40

class Buzzer:
    def run(self,command):
        if command!="0":
            buzzer_pwm.ChangeDutyCycle(PWM_MAX)
            # GPIO.output(Buzzer_Pin,True)
        else:
            buzzer_pwm.ChangeDutyCycle(0)
            # GPIO.output(Buzzer_Pin,False)

if __name__=='__main__':
    B=Buzzer()
    B.run('1')
    time.sleep(3)
    B.run('0')