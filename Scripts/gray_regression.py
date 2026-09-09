import os
import time
import csv
import sys

__HERE   = os.path.dirname(__file__)
__PARENT = os.path.dirname(__HERE)
__SERVER = os.path.join(__PARENT,"Server")

sys.path.append(__PARENT)
sys.path.append(__SERVER)

from Server.odometry   import SkidSteerOdometry
from Server.config     import CONFIG
from Server.encoders   import WheelEncoders
from Server.Motor      import Motor
from Server.Ultrasonic import Ultrasonic

def set_foward_motors_duty(m:Motor,duty_pwm:int):
    m.setMotorModel(duty_pwm,duty_pwm,duty_pwm,duty_pwm)

if __name__ == "__main__":
    
    try:
        from Server.ultrasonic_pigpio import UltrasonicPigpio
        ultrasonic = UltrasonicPigpio()
        print("[ultrasonic] using pigpio (hardware-timed echo)")
    except Exception as e:
        print(f"[ultrasonic] pigpio unavailable ({e}); RPi.GPIO fallback")
        ultrasonic = Ultrasonic()
        
    motor = Motor()
    encoders = WheelEncoders(CONFIG.sides)
    
    DUTY_TEST = 2048 
    set_foward_motors_duty(motor,DUTY_TEST)
    
    time.sleep(1)
    set_foward_motors_duty(motor,0)