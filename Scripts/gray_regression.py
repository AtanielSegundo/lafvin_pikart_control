import os
import time
import csv
import sys

__HERE   = os.path.dirname(os.path.abspath(__file__))
__PARENT = os.path.dirname(__HERE)
__SERVER = os.path.join(__PARENT,"Server")

if __SERVER not in sys.path:
    sys.path.insert(0, __SERVER)

print(f"[PATH ADDED] {__SERVER}")

from odometry   import SkidSteerOdometry
from config     import CONFIG
from encoders   import WheelEncoders
from Motor      import Motor
from Ultrasonic import Ultrasonic

def set_foward_motors_duty(m:Motor,duty_pwm:int):
    m.setMotorModel(duty_pwm,duty_pwm,duty_pwm,duty_pwm)

if __name__ == "__main__":
    
    try:
        from ultrasonic_pigpio import UltrasonicPigpio
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