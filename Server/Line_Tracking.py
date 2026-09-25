import time
from Motor import *
import RPi.GPIO as GPIO

# Poll period for the legacy line-following loop. The original ``while True`` had
# NO sleep at all, so it pinned a core at 100% for as long as the mode was
# active -- with the single-process design that meant it starved the 20 Hz
# control loop through the GIL. The IR sensors and the motor mix have nothing to
# say faster than this.
LINE_POLL_S = 0.02          # 50 Hz


class Line_Tracking:
    def __init__(self):
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IR01,GPIO.IN)
        GPIO.setup(self.IR02,GPIO.IN)
        GPIO.setup(self.IR03,GPIO.IN)

    def read(self):
        """Current sensor bits as (left, middle, right), each 0 or 1.

        Used by P_sensors to publish the line state for telemetry, so the web
        layer no longer reads GPIO itself (Server.sendLine used to).
        """
        return (1 if GPIO.input(self.IR01) else 0,
                1 if GPIO.input(self.IR02) else 0,
                1 if GPIO.input(self.IR03) else 0)

    def run(self, motor=None, stop_evt=None):
        """Legacy line-following mode.

        ``motor`` is injected: in P_aux this is a RemoteMotor that forwards
        duties to P_control, the only process allowed to touch the PCA9685. The
        old code drove the module-level ``PWM`` global from Motor.py, which is
        exactly the import-time singleton that cannot be shared across
        processes. ``stop_evt`` replaces the ctypes stop_thread() kill.
        """
        PWM = motor if motor is not None else Motor()
        while stop_evt is None or not stop_evt.is_set():
            self.LMR=0x00
            if GPIO.input(self.IR01)==True:
                self.LMR=(self.LMR | 4)
            if GPIO.input(self.IR02)==True:
                self.LMR=(self.LMR | 2)
            if GPIO.input(self.IR03)==True:
                self.LMR=(self.LMR | 1)

            if self.LMR==2:
                PWM.setMotorModel(1000,1000,1000,1000)
            elif self.LMR==6:
                PWM.setMotorModel(-1100,-1100,1100,1100)
            elif self.LMR==4:
                PWM.setMotorModel(-1300,-1300,1300,1300)
            elif self.LMR==3:
                PWM.setMotorModel(1100,1100,-1100,-1100)
            elif self.LMR==1:
                PWM.setMotorModel(1300,1300,-1300,-1300)
            elif self.LMR==0:
                PWM.setMotorModel(0,0,0,0)
            #elif self.LMR==5:
                #PWM.setMotorModel(-800,-800,-800,-800)
            elif self.LMR==7:
                #pass
                PWM.setMotorModel(0,0,0,0)

            time.sleep(LINE_POLL_S)
        PWM.setMotorModel(0, 0, 0, 0)


# No module-level ``infrared = Line_Tracking()``: that claimed GPIO 14/15/23 on
# import, in every process.

# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting ... ')
    infrared = Line_Tracking()
    PWM = Motor()
    try:
        infrared.run(motor=PWM)
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program  will be  executed.
        PWM.setMotorModel(0,0,0,0)
    except Exception as e:
        # 捕获其他所有类型的异常
        print(f'An error occurred: {e}')
    finally:
        # 无论是否发生异常，都会执行的代码
        PWM.setMotorModel(0,0,0,0)
        print('Motor model has been set to stop state.')
