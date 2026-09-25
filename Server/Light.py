import time
from Motor import *
from ADC import *

# The original loop had no sleep. It is rate-limited in practice by two I2C ADC
# reads per iteration (each of which itself loops until two reads agree), but a
# small sleep makes that explicit and bounded.
LIGHT_POLL_S = 0.02         # 50 Hz


class Light:
    def run(self, motor=None, adc=None, stop_evt=None):
        """Legacy photoresistor-following mode.

        ``motor`` / ``adc`` are injected: in P_aux the motor is a RemoteMotor
        forwarding duties to P_control (sole PCA9685 owner) and the ADC is read
        there rather than opening a second smbus client. ``stop_evt`` replaces
        the ctypes stop_thread() kill.
        """
        try:
            self.adc = adc if adc is not None else Adc()
            self.PWM = motor if motor is not None else Motor()
            self.PWM.setMotorModel(0,0,0,0)
            while stop_evt is None or not stop_evt.is_set():
                L = self.adc.recvADC(0)
                R = self.adc.recvADC(1)
                if L < 2.99 and R < 2.99 :
                    self.PWM.setMotorModel(800,800,800,800)
                elif abs(L-R)<0.15:
                    self.PWM.setMotorModel(0,0,0,0)

                elif L > 3 or R > 3:
                    if L > R :
                        self.PWM.setMotorModel(-1200,-1200,1400,1400)

                    elif R > L :
                        self.PWM.setMotorModel(1400,1400,-1200,-1200)

                time.sleep(LIGHT_POLL_S)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                self.PWM.setMotorModel(0, 0, 0, 0)
            except Exception:
                pass

if __name__=='__main__':
    print ('Program is starting ... ')
    led_Car=Light()
    led_Car.run()
