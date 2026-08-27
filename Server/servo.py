from PCA9685 import PCA9685


class Servo:
    def __init__(self):
        self.PwmServo = PCA9685(0x40, debug=True)
        self.PwmServo.setPWMFreq(50)
        #self.PwmServo.setServoPulse(8, 1500)
        #self.PwmServo.setServoPulse(9, 1500)
        # channel(str) -> last angle actually commanded. There is no hardware
        # readback on these servos, so this is the closest thing to a real
        # position: it lets telemetry report what the camera pan/tilt is
        # ACTUALLY at, instead of a client (e.g. the CoppeliaSim mirror) only
        # ever guessing from whatever it last sent.
        self.angles = {}
        # 设置舵机到中间位置（假设90度是中间位置）
        self.setServoPwm('0', 90)
        self.setServoPwm('1', 37)   # tilt -- measured camera-centered value
        self.setServoPwm('5', 50)   # pan  -- measured camera-centered value
        # 添加一个小的延迟，让舵机有时间移动到位
        import time
        time.sleep(0.5)



    def setServoPwm(self, channel, angle, error=10):
        angle = int(angle)
        if channel == '0':
            self.PwmServo.setServoPulse(8, 500 + int((angle + error) / 0.09))
        elif channel == '1':
            self.PwmServo.setServoPulse(9, 500 + int((angle + error) / 0.09))

        #! UNABLED FOR SAFETY SEE ENCODER.PY
        # elif channel == '2':
        #     self.PwmServo.setServoPulse(10, 500 + int((angle + error) / 0.09))
        # elif channel == '3':
        #     self.PwmServo.setServoPulse(11, 500 + int((angle + error) / 0.09))
        # elif channel == '4':
        #     self.PwmServo.setServoPulse(12, 500 + int((angle + error) / 0.09))
        elif channel == '5':
            self.PwmServo.setServoPulse(13, 500 + int((angle + error) / 0.09))

        elif channel == '6':
            self.PwmServo.setServoPulse(14, 500 + int((angle + error) / 0.09))
        elif channel == '7':
            self.PwmServo.setServoPulse(15, 500 + int((angle + error) / 0.09))
        else:
            return          # unknown/disabled channel -- nothing to record
        self.angles[channel] = angle


# Main program logic follows:
if __name__ == '__main__':
    print("Now servos will rotate to 90°.")
    print("If they have already been at 90°, nothing will be observed.")
    print("Please keep the program running when installing the servos.")
    print("After that, you can press ctrl-C to end the program.")
    pwm = Servo()
    while True:
        try:
            pwm.setServoPwm('0', 90)
            pwm.setServoPwm('1', 90)
        except KeyboardInterrupt:
            print("\nEnd of program")
            break
