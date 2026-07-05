# {PHASE_A}{PHASE_B} = {0|1}{0|1} => EG: 00,01,10,11
#       novo→  00   01   10   11
# ant 00 (0):    0,  -1,   1,   0
# ant 01 (1):    1,   0,   0,  -1
# ant 10 (2):   -1,   0,   0,   1
# ant 11 (3):    0,   1,  -1,   0
QUAD_TABLE = [
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0,
]

# MOTOR TAG : (PIN_PHASE_A,PIN_PHASE_B)
HARDWARE_ENCODERS_CONNECTION = {
    "M1" : (12,13), #PIN_TAGS: SERV0_4, SERVO_5
    "M2" : (26,20),
    "M3" : (19,16),
    "M4" : (10,11)  #PIN_TAGS: SERV0_2, SERVO_3,
}

class Encoder:
    def __init__(self,pin_phase_a,pin_phase_b):
        self.phase_a = pin_phase_a
        self.phase_b = pin_phase_b
        
        