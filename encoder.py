import pyb
from time import ticks_us, ticks_diff   # Use to get dt value in update()

class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''

    def __init__(self, tim, chan_1, chan_2, chA, chB,sign = 1):
        '''Initializes an Encoder object'''
    
        self.position   = 0     # Total accumulated position of the encoder
        self.prev_count = 0     # Counter value from the most recent update
        self.delta      = 0     # Change in count between last two updates
        self.dt         = 0     # Amount of time between last two updates
        self.AutoReload = 65535
        self.sign = sign



        self.tim_n = pyb.Timer(tim, period = self.AutoReload, prescaler = 0)
        self.tim_n.channel(chan_1, pin=chA, mode=pyb.Timer.ENC_AB)
        self.tim_n.channel(chan_2, pin=chB, mode=pyb.Timer.ENC_AB)

        self.prev_count = self.tim_n.counter()
        self.prev_time = ticks_us()


    
    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''
        now = ticks_us()
        count = self.tim_n.counter()
        
        self.data = count - self.prev_count
        if self.data < -(self.AutoReload+1)/2:
            self.data += self.AutoReload + 1
        elif self.data > (self.AutoReload +1)/2:
            self.data -= self.AutoReload +1

        self.delta = self.data * self.sign
        self.position += self.delta
        self.dt = ticks_diff(now, self.prev_time)
        self.prev_time = now
        self.prev_count = count
            
    def get_position(self):
        '''Returns the most recently updated value of position as determined
           within the update() method'''
        return self.position
            
    def get_velocity(self):
        '''Returns a measure of velocity using the the most recently updated
           value of delta as determined within the update() method'''
        if self.dt == 0:
            return 0
        return self.delta/self.dt
    
    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero position'''
        self.position = 0


###################### endocer test ##########################
left_motor = MotorDriver(Pin.cpu.A0, Pin.cpu.C8, Pin.cpu.C9, pwm_tim, 1)
right_motor = MotorDriver(Pin.cpu.A1, Pin.cpu.B8, Pin.cpu.B9, pwm_tim, 2)
 
left_enc = Encoder(3, 1, 2, Pin.cpu.B4, Pin.cpu.B5, sign = -1)
right_enc = Encoder(4, 1, 2, Pin.cpu.B6, Pin.cpu.B7)

 
print("=== Test 1: forward/backward at varying speed ===")
left_motor.enable()
right_motor.enable()
 
for effort in (25, 50, 75):
    left_motor.set_effort(effort)
    right_motor.set_effort(effort)
    for _ in range(10):
        update_encoders()
        sleep_ms(100)
    report("forward %d%%" % effort)
 
for effort in (25, 50, 75):
    left_motor.set_effort(-effort)
    right_motor.set_effort(-effort)
    for _ in range(10):
        update_encoders()
        sleep_ms(100)
    report("backward %d%%" % effort)
 
left_motor.set_effort(0)
right_motor.set_effort(0)
 
print("=== Test 2: enable/disable isolation ===")
left_motor.disable()
right_motor.disable()
sleep_ms(200)
update_encoders()
before = (left_enc.get_position(), right_enc.get_position())
 
left_motor.enable()
right_motor.enable()
sleep_ms(500)
update_encoders()
after = (left_enc.get_position(), right_enc.get_position())
print("position before enable:", before, " after enable:", after)
 
print("=== Test 3: encoder +/- position ===")
left_enc.zero()
right_enc.zero()
 
left_motor.set_effort(40)
right_motor.set_effort(40)
for _ in range(10):
    update_encoders()
    sleep_ms(100)
report("after forward run (expect positive)")
 
left_motor.set_effort(-40)
right_motor.set_effort(-40)
for _ in range(20):
    update_encoders()
    sleep_ms(100)
report("after reversing (expect decreasing / negative)")
 
left_motor.set_effort(0)
right_motor.set_effort(0)
 
print("=== Test 4: timer reload / overflow handling ===")
left_enc.zero()
right_enc.zero()
left_motor.set_effort(60)
right_motor.set_effort(60)
for _ in range(50):
    update_encoders()
    sleep_ms(100)
report("after sustained run through at least one timer wraparound")
left_motor.set_effort(0)
right_motor.set_effort(0)
 
print("=== Test 5/6: motor/encoder pairing and forward sign convention ===")
left_enc.zero()
right_enc.zero()
left_motor.set_effort(30)
right_motor.set_effort(30)
for _ in range(10):
    update_encoders()
    sleep_ms(100)
report("driving forward - both L and R should read positive")
left_motor.set_effort(0)
right_motor.set_effort(0)
 
left_motor.disable()
right_motor.disable()
print("=== Hardware test complete ===")
 

