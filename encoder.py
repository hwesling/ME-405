""" MECHA 15 - Romi """

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
