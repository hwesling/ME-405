from pyb import Pin, Timer
from time import sleep_ms


print("Motor Driver")

class MotorDriver:

    def __init__(self, pwm_pin: Pin, dir_pin: Pin, 
                nslp_pin: Pin, tim: Timer, chan: int):

            # Store a copy of each input parameter as an attribute
            self._dir_pin = Pin(dir_pin, mode=Pin.OUT_PP)
            self._nslp_pin = Pin(nslp_pin, mode=Pin.OUT_PP)
            self._pwm_chan = tim.channel(chan,
                                         pin=pwm_pin,
                                         mode=Timer.PWM,
                                         pulse_width_percent=0)
                                          
    def enable(self):
            self._nslp_pin.high()
            
    def disable(self):
            self._nslp_pin.low()
            
    def set_effort(self, effort: float):
            # This function has bugs that you must fix

        if effort >= 0:
                self._dir_pin.low()
        else:
                 self._dir_pin.high()

        self._pwm_chan.pulse_width_percent(abs(effort))

if __name__ == "__main__":

    pwm_tim = Timer(2, freq=20_000)
    left_motor = MotorDriver(Pin.cpu.A0, Pin.cpu.C8, Pin.cpu.C9, pwm_tim, 1)
    right_motor = MotorDriver(Pin.cpu.A1, Pin.cpu.B8, Pin.cpu.B9, pwm_tim, 2)
    
    left_motor.enable()
    right_motor.enable()
    
    left_motor.set_effort(50)
    right_motor.set_effort(50)
    sleep_ms(1000)
    left_motor.set_effort(0)
    right_motor.set_effort(0)
    sleep_ms(1000)
    
    left_motor.set_effort(-50)
    right_motor.set_effort(-50)
    sleep_ms(1000)
    left_motor.set_effort(0)
    right_motor.set_effort(0)

    print("motor done")
