from pyb import Pin, Timer, USB_VCP
from time import ticks_diff, ticks_add, ticks_us
from motor import MotorDriver
from encoder import Encoder
from taskmotor_gen import TaskMotor
from taskuser import TaskUser
import cotask
from array import array
 
 
class Cell:
    def __init__(self, value = None):
        self.value = value
 
ser = USB_VCP()
 
 
trial_set =  [33]
n_samples = 101*len(trial_set)
 
pwm_tim = Timer(2, freq=20_000)
 
right_mot = MotorDriver(Pin.cpu.A0, Pin.cpu.C8, Pin.cpu.C9, pwm_tim, 1)
left_mot = MotorDriver(Pin.cpu.A1, Pin.cpu.B8, Pin.cpu.B9, pwm_tim, 2)
 
right_enc = Encoder(3, 1, 2, Pin.cpu.B4, Pin.cpu.B5, sign = -1)
left_enc = Encoder(4, 1, 2, Pin.cpu.B6, Pin.cpu.B7)
 
l_go = Cell(False)          
r_go = Cell(False)          
l_done = Cell(False)        
r_done = Cell(False)        
l_data = array('f', [0]*(n_samples*4))
r_data = array('f', [0]*(n_samples*4))
 
 
# TaskMotor and TaskUser class Objects
left_mot_task = TaskMotor(left_enc, left_mot, trial_set, "left", l_go, l_done, l_data)
right_mot_task = TaskMotor(right_enc, right_mot, trial_set, "right", r_go, r_done, r_data)
user_task = TaskUser(l_go, r_go, l_done, r_done, l_data, r_data, ser) 
 
 
def main():
   
    cotask.task_list.append(cotask.Task(left_mot_task.run(), name = "left motor task", priority = 1, profile = False, period = 10))
    cotask.task_list.append(cotask.Task(right_mot_task.run(), name = "right motor task", priority = 2, profile = False, period = 10))
    cotask.task_list.append(cotask.Task(user_task.run(), name = "user task", priority = 0, profile = False, period = 10))
 
 
    try: 
        while True:
           
            cotask.task_list.pri_sched()
 
 
    except KeyboardInterrupt:
        pass
 
    finally:
## cleanup/shutdown code
        left_mot.disable()
        right_mot.disable()
 
        
        #print(cotask.task_list.profile())
       

 
 
        
 
if __name__ == '__main__':
    main ()
 
