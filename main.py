""" MECHA 15 - Harry """
""" main file for Romi that implements a priority scheduler to cooperatively multitask """

# Imports
import cotask  # type: ignore
from pyb       import Pin, Timer, USB_VCP, UART # type: ignore
from time      import ticks_diff, ticks_add, ticks_us
from motor     import MotorDriver
from encoder   import Encoder
from taskmotor import TaskMotor
from taskuser  import TaskUser
from array     import array

# The Cell class allows Bools of the class to be used across tasks 
class Cell:
    def __init__(self, value = None):
        self.value = value
 
ser = USB_VCP()
 
effort =  Cell(0)
n_samples = 101
 
pwm_tim = Timer(2, freq=20_000)

# MotorDriver and Encoder class objects 
right_mot = MotorDriver(Pin.cpu.A0, Pin.cpu.C8, Pin.cpu.C9, pwm_tim, 1)
left_mot = MotorDriver(Pin.cpu.A1, Pin.cpu.B8, Pin.cpu.B9, pwm_tim, 2)

right_enc = Encoder(3, 1, 2, Pin.cpu.B4, Pin.cpu.B5, sign = -1)
left_enc = Encoder(4, 1, 2, Pin.cpu.B6, Pin.cpu.B7)

# Intertask Variables 
l_go = Cell(False)          
r_go = Cell(False)          
l_done = Cell(False)        
r_done = Cell(False)        
l_data = array('f', [0]*(n_samples*4))
r_data = array('f', [0]*(n_samples*4)) 
 
# TaskMotor and TaskUser class objects
left_mot_task = TaskMotor(left_enc, left_mot, "left", l_go, l_done, l_data, effort)
right_mot_task = TaskMotor(right_enc, right_mot, "right", r_go, r_done, r_data, effort)
user_task = TaskUser(l_go, r_go, l_done, r_done, l_data, r_data, ser, effort) 
 
 
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
 
