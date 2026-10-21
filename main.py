from pyb              import Pin, Timer, USB_VCP, ADC
from time             import ticks_diff, ticks_add, ticks_us
from motor            import MotorDriver
from encoder          import Encoder
from taskmotor        import TaskMotor, ClosedLoop
from taskuser         import TaskUser
import cotask
from array            import array
from taskline         import TaskLine


 
 
class Cell:
    def __init__(self, value = None):
        self.value = value
 
ser = USB_VCP()
n_samples = 101

 
pwm_tim = Timer(1, freq=20_000)

channels = [2, 4, 6, 8, 12, 1, 3, 5, 7, 9, 11, 13]
sensor_pins = (Pin.cpu.C5, Pin.cpu.A6, Pin.cpu.A7, Pin.cpu.B1, Pin.cpu.A0, Pin.cpu.A1, Pin.cpu.A4, Pin.cpu.B0, Pin.cpu.C1, Pin.cpu.C0, Pin.cpu.C2, Pin.cpu.C3)

sensor_adc = [ADC(pin) for pin in sensor_pins]


CALIBRATION_FILE = "line_calibration.json"
MIN_SPAN = 100
RECALIBRATE = False

SENSOR_POSITIONS_MM = (24, 20, 16, 12, 4, 0, -4, -8, -12, -16, -20, -24)



#Driver class objects
right_mot = MotorDriver(Pin.cpu.A8, Pin.cpu.C8, Pin.cpu.C9, pwm_tim, 1)
left_mot = MotorDriver(Pin.cpu.A9, Pin.cpu.B8, Pin.cpu.B9, pwm_tim, 2)
 
right_enc = Encoder(3, 1, 2, Pin.cpu.B4, Pin.cpu.B5, sign = -1)
left_enc = Encoder(8, 1, 2, Pin.cpu.C6, Pin.cpu.C7)


#Closed-loop controller setup
V_bat = 7.2

motor_period_ms = 50
user_period_ms = 0

left_controller = ClosedLoop(-V_bat, V_bat)
right_controller = ClosedLoop(-V_bat, V_bat)


#Shared variables changed by TaskUser
left_omega_sp = Cell(0.0)
right_omega_sp = Cell(0.0)
kp = Cell(0.0)
ki = Cell(0.0)
l_go = Cell(False)          
r_go = Cell(False)          
l_done = Cell(False)        
r_done = Cell(False)        
l_data = array('f', [0]*(n_samples*4))
r_data = array('f', [0]*(n_samples*4))
calibration = Cell(None)


line_kp = Cell(0.05)
base_speed = Cell(4.0)
line_go = Cell(False)

 
#Class Objects
left_mot_task = TaskMotor(left_enc, left_mot, left_controller, "left", l_go, l_done, l_data, left_omega_sp, kp, ki, motor_period_ms, user_period_ms, V_bat, line_go)
right_mot_task = TaskMotor(right_enc, right_mot, right_controller, "right", r_go, r_done, r_data, right_omega_sp, kp, ki, motor_period_ms, user_period_ms, V_bat, line_go)
user_task = TaskUser(l_go, r_go, l_done, r_done, l_data, r_data, ser, left_omega_sp, right_omega_sp, base_speed, line_kp, line_go, kp, ki, motor_period_ms, user_period_ms, V_bat, sensor_adc, calibration, channels, CALIBRATION_FILE, MIN_SPAN) 
line_task = TaskLine(user_task.read_sensor_values, calibration, SENSOR_POSITIONS_MM, left_omega_sp, right_omega_sp, base_speed, line_kp, line_go, l_go, r_go)



def main():

    # calibration, motors disabled

    left_mot.disable()
    right_mot.disable()





    cotask.task_list.append(cotask.Task(left_mot_task.run(), name = "left motor task", priority = 2, profile = True, period = motor_period_ms))
    cotask.task_list.append(cotask.Task(right_mot_task.run(), name = "right motor task", priority = 2, profile = True, period = motor_period_ms))
    cotask.task_list.append(cotask.Task(user_task.run(), name = "user task", priority = 0, profile = True, period = user_period_ms))
    cotask.task_list.append(cotask.Task(line_task.run(), name = " line task", priority = 1, profile = True, period = 30))
 
    try: 
        while True:
           
            cotask.task_list.pri_sched()
 
 
    except KeyboardInterrupt:
        pass
 
    finally:
## cleanup/shutdown code
        left_mot.disable()
        right_mot.disable()
 
        
        print(cotask.task_list.profile())
       
 
if __name__ == '__main__':
    main ()
 
