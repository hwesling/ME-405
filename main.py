from pyb              import Pin, Timer, USB_VCP, ADC
from time             import ticks_diff, ticks_add, ticks_us
from motor            import MotorDriver
from encoder          import Encoder
from taskmotor        import TaskMotor, ClosedLoop
from taskuser         import TaskUser
import cotask
from array            import array
from line_calibration import save_calibration, load_calibration
from taskline         import TaskLine


 
 
class Cell:
    def __init__(self, value = None):
        self.value = value
 
ser = USB_VCP()
n_samples = 101

 
pwm_tim = Timer(1, freq=20_000)

channels = [2, 4, 6, 8, 12, 1, 3, 5, 7, 9, 11, 13]
sensor_pins = (Pin.cpu.C5, Pin.cpu.A6, Pin.cpu.A7, Pin.cpu.B1, Pin.cpu.A0, Pin.cpu.A1, Pin.cpu.A4, Pin.cpu.B0, Pin.cpu.C1, Pin.cpu.C0, Pin.cpu.C2, Pin.cpu.C3)

sensor_adc = tuple(ADC(pin) for pin in sensor_pins)


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


line_kp = Cell(0.05)
base_speed = Cell(4.0)
line_go = Cell(False)

 
#Class Objects
left_mot_task = TaskMotor(left_enc, left_mot, left_controller, "left", l_go, l_done, l_data, left_omega_sp, kp, ki, motor_period_ms, user_period_ms, V_bat, line_go)
right_mot_task = TaskMotor(right_enc, right_mot, right_controller, "right", r_go, r_done, r_data, right_omega_sp, kp, ki, motor_period_ms, user_period_ms, V_bat, line_go)
user_task = TaskUser(l_go, r_go, l_done, r_done, l_data, r_data, ser, left_omega_sp, right_omega_sp, base_speed, line_kp, line_go, kp, ki, motor_period_ms, user_period_ms, V_bat) 



def main():

    # calibration, motors disabled

    left_mot.disable()
    right_mot.disable()

    calibration = None

    if not RECALIBRATE:
        try:

            calibration = load_calibration(CALIBRATION_FILE, channels, MIN_SPAN)

            print("Saved calibration loaded.")
            print("black:", calibration["black"])
            print("white:", calibration["white"])

        except (OSError, ValueError, KeyError, TypeError):
            print("no valid saved calibration: recalibration required")

    if RECALIBRATE or calibration is None:

        wait_for_enter("place all 13 sensors over black, then press enter")
        black_averages = capture_averages()
        print("black averages:", black_averages)
        wait_for_enter("place all sensors over white, then press enter")
        white_averages = capture_averages()
        print("white averages:", white_averages)
        calibration = {"channels": channels, "black": black_averages, "white": white_averages,}
        save_calibration(CALIBRATION_FILE, calibration, channels, MIN_SPAN)
        print("New calibration")


    line_task = TaskLine(read_sensor_values, calibration, SENSOR_POSITIONS_MM, left_omega_sp, right_omega_sp, base_speed, line_kp, line_go, l_go, r_go)


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
       


def wait_for_enter(message):
    print(message)

    while True:
        if ser.any():
            char = ser.read(1)

            if char == b"\r" or char == b"\n":
                return

def capture_averages(samples=20):
    totals = [0] *12

    for _ in range(samples):
        for index, adc in enumerate(sensor_adc):
            totals[index] += adc.read()

    return [total / samples for total in totals]


def get_normalized_weights(calibration):
    readings = capture_averages()

    white = calibration["white"]
    black = calibration["black"]
    weights = [0.0] * 12

    for index in range(12):
        span = black[index] - white[index]

        if span < MIN_SPAN:
            raise ValueError(
                "Bad calibration span at sensor {}".format(index + 1)
            )

        weight = (readings[index] - white[index]) / span

        if weight < 0:
            weight = 0.0
        elif weight > 1:
            weight = 1.0

        weights[index] = weight

    return readings, weights


def get_centroid(calibration):
    readings, weights = get_normalized_weights(calibration)
    sum_wi = sum(weights)

    if sum_wi <= 0.001:
        return None, sum_wi, weights

    sum_yiwi = 0

    for index in range(12):
        sum_yiwi += SENSOR_POSITIONS_MM[index] * weights[index]

    centroid_mm = sum_yiwi / sum_wi
    return centroid_mm, sum_wi, weights

def read_sensor_values():
    return [adc.read() for adc in sensor_adc]

        
 
if __name__ == '__main__':
    main ()
 
