""" MECHA 15 - ROMI """
"""Integrated application: motor tasks, user task, line task, and BNO055."""

import cotask
from array import array
from pyb import Pin, Timer, USB_VCP, ADC, I2C

from motor import MotorDriver
from encoder import Encoder
from taskmotor import TaskMotor, ClosedLoop
from taskuser import TaskUser
from taskline_ca import TaskLineCA
from imu_driver import BNO055
from imu_calibration import load_calibration as load_imu_calibration
from imu_calibration import calibrate_imu


class Cell:
    def __init__(self, value=None):
        self.value = value


ser = USB_VCP()
n_samples = 101
pwm_tim = Timer(1, freq=20_000)

channels = [2, 4, 6, 8, 12, 1, 3, 5, 7, 9, 11, 13]
sensor_pins = (
    Pin.cpu.C5, Pin.cpu.A6, Pin.cpu.A7, Pin.cpu.B1,
    Pin.cpu.A0, Pin.cpu.A1, Pin.cpu.A4, Pin.cpu.B0,
    Pin.cpu.C1, Pin.cpu.C0, Pin.cpu.C2, Pin.cpu.C3,
)
sensor_adc = [ADC(pin) for pin in sensor_pins]

CALIBRATION_FILE = "line_calibration.json"
IMU_CALIBRATION_FILE = "imu_calibration.json"
MIN_SPAN = 100
SENSOR_POSITIONS_MM = (24, 20, 16, 12, 4, 0, -4, -8, -12, -16, -20, -24)

right_mot = MotorDriver(Pin.cpu.A8, Pin.cpu.C8, Pin.cpu.C9, pwm_tim, 1)
left_mot = MotorDriver(Pin.cpu.A9, Pin.cpu.B8, Pin.cpu.B9, pwm_tim, 2)
right_enc = Encoder(3, 1, 2, Pin.cpu.B4, Pin.cpu.B5, sign=-1)
left_enc = Encoder(8, 1, 2, Pin.cpu.C6, Pin.cpu.C7)

V_bat = 7.2
motor_period_ms = 20
user_period_ms = 0
line_period_ms = 20

left_controller = ClosedLoop(-V_bat, V_bat)
right_controller = ClosedLoop(-V_bat, V_bat)

left_omega_sp = Cell(0.0)
right_omega_sp = Cell(0.0)
base_speed = Cell(5.0)
kp = Cell(0.3)
ki = Cell(0.5)
line_kp = Cell(0.5)

line_go = Cell(False)
l_go = Cell(False)
r_go = Cell(False)
l_done = Cell(False)
r_done = Cell(False)

# These two flags are only for the complete M-command lane-change maneuver.
man_go = Cell(False)
man_done = Cell(False)

l_data = array('f', [0] * (n_samples * 4))
r_data = array('f', [0] * (n_samples * 4))
calibration = Cell(None)

left_mot_task = TaskMotor(
    left_enc, left_mot, left_controller, "left", l_go, l_done, l_data,
    left_omega_sp, kp, ki, motor_period_ms, user_period_ms, V_bat, line_go
)
right_mot_task = TaskMotor(
    right_enc, right_mot, right_controller, "right", r_go, r_done, r_data,
    right_omega_sp, kp, ki, motor_period_ms, user_period_ms, V_bat, line_go
)
user_task = TaskUser(
    l_go, r_go, l_done, r_done, l_data, r_data, ser,
    left_omega_sp, right_omega_sp, base_speed, line_kp, line_go,
    kp, ki, motor_period_ms, user_period_ms, V_bat, sensor_adc,
    calibration, channels, CALIBRATION_FILE, MIN_SPAN, man_go, man_done
)


def startup_imu(i2c, imu):
    """Configure and calibrate before the scheduler is started."""
    if BNO055.DEV_ADDR not in i2c.scan():
        raise RuntimeError("BNO055 not found on I2C bus")
    if imu.chip_id() != 0xA0:
        raise RuntimeError("Unexpected BNO055 chip ID")

    imu.configure()
    try:
        imu_calibration = load_imu_calibration(
            IMU_CALIBRATION_FILE, BNO055.IMUPLUS_MODE, BNO055.UNITS,
            BNO055.AXIS_MAP_CONFIG, BNO055.AXIS_MAP_SIGN
        )
        imu.restore_calibration_profile(imu_calibration["profile"])
        print("Saved IMU calibration loaded.")
    except (OSError, ValueError, KeyError, TypeError):
        print("No valid saved IMU calibration.")
        calibrate_imu(
            imu, IMU_CALIBRATION_FILE, BNO055.IMUPLUS_MODE, BNO055.UNITS,
            BNO055.AXIS_MAP_CONFIG, BNO055.AXIS_MAP_SIGN
        )


def main():
    # IMU configuration/calibration may block, so it happens before scheduling.
    left_mot.disable()
    right_mot.disable()

    i2c = I2C(2, I2C.CONTROLLER, baudrate=400_000)
    imu = BNO055(i2c)
    startup_imu(i2c, imu)

    # imu now exists, so construct the task here rather than at global scope.
    line_task = TaskLineCA(user_task.read_sensor_values, calibration, SENSOR_POSITIONS_MM,
                      left_omega_sp, right_omega_sp, base_speed, line_kp,
                      line_go, l_go, r_go, imu, left_enc, right_enc, man_go, man_done)

    cotask.task_list.append(cotask.Task(
        left_mot_task.run(), name="left motor task", priority=2,
        profile=True, period=motor_period_ms
    ))
    cotask.task_list.append(cotask.Task(
        right_mot_task.run(), name="right motor task", priority=2,
        profile=True, period=motor_period_ms
    ))
    cotask.task_list.append(cotask.Task(
        user_task.run(), name="user task", priority=0,
        profile=True, period=user_period_ms
    ))
    cotask.task_list.append(cotask.Task(
        line_task.run(), name="line task", priority=1,
        profile=True, period=line_period_ms
    ))

    try:
        while True:
            cotask.task_list.pri_sched()
    except KeyboardInterrupt:
        pass
    finally:
        line_go.value = False
        man_go.value = False
        l_go.value = False
        r_go.value = False
        left_mot.disable()
        right_mot.disable()
        print(cotask.task_list.profile())


if __name__ == "__main__":
    main()
