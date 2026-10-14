""" MECHA 15 - Harry """
""" A user interface task for the Romi robot, implemented as a FSM """

import sys
from pyb       import Pin, Timer, USB_VCP, UART # type: ignore

help_menu =  (
    "+------------------------------------------------------------------------------+",
    "| ME 405 Romi Tuning Interface Help Menu                                       |",
    "+-----+------------------------------------------------------------------------+",
    "| h/H | Print help menu                                                        |",
    "| s/S | Enter velocity setpoint [rad/s],                                       |",
    "| p/P | Enter proportional gain (Kp) [V/(rad/s)]                               |",
    "| i/I | Enter integral gain (Ki) [V/rad]                                       |",
    "| l/L | Trigger left motor closed-loop response                                |",
    "| r/R | Trigger right motor closed-loop response                               |",
    "| x/X | Abort the active test                                                  |",
    "| e/E | Exit program                                                           |",
    "+-----+------------------------------------------------------------------------+",
)

# Used for printing encoder data
ROWS_PER_PASS = 4
CHARS_PER_PASS = 16

# List of states
S0_INIT = 0
S1_WAIT_FOR_CHAR = 1
S2_PRINT_HELP = 2
S3_LEFT_MOT = 3
S4_RIGHT_MOT = 4
S5_PRINT_RESULTS = 5
S6_VELOCITY_SP_INPUT = 6
S7_KP_INPUT = 7
S8_KI_INPUT = 8


class TaskUser:
    def __init__(self, l_go, r_go, l_done, r_done, l_status, r_status, l_count, r_count, l_data, r_data, vcp, omega_sp, Kp, Ki, motor_period_ms, user_period_ms, n_samples, V_batt):

        self.l_go = l_go
        self.r_go = r_go
        self.l_done = l_done
        self.r_done = r_done
        self.l_status = l_status
        self.r_status = r_status
        self.l_count = l_count
        self.r_count = r_count
        self.l_data = l_data
        self.r_data = r_data

        self.vcp = vcp
        self.omega_sp = omega_sp
        self.Kp = Kp
        self.Ki = Ki
        self.motor_period_ms = motor_period_ms
        self.user_period_ms = user_period_ms
        self.n_samples = n_samples
        self.V_batt = V_batt

        self.help_idx = 0
        self.active_data = None
        self.active_done = None
        self.active_go = None
        self.active_status = None
        self.active_count = None
        self.active_mot = None
        self.active_omega_sp = 0.0
        self.active_Kp = 0.0
        self.active_Ki = 0.0
        self.header = False
        self.row = 0
        self.input_gen = None


    def multichar_input(self, signed = True):

        char_buf: list = []
        digits: set = set(map(str, range(10)))
        term: set = {"\r", "\n"}

        while True:

            if self.vcp.any():
                char_in = self.vcp.read(1).decode()

                if char_in in digits:
                    self.vcp.write(char_in)
                    char_buf.append(char_in)

                elif char_in == "." and char_in not in char_buf:
                    self.vcp.write(char_in)
                    char_buf.append(char_in)

                elif char_in == "-" and len(char_buf) == 0 and signed:
                    self.vcp.write(char_in)
                    char_buf.append(char_in)

                elif char_in == "\x7f" and len(char_buf) > 0:
                    self.vcp.write(char_in)
                    char_buf.pop()

                # If a termination character is received it is interpreted
                # as the end of data entry.
                elif char_in in term:
                    self.vcp.write("\r\n")
                    entered_text = "".join(char_buf)

                    if len(char_buf) == 0:
                        self.vcp.write("Value not changed\r\n")
                        return None

                    elif entered_text in {"-", ".", "-."}:
                        self.vcp.write("Invalid input\r\n")
                        return None

                    value = float(entered_text)

                    if value != value:
                        self.vcp.write("Invalid input\r\n")
                        return None

                    return value

            yield 0


    def run(self):
        state = 0

        while True:

            if  state == 0:
                for _ in range(16):         # CHARS_PER_PASS = 16
                   if self.vcp.any():
                      self.vcp.read(1)
                self.help_idx = 0  
                state = 2



            elif state == 1:

                if self.vcp.any():
                    char_in = self.vcp.read(1).decode()

                    if char_in in {"H", "h"}:
                        self.help_idx = 0
                        state = 2

                    elif char_in in {"S", "s"}:
                        print("Enter Velocity Setpoint [rad/s]:")
                        state = 6

                    elif char_in in {"P", "p"}:
                        print("Enter Proportional Gain Kp [V/(rad/s)]:")
                        state = 7

                    elif char_in in {"I", "i"}:
                        print("Enter Integral Gain Ki [V/rad]:")
                        state = 8

                    elif char_in in {"L", "l"}:
                        self.active_data = self.l_data
                        self.active_done = self.l_done
                        self.active_status = self.l_status
                        self.active_count = self.l_count
                        self.active_mot = "left"

                        self.active_omega_sp = self.omega_sp.value
                        self.active_Kp = self.Kp.value
                        self.active_Ki = self.Ki.value

                        self.active_count.value = 0
                        self.active_status.value = "starting"
                        self.l_done.value = False
                        self.header = False
                        self.row = 0
                        self.l_go.value = True
                        state = 3

                    elif char_in in {"R", "r"}:
                        print("Right Motor Start")

                        self.active_data = self.r_data
                        self.active_done = self.r_done
                        self.active_status = self.r_status
                        self.active_count = self.r_count
                        self.active_mot = "right"

                        self.active_omega_sp = self.omega_sp.value
                        self.active_Kp = self.Kp.value
                        self.active_Ki = self.Ki.value

                        self.active_count.value = 0
                        self.active_status.value = "starting"
                        self.r_done.value = False
                        self.header = False
                        self.row = 0
                        self.r_go.value = True
                        state = 4

                    elif char_in in {"X", "x"}:
                        self.vcp.write("Abort requested\r\n")
                        self.l_go.value = False
                        self.r_go.value = False
                        sys.exit(0)

                    elif char_in in {"E", "e"}:
                        self.l_go.value = False
                        self.r_go.value = False
                        raise KeyboardInterrupt


            elif state == 2:
                if self.help_idx < len(help_menu):
                    print(help_menu[self.help_idx])
                    self.help_idx +=1

                else:
                    print(f"Current settings: setpoint = {self.omega_sp.value} rad/s, Kp = {self.Kp.value}, Ki = {self.Ki.value}")
                    state = 1


            elif state == 3:                           # Left Motor State
                for _ in range (16):
                    if self.vcp.any():
                        self.vcp.read(1).decode()

                    elif char_in in {"E", "e"}:
                        self.l_go.value = False
                        self.r_go.value = False
                        raise KeyboardInterrupt

                if self.l_done.value:
                    self.l_done.value = False
                    state = 5
                

            elif state == 4:                           # Right Motor State
                for _ in range(16):
                    if self.vcp.any():
                        self.vcp.read(1)
                           
                    elif char_in in {"E", "e"}:
                        self.l_go.value = False
                        self.r_go.value = False
                        raise KeyboardInterrupt

                if self.r_done.value:
                    self.r_done.value = False
                    state = 5


            elif state == 5:
                rows = self.active_count.value

                if not self.header:
                    high_sat = False
                    low_sat = False

                    for row in range(rows):
                        voltage = self.active_data[row*4 + 3]

                    print("STARTING CLOSED-LOOP RESPONSE")
                    print(f"Motor: {self.active_mot}")
                    print(f"Setpoint [rad/s]: {self.active_omega_sp}")
                    print(f"Kp [V/(rad/s)]: {self.active_Kp}")
                    print(f"Ki [V/rad]: {self.active_Ki}")
                    print(f"Motor task period [ms]: {self.motor_period_ms}")
                    print(f"User task period [ms]: {self.user_period_ms}")
                    print(f"Samples requested: {self.n_samples}")
                    print(f"Battery voltage reference [V]: {self.V_batt}")
                    print(f"Voltage limits [V]: {-self.V_batt}, {self.V_batt}")

                    print("Time [s], Setpoint [rad/s], Measured Velocity [rad/s], Voltage Command [V]")
                    self.header = True


                for _ in range(ROWS_PER_PASS):
                    if self.row < rows:
                        i = self.row*4
                        d = self.active_data
                        print("{},           {},          {},          {}".format(d[i], d[i + 1], d[i + 2], d[i + 3]))
                        self.row +=1

                    if self.row >= rows:
                        if self.active_status.value == "completed":
                            print("CLOSED-LOOP RESPONSE COMPLETE")

                        elif self.active_status.value == "aborted":
                            print("CLOSED-LOOP RESPONSE ABORTED")

                        else:
                            print("CLOSED-LOOP RESPONSE FAILED")

                        state = 1
                        break


            elif state == 6:
                if self.input_gen is None:
                    self.input_gen = self.multichar_input(signed = True)

                try:
                    next(self.input_gen)

                except StopIteration as result:
                    value = result.value
                    self.input_gen = None

                    if value is not None:
                        self.omega_sp.value = value
                        print(f"Velocity setpoint set to {value} rad/s")
                        voltage = self.Kp.value*self.omega_sp.value + self.Ki.value*self.omega_sp.value*self.motor_period_ms/1_000

                        if voltage > self.V_batt:
                              print("Motor voltage too high, saturating to max value.")
                        elif voltage < -self.V_batt:
                            print("Motor voltage too low, saturating to min value.")

                    state = 1


            elif state == 7:
                if self.input_gen is None:
                    self.input_gen = self.multichar_input(signed = False)

                try:
                    next(self.input_gen)

                except StopIteration as result:
                    value = result.value
                    self.input_gen = None

                    if value is not None:
                        self.Kp.value = value
                        print(f"Proportional Gain Kp set to {value} V/(rad/s)")

                    state = 1


            elif state == 8:
                if self.input_gen is None:
                    self.input_gen = self.multichar_input(signed = False)

                try:
                     next(self.input_gen)

                except StopIteration as result:

                    value = result.value
                    self.input_gen = None

                    if value is not None:
                        self.Ki.value = value
                        print(f"Integral Gain Ki set to {value} V/rad")

                    state = 1


            yield state
