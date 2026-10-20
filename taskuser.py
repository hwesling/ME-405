""" MECHA 15 - Trinity """

import sys
from pyb import USB_VCP

help_menu =  (
    "+------------------------------------------------------------------------------+",
    "| ME 4305 Romi Tuning Interface Help Menu                                      |",
    "+-----+------------------------------------------------------------------------+",
    "| h/H | Print help menu                                                        |",
    "| v/V | Enter the desired angular velocity [rad/s]                             |",
    "| p/P | Enter the proportional gain Kp [V/(rad/s)]                             |",
    "| i/I | Enter the integral gain Ki [V/rad]                                     |",
    "| l/L | Run closed-loop repsonse on left motor and print results               |",
    "| r/R | Run closed-loop response on right motor and print results              |", 
    "| a/A | Print one reading from each line sensor                                |",
    "| g/G | Run both motors                                                        |",
    "| e/E | Exit program                                                           |",
    "+-----+------------------------------------------------------------------------+",
)


# Used for printing encoder data
ROWS_PER_PASS = 4

OMEGA_LIMIT = 50
KP_LIMIT = 100
KI_LIMIT = 100

# List of states
S0_INIT = 0
S1_WAIT_FOR_CHAR = 1
S2_PRINT_HELP = 2
S3_LEFT_MOT = 3
S4_RIGHT_MOT = 4
S5_PRINT_RESULTS = 5
S6_OMEGA_INPUT = 6
S7_KP = 7
S8_KI = 8
S9_BOTH_MOT = 9



class TaskUser:
    def __init__(self, l_go, r_go, l_done, r_done, l_data, r_data, vcp, omega_sp, kp, ki, motor_period_ms, user_period_ms, V_bat):

        self.l_go = l_go
        self.r_go = r_go
        self.l_done = l_done
        self.r_done = r_done
        self.l_data = l_data
        self.r_data = r_data
        self.omega_sp = omega_sp
        self.kp = kp
        self.ki = ki
        self.input_gen = None

        self.motor_period_ms = motor_period_ms
        self.user_period_ms = user_period_ms
        self.V_bat = V_bat
     
        self.vcp = vcp
        self.help_idx = 0
        self.active_data = None
        self.active_done = None
        self.active_mot = None
        self.header = False
        self.row = 0


    def get_float_input(self, limit = 100, signed = True):
        
        char_buf: list = []
        digits: set = set(map(str, range(10)))
        term: set = {"\r", "\n"}

        while True:
            
            if self.vcp.any():
                char_in = self.vcp.read(1).decode()

                if char_in in {"E", "e"}:
                    self.vcp.write("\r\n")
                    return "EXIT"
                
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
    
                    elif "".join(char_buf) in {"-", ".", "-."}:
                          self.vcp.write("Invalid input\r\n")
                          return None
                    
                    value = float(entered_text)                            
        
                    if abs(value) > limit:                                 
                        self.vcp.write("Value outside of allowable range\r\n")
                        return None
                    return value
                        
                    
            yield 0


    def exit_program(self, active_test=False):
        if active_test:
            print("# TEST_ABORTED")

        print("# EXIT")
        raise KeyboardInterrupt

    def run(self):
        state = 0

        while True:

            if  state == 0:
                for _ in range(16):
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

                    elif char_in in {"V", 'v'}:
                        print("Enter Velocity as Omega [rad/s]:")

                        state = 6

                    elif char_in in {"P", "p"}:
                        print("Enter Proportional Gain Kp [V/(rad/s)]:")

                        state = 7

                    elif char_in in {"I", "i"}:
                        print("Enter Integral Gain Ki [V/rad]:")

                        state = 8

                    elif char_in in {"L", "l"}:
                        print("Left Motor Start")

                        self.active_data = self.l_data
                        self.active_done = self.l_done
                        self.active_mot = "left"
                    
                        self.active_omega_sp = self.omega_sp.value
                        self.active_kp = self.kp.value
                        self.active_ki = self.ki.value
                    
                        self.header = False
                        self.row = 0
                        self.l_go.value = True
                        state = 3
                    
                    elif char_in in {"R", "r"}:
                        print("Right Motor Start")

                        self.active_data = self.r_data
                        self.active_done = self.r_done
                        self.active_mot = "right"

                        self.active_omega_sp = self.omega_sp.value
                        self.active_kp = self.kp.value
                        self.active_ki = self.ki.value

                        self.header = False
                        self.row = 0
                        self.r_go.value = True
                        state = 4

                    elif char_in in {"A", "a"}:
                        readings = self.line_sensor.read()

                        print("line-sensor readings:")
                        for index, value in enumerate(readings):
                            print(f"OUT{index + 1}: {value}")

                    elif char_in in {"G","g"}:
                        print("Both motors start")
                        
                        self.active_data = self.r_data
                        self.active_done = self.r_done
                        self.active_mot = "right"

                        self.active_omega_sp = self.omega_sp.value
                        self.active_kp = self.kp.value
                        self.active_ki = self.ki.value

                        self.active_data = self.l_data
                        self.active_done = self.l_done
                        self.active_mot = "left"

                        self.header = False
                        self.row = 0
                        self.r_go.value = True
                        self.l_go.value = True

                        state = 9

                    elif char_in in {"E", "e"}:
                        self.exit_program()


            elif state == 2:
                if self.help_idx < len(help_menu):
                    print(help_menu[self.help_idx])
                    self.help_idx +=1

                else:
                    state = 1

            elif state == 3:                           # Left Motor State

                if self.vcp.any():
                    char_in = self.vcp.read(1).decode()

                    if char_in in{"E", "e"}:
                        self.exit_program(active_test=True)
              
                if self.l_done.value:
                    self.l_done.value = False
                    state = 5
                

            elif state == 4:                           # Right Motor State

                if self.vcp.any():
                    char_in = self.vcp.read(1).decode()

                    if char_in in {"E", "e"}:
                        self.exit_program(active_test=True)

              
                if self.r_done.value:
                    self.r_done.value = False
                    state = 5

            elif state == 5:

                rows = len(self.active_data) // 4

                if not self.header:

                    if self.active_ki == 0:
                        control_structure = "P"

                    else:
                        control_structure = "PI"


                    duration = 100

                    
                    print("# STARTING CLOSED-LOOP RESPONSE")
                    print(f"# Control Structure: {control_structure}")
                    print(f"# Motor: {self.active_mot}")
                    print(f"# Setpoint [rad/s]: {self.active_omega_sp}")
                    print(f"# Kp [V/(rad/s)]: {self.active_kp}")
                    print(f"# Ki [V/rad]: {self.active_ki}")
                    print(f"# Motor task period [ms]: {self.motor_period_ms} ")
                    print(f"# User task period [ms]: {self.user_period_ms}")
                    print(f"# Duration [s]: {duration}")
                    print(f"# Battery voltage refrence [V]: {self.V_bat}")
                    print(f"# Voltage limits [V]: {-self.V_bat}, {self.V_bat}")
                    print("# Units: time = s, setpoint = rad/s, velocity = rad/s, voltage = V")
                    print(
                        f"# Validation Limits: |setpoint| <= {OMEGA_LIMIT} rad/s, "
                        f"|Kp| <= {KP_LIMIT} V/(rad/s), "
                        f"|Ki| <= {KI_LIMIT} V/rad"
                    )

                    print("time_s,setpoint_rad_s,measured_velocity_rad_s,voltage_command_V")


                    self.header = True


                for _ in range(ROWS_PER_PASS):
                    if self.row < rows:
                        i = self.row*4
                        d = self.active_data

                        print("{},          {},          {},          {}".format(d[i], d[i + 1], d[i + 2], d[i + 3]))
                        self.row +=1

                    if self.row >= rows:
                        print("# TEST_COMPLETE")
                        state = 1
                        break


            elif state == 6:        
                if self.input_gen is None:
                    self.input_gen = self.get_float_input(limit = OMEGA_LIMIT)
                try:
                    next(self.input_gen)

                except StopIteration as result:
                    value = result.value
                    self.input_gen = None

                    if value == "EXIT":
                        self.exit_program()

                    elif value is not None:
                        self.omega_sp.value = value
                        print(f"Omega set to {value} rad/s")
                        voltage = self.kp.value*self.omega_sp.value + self.ki.value*self.omega_sp.value*self.motor_period_ms/1_000

                        if voltage > self.V_bat:
                            print("Motor voltage too high, saturating to max value")
                        elif voltage < -self.V_bat:
                            print("Motor voltage too low, saturation to min value")



                    state = 1
                

            elif state == 7:
                if self.input_gen is None:
                    self.input_gen = self.get_float_input(limit = KP_LIMIT, signed=False)

                try:

                    next(self.input_gen)

                except StopIteration as result:

                    value = result.value
                    self.input_gen = None

                    if value == "EXIT":
                        self.exit_program()
                    
                    elif value is not None:
                        self.kp.value = value
                        print(f"Proportional Gain Kp set to {value} V(rad/s)")

                    state = 1

            elif state == 8:
                if self.input_gen is None:
                    self.input_gen = self.get_float_input(limit = KI_LIMIT, signed=False)

                try:
                     next(self.input_gen)

                except StopIteration as result:

                    value = result.value
                    self.input_gen = None

                    if value == "EXIT":
                        self.exit_program()

                    elif value is not None:
                        self.ki.value = value
                        print(f"Integral Gain Ki set to {value} V/rad")

                    state = 1

            elif state == 9:

                #  motor
                if self.vcp.any():
                    char_in = self.vcp.read(1).decode()

                    if char_in in{"E", "e"}:
                        self.exit_program(active_test=True)
                              
                if self.l_done.value and self.r_done.value:
                    self.l_done.value = False
                    self.r_done.value = False
                    state = 5
                    

            yield state
