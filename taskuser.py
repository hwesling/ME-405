""" MECHA 15 - Trinity """

import sys
from pyb import USB_VCP

help_menu =  (
    "+------------------------------------------------------------------------------+",
    "| ME 4305 Romi Tuning Interface Help Menu                                      |",
    "+-----+------------------------------------------------------------------------+",
    "| h/H | Print help menu                                                        |",
    "| o/O | Enter the desired angular velocity [rad/s]                             |",
    "| p/P | Enter the proportional gain Kp [V/(rad/s)]                             |",
    "| i/I | Enter the integral gain Ki [V/rad]                                     |",
    "| l/L | Run closed-loop repsonse on left motor and print results               |",
    "| r/R | Run closed-loop response on right motor and print results              |",
    "| e/E | Exit program                                                           |",
    "+-----+------------------------------------------------------------------------+",
)


# Used for printing encoder data
ROWS_PER_PASS = 4

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



class TaskUser:
    def __init__(self, l_go, r_go, l_done, r_done, l_data, r_data, vcp, omega_sp, kp, ki):

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
     
        self.vcp = vcp
        self.help_idx = 0
        self.active_data = None
        self.active_done = None
        self.active_mot = None
        self.header = False
        self.row = 0


    def get_float_input(self, limit = 100):
        
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
        
                elif char_in == "-" and len(char_buf) == 0:
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
                    
                    if len(char_buf) == 0:                                                     # Might need "or abs(float("".join(char_buf))) > 100" in this for kicking out bad vals
                        self.vcp.write("Value not changed\r\n")
                        return None
    
                    elif "".join(char_buf) in {"-", ".", "-."}:
                          self.vcp.write("Invalid input\r\n")
                          return None
                    
                    value = float(entered_text)                            
        
                    if abs(value) > limit:                                  # Might need to remove this line (see comment above)
                        self.vcp.write("Value outside of allowable range\r\n")
                        return None
                    return value
                        
                    
            yield 0




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

                    elif char_in in {"O", 'o'}:
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

                    elif char_in in {"E", "e"}:
                        raise KeyboardInterrupt


            elif state == 2:
                if self.help_idx < len(help_menu):
                    print(help_menu[self.help_idx])
                    self.help_idx +=1

                else:
                    state = 1

            elif state == 3:                           # Left Motor State
              
                if self.l_done.value:
                    self.l_done.value = False
                    state = 5
                

            elif state == 4:                           # Right Motor State
              
                if self.r_done.value:
                    self.r_done.value = False
                    state = 5

            elif state == 5:

                rows = len(self.active_data) // 4

                if not self.header:
                    print("STARTING CLOSED-LOOP RESPONSE")
                    print(f"Motor: {self.active_mot}")
                    print(f"Setpoint [rad/s]: {self.active_omega_sp}")
                    print(f"Kp [V/(rad/s)]: {self.active_kp}")
                    print(f"Ki [V/rad]: {self.active_ki}")
                    print(f"Time [s], Setpoint [rad/s], Measured Velocity [rad/s], Voltage Command [V]") 
                    self.header = True


                for _ in range(ROWS_PER_PASS):
                    if self.row < rows:
                        i = self.row*4
                        d = self.active_data
                        print("{},{},{},{}".format(d[i], d[i + 1], d[i + 2], d[i + 3]))
                        self.row +=1

                    if self.row >= rows:
                        print("CLOSED-LOOP RESPONSE COMPLETE")
                        state = 1
                        break

            elif state == 6:        
                if self.input_gen is None:
                    self.input_gen = self.get_float_input(limit = 50)
                try:
                    next(self.input_gen)

                except StopIteration as result:
                    value = result.value
                    self.input_gen = None

                    if value is not None:
                        self.omega_sp.value = value
                        print(f"Omega set to {value} rad/s")

                    state = 1
                

            elif state == 7:
                if self.input_gen is None:
                    self.input_gen = self.get_float_input(limit = 100)

                try:

                    next(self.input_gen)

                except StopIteration as result:

                    value = result.value
                    self.input_gen = None

                    if value is not None:
                        self.kp.value = value
                        print(f"Proportional Gain Kp set to {value} V(rad/s)")

                    state = 1

            elif state == 8:
                if self.input_gen is None:
                    self.input_gen = self.get_float_input(limit = 100)

                try:
                     next(self.input_gen)

                except StopIteration as result:

                    value = result.value
                    self.input_gen = None

                    if value is not None:
                        self.ki.value = value
                        print(f"Integral Gain Ki set to {value} V/rad")

                    state = 1


            yield state
