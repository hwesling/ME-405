""" MECHA 15 - Harry """
""" A user interface task for the Romi robot, implemented as a FSM """

import sys
from pyb       import Pin, Timer, USB_VCP, UART # type: ignore

help_menu =  (
    "+------------------------------------------------------------------------------+",
    "| ME 4305 Romi Tuning Interface Help Menu                                      |",
    "+-----+------------------------------------------------------------------------+",
    "| h/H | Print help menu                                                        |",
    "| d/D | Enter the duty cycle for the next open-loop test                       |",
    "| l/L | Trigger step response sequence on left motor and print results         |",
    "| r/R | Trigger step response sequence on right motor and print results        |",
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
S6_DUTY_CYCLE_INPUT = 6


class TaskUser:
    def __init__(self, l_go, r_go, l_done, r_done, l_data, r_data, vcp, effort):

        self.l_go = l_go
        self.r_go = r_go
        self.l_done = l_done
        self.r_done = r_done
        self.l_data = l_data
        self.r_data = r_data
        self.effort = effort
     
        self.vcp = vcp
        self.help_idx = 0
        self.active_data = None
        self.active_done = None
        self.active_mot = None
        self.active_effort = 0
        self.header = False
        self.row = 0
        self.ser = USB_VCP()


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
                print(">: ")

                if self.vcp.any():
                    char_in = self.vcp.read(1).decode()

                    if char_in in {"H", "h"}:
                        self.help_idx = 0
                        state = 2

                    elif char_in in {"D", 'd'}:
                        print("Enter Duty Cycle [%]:")
                        value: float = self.effort.value
                        char_buf: list = []
                        digits: set = set(map(str, range(10)))
                        term: set = {"\r", "\n"}
                        done = False
                        state = 6
                    
                    elif char_in in {"L", "l"}:
                        print("Left Motor Start")
                        self.active_data = self.l_data
                        self.active_done = self.l_done
                        self.active_mot = "left"
                        self.active_effort = self.effort.value
                        self.header = False
                        self.row = 0
                        self.l_go.value = True
                        state = 3
                    
                    elif char_in in {"R", "r"}:
                        print("Right Motor Start")
                        self.active_data = self.r_data
                        self.active_done = self.r_done
                        self.active_mot = "right"
                        self.active_effort = self.effort.value
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
                for _ in range (16):
                    if self.vcp.any():
                        self.vcp.read(1)

                if self.l_done.value:
                    self.l_done.value = False
                    state = 5
                

            elif state == 4:                           # Right Motor State
                for _ in range(16):
                    if self.vcp.any():
                        self.vcp.read(1)

                if self.r_done.value:
                    self.r_done.value = False
                    state = 5

            elif state == 5:
                for _ in range(16):
                    if self.vcp.any():
                        self.vcp.read(1)
                rows = len(self.active_data) // 4

                if not self.header:
                    print("STARTING OPEN-LOOP RESPONSE")
                    print(f"Motor: {self.active_mot}    Duty Cycle [%]: {self.active_effort}")
                    print("Duty Cycle [%], time [us], Position [ticks], Velocity [ticks/s]")
                    self.header = True

                for _ in range(ROWS_PER_PASS):
                    if self.row < rows:
                        i = self.row*4
                        d = self.active_data
                        print("{},{},{},{}".format(d[i], d[i + 1], d[i + 2], d[i + 3]))
                        self.row +=1

                    if self.row >= rows:
                        print("OPEN-LOOP RESPONSE COMPLETE")
                        state = 1
                        break

            elif state == 6:                           # Duty Cycle Input State --> only listed as S6 to maintain existing FSM logic,
                                                       # state 5 (print) kicks back to main/taskmotor, S6 should be a generator for reusability later (as of 10/1/26)
                if not done:

                    if self.ser.any():
                        char_in = self.ser.read(1).decode()
                        
                        if char_in in digits:
                            self.ser.write(char_in)
                            char_buf.append(char_in)
                
                        elif char_in == "." and char_in not in char_buf:
                            self.ser.write(char_in)
                            char_buf.append(char_in)
                
                        elif char_in == "-" and len(char_buf) == 0:
                            self.ser.write(char_in)
                            char_buf.append(char_in)
                        
                        elif char_in == "\x7f" and len(char_buf) > 0:
                            self.ser.write(char_in)
                            char_buf.pop()

                        # If a termination character is received it is interpreted
                        # as the end of data entry.
                        elif char_in in term:
                            
                            if len(char_buf) == 0:                                                     # Might need "or abs(float("".join(char_buf))) > 100" in this for kicking out bad vals
                                self.ser.write("\r\n")                                                 
                                self.ser.write("Value not changed\r\n")
                                char_buf = []
                                done = True
                                
                            elif "".join(char_buf) in {"-", ".", "-."}:
                                  self.ser.write("\r\n")
                                  self.ser.write("Invalid input\r\n")
                                  char_buf = []
                                  done = True                            

                            elif abs(float("".join(char_buf))) > 100:                                  # Might need to remove this line (see comment above)
                                self.ser.write("\r\n")                                                 # Will need to be adjusted when controller gain is implemented
                                self.ser.write("Value outside of allowable range\r\n")
                                char_buf = []
                                done = True    
                                
                            else:
                                self.ser.write("\r\n")
                                value = float("".join(char_buf))
                                self.effort.value = value
                                self.ser.write(f"Value set to {value}\r\n")
                                char_buf = []
                                done = True
                else:
                    state = 1                

            yield state
