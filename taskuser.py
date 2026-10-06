""" MECHA 15 TRIN"""
import sys
from pyb import USB_VCP, UART


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
 
ROWS_PER_PASS = 4



S0_INIT = 0
S1_WAIT_FOR_CHAR = 1
S2_PRINT_HELP = 2
S3_LEFT_MOT = 3
S4_RIGHT_MOT = 4
S5_PRINT_RESULTS = 5
S6_DUTY_CYCLE = 6





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
        
        

    #def multichar_input(self, ser): implament week 7

        #yield from ....



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
                print(">:")
                
                if self.vcp.any():
                    char_in = self.vcp.read(1).decode()

                    if char_in in {"H", "h"}:
                        self.help_idx = 0
                        state = 2
                    
                    elif char_in in {"L", "l"}:
                        self.active = self.l_data
                        self.active_done = self.l_done
                        self.active_mot = "left"
                        self.active_effort = self.effort.value
                        self.header = False
                        self.row = 0
                        self.l_go.value = True
                        state = 3
                    
                    elif char_in in {"R", "r"}:
                        self.active = self.r_data
                        self.active_done = self.r_done
                        self.active_mot = "right"
                        self.active_effort = self.effort.value
                        self.header = False
                        self.row = 0
                        self.r_go.value = True
                        state = 4

                    elif char_in in {"D", "d"}:
                        print("Enter duty cycle [%], then press Enter:")
                        value: float = self.effort.value
                        char_buf: list []
                        digits: set = set(map(str, range(10)))
                        term: set = {\r", "\n"}
                        done = False
              

                        state = 6

                    elif char_in in {"E", "e"}:
                        raise KeyboardInterrupt


            elif state == 2:
                if self.help_idx < len(help_menu):
                    print(help_menu[self.help_idx])
                    self.help_idx +=1

                else:
                    state = 1

            elif state == 3:
                for _ in range (16):
                    if self.vcp.any():
                        self.vcp.read(1)

                if self.l_done.value:
                    self.l_done.value = False
                    state = 5
                

            elif state == 4:
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
                rows = len(self.active) // 4

                if not self.header:
                    print("STARTING OPEN-LOOP RESPONSE")
                    print(F"MOTOR: {self.active_mot}    Duty Cycle [$]: {se;f.active_effort}")
                    print("Duty cycle [%], time [us], Position [ticks], Velocity [ticks/s]")
                    self.header = True

                for _ in range(ROWS_PER_PASS):
                    if self.row < rows:
                        i = self.row*4
                        d = self.active
                        print("{},{},{},{}".format(d[i], d[i + 1], d[i + 2], d[i + 3]))
                        self.row +=1
                        
                if self.row >= rows:
                    print("OPEN-LOOP RESPONSE COMPLETE")
                    state = 1
                    break

            elif state == 6:

    
                    
                if self.vcp.any():
                    char_in = self.vcp.read(1).decode()

                    if char_in in self.digits:
                        self.vcp.write(char_in)
                        self.char_buf.append(char_in)
    
                    elif char_in == "-" and len(self.char_buf) == 0:
                        self.vcp.write(char_in)
                        self.char_buf.append(char_in)
    
                    elif char_in == "\x7f" and len(self.char_buf) >0:
                        self.vcp.write(char_in)
                        self.char_buf.pop()
    
                    elif char_in in self.term:
    
                        if len(self.char_buf) == 0:
                            print ("No duty cycle entered. Returning to main menu")
                            self.char_buf = []
                            state = 1
    
                        elif self.char_buf != ["-"]:
                            print()
                            self.duty_cycle = float("".join(self.char_buf))
                            print("Duty cycle set to {}%".format(self.duty_cycle))
                            self.char_buf = []
                            self.help.idx = 0
                            state = 2
    
                    elif (char_in == "." and len(self.char_buf) > 0 and self.char_buf != ["-"] and "." not in self.char_buf):
                        self.vcp.write(char_in)
                        self.char_buf.append(char_in)
               

                else:
                    pass


            yield state



                



