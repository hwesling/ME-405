""" MECHA 15 TRIN"""
import sys

help_menu =  (
    "+------------------------------------------------------------------------------+",
    "| ME 4305 Romi Tuning Interface Help Menu                                      |",
    "+-----+------------------------------------------------------------------------+",
    "| h/H | Print help menu                                                        |",
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




class TaskUser:
    def __init__(self, l_go, r_go, l_done, r_done, l_data, r_data, vcp):

        self.l_go = l_go
        self.r_go = r_go
        self.l_done = l_done
        self.r_done = r_done
        self.l_data = l_data
        self.r_data = r_data
     
        self.vcp = vcp
        self.help_idx = 0
        self.active = None
        self.row = 0


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
                    
                    elif char_in in {"L", "l"}:
                        self.active = self.l_data
                        self.active_done = self.l_done
                        self.row = 0
                        self.l_go.value = True
                        state = 3
                    
                    elif char_in in {"R", "r"}:
                        self.active = self.r_data
                        self.active_done = self.r_done
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

                for _ in range(ROWS_PER_PASS):
                    if self.row < rows:
                        i = self.row*4
                        d = self.active
                        print("{},{},{},{}".format(d[i], d[i + 1], d[i + 2], d[i + 3]))
                        self.row +=1


                if self.row >= rows:
                    state = 1


            yield state



                



