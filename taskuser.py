""" MECHA 15 - Trinity """

import sys
from pyb import USB_VCP
from line_calibration import save_calibration, load_calibration

help_menu =  (
    "+------------------------------------------------------------------------------+",
    "| ME 4305 Romi Tuning Interface Help Menu                                      |",
    "| Special Credit to Maddox Wintzer for Introducing 'Bingus Testing'            |",
    "+-----+------------------------------------------------------------------------+",
    "| h/H | Print help menu                                                        |",
    "| v/V | Enter the desired angular velocity [rad/s]                             |",
    "| p/P | Enter the proportional gain Kp [V/(rad/s)]                             |",
    "| i/I | Enter the integral gain Ki [V/rad]                                     |",
    "| l/L | Run closed-loop repsonse on left motor and print results               |",
    "| r/R | Run closed-loop response on right motor and print results              |", 
    "| g/G | Run both motors                                                        |",
    "| f/F | Run line following program                                             |",
    "| c/C | Line following gain (k line) [(rad/s)/mm]                              |",
    "| b/B | Calibrate line sensors(black then white)                               |",
    "| m/M | Line change manuver                                                    |",
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
S10_LINE_FOLLOW = 10
S11_LINE_KP = 11
S12_CAL_BLACK = 12
S13_CAL_WHITE = 13
S14_MAN = 14




class TaskUser:
    def __init__(self, l_go, r_go, l_done, r_done, l_data, r_data, vcp, left_omega_sp, right_omega_sp, base_speed, line_kp, line_go, kp, ki, motor_period_ms, user_period_ms, V_bat, sensor_adc, calibration, channels, calibration_file, min_span, man_go, man_done):

        self.l_go = l_go
        self.r_go = r_go
        self.l_done = l_done
        self.r_done = r_done
        self.l_data = l_data
        self.r_data = r_data
        self.kp = kp
        self.ki = ki
        self.input_gen = None

        self.left_omega_sp = left_omega_sp
        self.right_omega_sp = right_omega_sp
        self.line_kp = line_kp
        self.line_go = line_go
        self.base_speed = base_speed

        self.sensor_adc = sensor_adc

        self.calibration = calibration
        self.channels = channels
        self.calibration_file = calibration_file
        self.min_span = min_span

        self.man_go = man_go
        self.man_done = man_done

        try:
            self.calibration.value = load_calibration(
                self.calibration_file,
                self.channels,
                self.min_span
            )
            print("Saved calibration loaded.")
        
        except (OSError, ValueError, KeyError, TypeError):
            self.calibration.value = None
            print("No valid saved calibration. Press B to calibrate.")

        self.state = 0


        self.black_averages = None



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

    def read_sensor_values(self):
        """Return one raw ADC reading from each active line sensor."""
        return [adc.read() for adc in self.sensor_adc]


    def capture_averages(self, samples=20):
        """Return averaged readings from all 12 active sensors."""
        totals = [0] * len(self.sensor_adc)
    
        for _ in range(samples):
            readings = self.read_sensor_values()
    
            for index, reading in enumerate(readings):
                totals[index] += reading
    
        return [total / samples for total in totals]
    
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
                    
                        self.active_omega_sp = self.left_omega_sp.value
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

                        self.active_omega_sp = self.right_omega_sp.value
                        self.active_kp = self.kp.value
                        self.active_ki = self.ki.value

                        self.header = False
                        self.row = 0
                        self.r_go.value = True
                        state = 4

                    elif char_in in {"G","g"}:
                        print("Both motors start")
                     
                        self.r_go.value = True
                        self.l_go.value = True

                        state = 9

                    elif char_in in {"F", "f"}:
                        print("line following enabled")
                        self.line_go.value = True

                        state = 10

                    elif char_in in {"M", "m"}:
                        print("line change manueaver")
                        self.man_done.value = False
                        self.man_go.value = True
                        self.line_go.value = True

                        state = 14

                    elif char_in in {"C", "c"}:
                        print("enter line following gain [(rad/s)/mm]:")
                        state = 11

                    elif char_in in {"B", "b"}:
                        self.line_go.value = False
                        self.l_go.value = False
                        self.r_go.value = False
                    
                        self.black_averages = None
                    
                        print("Place all 12 active sensors over BLACK, then press Enter.")
                        state = 12

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



                    
                    print("# STARTING CLOSED-LOOP RESPONSE")
                    print(f"# Control Structure: {control_structure}")
                    print(f"# Motor: {self.active_mot}")
                    print(f"# Setpoint [rad/s]: {self.active_omega_sp}")
                    print(f"# Kp [V/(rad/s)]: {self.active_kp}")
                    print(f"# Ki [V/rad]: {self.active_ki}")
                    print(f"# Motor task period [ms]: {self.motor_period_ms} ")
                    print(f"# User task period [ms]: {self.user_period_ms}")
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
                    self.input_gen = self.get_float_input(limit=OMEGA_LIMIT)
            
                try:
                    next(self.input_gen)
            
                except StopIteration as result:
                    value = result.value
                    self.input_gen = None
            
                    if value == "EXIT":
                        self.exit_program()
            
                    elif value is not None:
                        # Nominal line-following speed and equal speeds
                        # for manual left/right motor tests.
                        self.base_speed.value = value
                        self.left_omega_sp.value = value
                        self.right_omega_sp.value = value
            
                        print("Omega set to {} rad/s".format(value))
            
                        voltage = self.kp.value * value
            
                        if voltage > self.V_bat:
                            print("Motor voltage may saturate high")
                        elif voltage < -self.V_bat:
                            print("Motor voltage may saturate low")
            
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
                   print("# BOTH_MOTOR_TEST_COMPLETE")
                   state = 1

            elif state == 10:
                if self.vcp.any():
                    char_in = self.vcp.read(1).decode()
                    if char_in in {"E", "e"}:
                        self.exit_program(active_test=True)
                    elif char_in in {"M", "m"}:
                        print("Line following active; lane change requested")
                        self.man_done.value = False
                        self.man_go.value = True
                        state = 14
                    if not self.line_go.value:
                        state = 1
            
            elif state == 11:
                if self.input_gen is None:
                    self.input_gen = self.get_float_input(limit=1.0, signed=False)
            
                try:
                    next(self.input_gen)
            
                except StopIteration as result:
                    value = result.value
                    self.input_gen = None
            
                    if value == "EXIT":
                        self.exit_program()
            
                    elif value is not None:
                        self.line_kp.value = value
                        print("Line gain set to {}".format(value))
            
                    state = 1

            elif state == 12:
               if self.vcp.any():
                   char_in = self.vcp.read(1)
                   if char_in in (b"\r", b"\n"):
                       # Remove any remaining bytes from this Enter press,
                       # including a paired CR/LF character.
                       while self.vcp.any():
                           self.vcp.read(self.vcp.any())
                       self.black_averages = self.capture_averages()
                       print("Black:", self.black_averages)
                       print(
                           "Place all 12 active sensors over WHITE, "
                           "then press Enter."
                       )
                       state = 13

            elif state == 13:
                if self.vcp.any():
                    char_in = self.vcp.read(1)

                    if char_in in (b"\r", b"\n"):
                        while self.vcp.any():
                            self.vcp.read(self.vcp.any())

                        white_averages = self.capture_averages()
                        print("White:", white_averages)

                        calibration = {
                            "channels": self.channels,
                            "black": self.black_averages,
                            "white": white_averages,
                        }

                        try:
                            save_calibration(
                                self.calibration_file,
                                calibration,
                                self.channels,
                                self.min_span
                            )

                            self.calibration.value = calibration
                            print("Calibration saved.")

                        except (ValueError, OSError, KeyError, TypeError) as error:
                            print("Calibration rejected:", error)

                        state = 1

            elif state == 14:
                if self.man_done.value:
                    self.man_go.value = False
                    self.line_go.value = False
                    print("line change ended")

                    
                    state = 1
            
                                            

            yield state
