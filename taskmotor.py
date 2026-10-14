""" MECHA 15 - Harry """
""" Motor function task for the Romi robot, implemented as a generator and FSM so the class can be instantiated for both the left and right motors """

from time import ticks_diff, ticks_us
from math import pi


S0_INIT = 0
S1_COLLECT = 1
S2_PRINT = 2

N_SAMPLES = 101


class TaskMotor:
    def __init__(self, enc, mot, task_label, go, done, status, sample_count, data, omega_sp, Kp, Ki, V_batt = 7.2):
        self.enc = enc
        self.mot = mot
        self.task_label = task_label
        self.go = go
        self.done = done
        self.status = status
        self.sample_count = sample_count
        self.data = data
        self.omega_sp = omega_sp
        self.Kp = Kp
        self.Ki = Ki
        self.V_batt = V_batt

        # Initialize controller object
        self.controller = ClosedLoop(Kp=self.Kp.value, Ki=self.Ki.value, V_min = -V_batt, V_max = V_batt)

        self.state = 0

        self.idx = 0
        self.collect = 0


    def run(self):

        while True:
    
            if self.state == 0:
                if self.go.value:
                    self.collect = 0
                    self.done.value = False
                    self.status.value = "running"

                    self.test_omega_sp = self.omega_sp.value
                    self.test_Kp = self.Kp.value
                    self.test_Ki = self.Ki.value
                    self.controller.Kp = self.test_Kp                       # Set P and I gain values
                    self.controller.Ki = self.test_Ki
                    self.controller.reset()                                 # Reset controller on startup
        
                    self.enc.zero()
                    self.trial_start = ticks_us()
                    self.previous_time = self.trial_start
        
                    self.mot.enable()
                    self.state = 1
    
            elif self.state == 1:
                if not self.go.value:
                    self.mot.disable()
                    self.status.value = "aborted"
                    self.done.value = True
                    self.state = 2

                else:
                    i = self.collect*4
                    self.enc.update()
                    now = ticks_us()
                    dt_s = ticks_diff(now, self.previous_time)/1_000_000
                    self.previous_time = now

                    # Obtain measured velocity and convert to motor effort
                    omega_hat_ticks = self.enc.get_velocity()                                   # [ticks/s]
                    omega_hat = omega_hat_ticks*(2*pi/1440)*1_000_000                           # [rad/s]
                    voltage = self.controller.update(self.test_omega_sp, omega_hat, dt_s)
                    duty_cycle = self.voltage_to_dc(voltage)
                    self.mot.set_effort(duty_cycle)

                    # collect data into array
                    self.data[i] = ticks_diff(now, self.trial_start) / 1_000_000
                    self.data[i + 1] = self.test_omega_sp
                    self.data[i + 2] = omega_hat
                    self.data[i + 3] = voltage
    
                    self.collect += 1
                    self.sample_count.value = self.collect

                    if self.collect >= N_SAMPLES:
                        self.mot.disable()
                        self.go.value = False
                        self.status.value = "completed"
                        self.done.value = True
                        self.state = 2 

            elif self.state == 2:
                if not self.go.value:
                    self.state = 0

            else:
                self.mot.disable()
                self.go.value = False
                self.status.value = "failed"
                self.done.value = True
                self.state = 0

            # Yield the value of the next state to run
            yield self.state


    def voltage_to_dc(self, voltage):
        if self.V_batt <= 0:
            return 0.0

        duty_cycle = 100.0*voltage/self.V_batt
        return max(-100.0, min(100.0, duty_cycle))


class ClosedLoop:
    def __init__(self, Kp, Ki, V_min = -7.2, V_max = 7.2):
        self.Kp = Kp
        self.Ki = Ki
        self.V_min = V_min
        self.V_max = V_max

        # Controller state
        self.error = 0.0
        self.i_error = 0.0
        self.p_term = 0.0
        self.i_term = 0.0
        self.V_req = 0.0
        self.V_sat = 0.0


    def update(self, omega_sp, omega_hat, dt_s):

        # Proportional Control
        self.error = omega_sp - omega_hat
        self.p_term = self.Kp*self.error
    
        # Proposed Integral Control
        new_i_error = self.i_error + self.error*dt_s
        new_i_term = self.Ki*new_i_error
    
        # Requested Voltage with proposed integral term
        self.V_req = self.p_term + new_i_term
    
        # Saturate Voltage
        self.V_sat = self.saturate(self.V_req)
    
        # Anti-windup, only accept the new integral error if it doesn't push the saturated voltage past its limits.
        if self.V_req == self.V_sat:
            self.i_error = new_i_error
    
        elif self.V_req > self.V_max and self.error < 0:
            self.i_error = new_i_error
    
        elif self.V_req < self.V_min and self.error > 0:
            self.i_error = new_i_error
    
        # Recalculate using accepted integral error
        self.i_term = self.Ki*self.i_error
        self.V_req = self.p_term + self.i_term
        self.V_sat = self.saturate(self.V_req)
    
        return self.V_sat


    def saturate(self, voltage):
        if voltage > self.V_max:
            return self.V_max
    
        elif voltage < self.V_min:
            return self.V_min
    
        else:
            return voltage

    
    def reset(self):
         self.error = 0.0
         self.i_error = 0.0
         self.p_term = 0.0
         self.i_term = 0.0
         self.V_req = 0.0
         self.V_sat = 0.0