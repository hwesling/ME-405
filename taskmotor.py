""" MECHA 15 - Trinity """
""" Motor function task for the Romi robot, implemented as a generator and FSM so the class can be instantiated for both the left and right motors """

from time import ticks_diff, ticks_us
from math import pi


S0_INIT = 0
S1_COLLECT = 1
S2_PRINT = 2

N_SAMPLES = 101

class TaskMotor:
    def __init__(self, enc, mot, controller, task_label, go, done, data, omega_sp, kp, ki, motor_period_ms, user_period_ms, V_bat):

        self.enc = enc
        self.mot = mot
        self.controller = controller
        self.task_label = task_label
        self.go = go
        self.done = done
        self.data = data

        self.omega_sp = omega_sp
        self.kp = kp
        self.ki = ki

        self.motor_period_ms = motor_period_ms
        self.user_period_ms = user_period_ms
        self.V_bat = V_bat

        self.state = 0

        self.idx = 0
        self.collect = 0

        self.trial_start = 0
        self.prev_control_time = 0
        self.test_omega_sp = 0
        self.test_kp = 0
        self.test_ki = 0


    def run(self):

        while True: 
    
            if self.state == 0:
                if self.go.value:
                    self.collect = 0
                    self.done.value = False

                    self.test_omega_sp = self.omega_sp.value
                    self.test_kp = self.kp.value
                    self.test_ki = self.ki.value


                    self.controller.reset()

        
                    self.enc.zero()
                    self.trial_start = ticks_us()
                    self.prev_control_time = self.trial_start
        
                    self.mot.enable()
                    self.state = 1
    
            elif self.state == 1:
                i = self.collect*4
                now = ticks_us()
                self.enc.update()

                raw_velocity = self.enc.get_velocity()
                velocity = raw_velocity*(2* pi/1440) *1_000_000 

                dt = ticks_diff(now, self.prev_control_time) / 1_000_000
                self.prev_control_time = now

                voltage = self.controller.run(self.test_omega_sp, velocity, self.test_kp, self.test_ki, dt)
                duty = 100 *voltage/self.V_bat
                self.mot.set_effort(duty)


                self.data[i] = ticks_diff(ticks_us(), self.trial_start) / 1_000_000
                self.data[i + 1] = self.test_omega_sp
                self.data[i + 2] = velocity
                self.data[i + 3] = voltage

                self.collect += 1
    
                if self.collect >= N_SAMPLES:
                    self.mot.disable()
                    self.go.value = False
                    self.done.value = True
                    self.state = 2 

            elif self.state == 2:
                if not self.go.value:
                    self.state = 0

            else:
                self.mot.disable()
                self.go.value = False
                self.done.value = True
                self.state = 0

            # Yield the value of the next state to run
            yield self.state

class ClosedLoop:
    def __init__(self, V_min, V_max):

        self.V_min = V_min
        self.V_max = V_max

        self.integral = 0

    def reset(self):

        self.integral = 0

    def run(self, omega_sp, omega_k, kp, ki, dt):

        e_k = omega_sp - omega_k

        V_req = kp * e_k + ki * self.integral

        V_sat = min(self.V_max, max(self.V_min, V_req))

        # conditional-integration anti-windup:
        # do not integrate when it would push farther into saturation
        if not ((V_req >= self.V_max and ki * e_k > 0) or
                ( V_req <= self.V_min and ki * e_k < 0)):

            self.integral += e_k *dt

        return V_sat



        

    


    


