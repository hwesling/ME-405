""" MECHA 15 - Harry """
""" Motor function task for the Romi robot, implemented as a generator and FSM so the class can be instantiated for both the left and right motors """

from time import ticks_diff, ticks_us
from math import pi
from ulab import numpy as np


S0_INIT = 0
S1_COLLECT = 1
S2_PRINT = 2

N_SAMPLES = 101
COUNTS_TO_RAD = 2*pi/1440

# Paste MATLAB arrays from observer_output_check.py
Ao = np.array([
    [3.550270598904896e-05, -1.5352171978371274, 0.0048798267467947748],
    [0.0048798267467947704, 0.7483046398689529, 0.00022018644547564163],
    [0, -99.029067144029568, 1],
])
Bo = np.array([
    [2.3499165686409258, 1.5352171978371274],
    [0.10603240714503231, 0.2516953601310471],
    [0, 99.029067144029568],
])

Co = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
])
Do = np.array([
    [0, 0],
    [0, 0],
    [0, 0],
])


class TaskMotor:
    def __init__(self, enc, mot, controller, task_label, go, done, data, omega_sp, kp, ki, motor_period_ms, user_period_ms, V_bat, line_go):

        self.enc = enc
        self.mot = mot
        self.controller = controller
        self.task_label = task_label
        self.go = go
        self.done = done
        self.data = data

        self.omega_sp = omega_sp
        self.line_go = line_go
     

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

        self.x_hat = np.array([0.0, 0.0, 0.0])
        self.w_obs = np.array([0.0, 0.0])
        self.omega_hat = 0.0


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

                    self.x_hat = np.array([0.0, 0.0, 0.0])                     # Measured position begins at zero radians since encoder was zeroed
                    self.w_obs = np.array([0.0, 0.0])
                    self.omega_hat = 0.0

                    self.trial_start = ticks_us()
                    self.prev_control_time = self.trial_start
        
                    self.mot.enable()
                    self.state = 1

    
            elif self.state == 1:

                if not self.go.value:
                    self.mot.disable()
                    self.done.value = True
                    self.state = 0
                    yield self.state
                    continue

                now = ticks_us()
                self.enc.update()

                raw_velocity = self.enc.get_velocity()
                velocity = raw_velocity * COUNTS_TO_RAD * 1_000_000
                encoder_position = self.enc.get_position() * COUNTS_TO_RAD

                dt = ticks_diff(now, self.prev_control_time) / 1_000_000
                self.prev_control_time = now

                if self.line_go.value:
                    active_omega_sp = self.omega_sp.value
                else:
                    active_omega_sp = self.test_omega_sp

                # 1. Read CURRENT observer estimate before updating it.
                self.omega_hat = self.x_hat[0]
                
                # 2. Controller uses the observer velocity feedback.
                voltage = self.controller.run(active_omega_sp, self.omega_hat, self.test_kp, self.test_ki, dt)
                
                # 3. Convert command to duty and explicitly enforce driver limits.
                duty = 100 * voltage / self.V_bat
                duty = min(100.0, max(-100.0, duty))
                self.mot.set_effort(duty)
                
                # 4. Recover the voltage that was ACTUALLY sent through the duty command.
                voltage_applied = duty * self.V_bat / 100.0
                
                # 5. Form w = [applied voltage; measured encoder position].
                self.w_obs[0] = voltage_applied
                self.w_obs[1] = encoder_position
                
                # 6. Compute ALL of x_hat_next before replacing x_hat.
                x_hat_next = np.dot(Ao, self.x_hat) + np.dot(Bo, self.w_obs)
                self.x_hat = x_hat_next

                if not self.line_go.value:
                    if self.collect < N_SAMPLES:
                        i = self.collect * 5
                
                        self.data[i] = ticks_diff(ticks_us(), self.trial_start) / 1_000_000        # time
                        self.data[i + 1] = active_omega_sp                                         # wheel velocity setpoint
                        self.data[i + 2] = velocity                                                # original finite diff. velocity
                        self.data[i + 3] = self.omega_hat                                          # observer velocity at time t
                        self.data[i + 4] = voltage_applied                                         # actual applied voltage
                
                        self.collect += 1
                
                    # End only after the fixed-size data buffer is full.
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


# Inner control loop. Controls motors, converting velocity setpoint to duty cycle and required voltage
class ClosedLoop:
    def __init__(self, V_min, V_max):

        self.V_min = V_min
        self.V_max = V_max

        self.integral_voltage = 0


    def reset(self):
        self.integral_voltage = 0


    def run(self, omega_sp, omega_hat, kp, ki_s, dt):           # Observer feedback + discrete PI Kp = 0.5, Ki_s = 0.04
                                                                # Unstable at 2 rad/s: motor jitter and serial disconnect
        e_k = omega_sp - omega_hat                              # DISCRETE KI VALUE -> Ki_s = Ki_c*Ts = (.8)(.05) = .04

        V_req = kp * e_k + self.integral_voltage

        V_sat = min(self.V_max, max(self.V_min, V_req))

        residual = V_sat - V_req

        # Integrate when unsaturated, or when the error drives the integral state back toward the allowed voltage range.
        if residual == 0 or residual * e_k > 0:
            self.integral_voltage += ki_s * e_k

        return V_sat
