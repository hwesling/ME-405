""" MECHA 15 - Harry """
""" Motor function task for the Romi robot, implemented as a generator and FSM so the class can be instantiated for both the left and right motors """

from time import ticks_diff, ticks_us

S0_INIT = 0
S1_COLLECT = 1
S2_PRINT = 2

N_SAMPLES = 101

class TaskMotor:
    def __init__(self, enc, mot, task_label, go, done, data, effort):

        self.enc = enc
        self.mot = mot
        self.effort = effort
        self.task_label = task_label
        self.go = go
        self.done = done
        self.data = data

        self.state = 0

        self.idx = 0
        self.collect = 0

        self.trial_start = 0
        self.test_effort = 0


    def run(self):

        while True: 
    
            if self.state == 0:
                if self.go.value:
                    self.collect = 0
                    self.done.value = False
                    self.test_effort = self.effort.value
        
                    self.enc.zero()
                    self.trial_start = ticks_us()
        
                    self.mot.enable()
                    self.mot.set_effort(self.test_effort)
                    self.state = 1
    
            elif self.state == 1:
                i = self.collect*4
                self.data[i] = self.test_effort
                self.data[i + 1] = ticks_diff(ticks_us(), self.trial_start)
                self.data[i + 2] = self.enc.get_position()
                self.data[i + 3] = self.enc.get_velocity()

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


