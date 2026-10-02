from time import ticks_diff, ticks_us
 
S0_INIT = 0
S1_COLLECT = 1
S2_IDLE = 2
 
class TaskMotor:
    def __init__(self, enc, mot, trial_set, task_motor, go, done, data):
 
        self.enc = enc
        self.mot = mot
        self.trial_set = trial_set
 
        self.state = 0
 
 
        self.idx = 0
        self.collect = 0
        self.printcollect = 0
        self.row = 0
 
        self.pos = []
        self.vel = []
        self.time = []
 
        self.trial_start = 0
 
        self.task_motor = task_motor
 
        self.go = go
        self.done = done
        self.data = data
 
 
 
    def run(self):
 
        while True:
            self.enc.update()
            now = ticks_us()
 
            if self.state == 0 and self.idx == 0 and not self.go.value:
                yield self.state
                continue
    
            if self.state == 0:
                if self.idx == 0:
                    self.go.value = False
                    self.row = 0
 
                self.pos = []
                self.vel = []
                self.time = []
    
                self.collect = 0
                self.printcollect = 0
    
                self.enc.zero()
                self.trial_start = now
    
                self.mot.enable()
                self.mot.set_effort(self.trial_set[self.idx])
    
                self.state = 1
    
            elif self.state == 1:
                self.pos.append(self.enc.get_position())
                self.vel.append(self.enc.get_velocity())
    
                self.time.append(ticks_diff(now, self.trial_start))
    
                self.collect += 1
    
                if self.collect >= 101:
                    self.mot.disable()
                    self.state = 2
            elif self.state == 2:
                if self.printcollect < len(self.time):
           
                    i = self.row * 4
                    self.data[i] = self.trial_set[self.idx]
                    self.data[i + 1] = self.time[self.printcollect]
                    self.data[i + 2] = self.pos[self.printcollect]
                    self.data[i + 3] = self.vel[self.printcollect]
                    self.row += 1
    
                    self.printcollect += 1
    
    
                else:
                    self.idx += 1
        
                    if self.idx >= len(self.trial_set):
                        self.mot.disable()
                        self.done.value = True
                        self.idx = 0
                        self.state = 0
                    else:
                        self.state = 0
 
            yield self.state
 
