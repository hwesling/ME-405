""" MECHA 15 - Harry """
""" Motor function task for the Romi robot, implemented as a generator and FSM so the class can be instantiated for both the left and right motors """

from time import ticks_diff

S0_INIT = 0
S1_COLLECT = 1
S2_PRINT = 2

class TaskMotor:
    def __init__(self, enc, mot, task_label, effort):

        self.enc = enc
        self.mot = mot
        self.effort = effort
        self.task_label = task_label

        self.state = 0

        self.idx = 0
        self.collect = 0
        self.printcollect = 0

        self.pos = []
        self.vel = []
        self.time = []

        self.trial_start = 0
        self.done = False


    def run(self, now):

        while True: 

            if self.done:
                self.mot.disable()
                return
    
            if self.state == 0:
                self.pos = []
                self.vel = []
                self.time = []
    
                self.collect = 0
                self.printcollect = 0
    
                self.enc.zero()
                self.trial_start = now
    
                self.mot.enable()
                self.mot.set_effort(self.effort[self.idx])

                print(self.task_label, ":")
                print("\tThe state is ", self.state)
                self.state = 1
    
            elif self.state == 1:
                self.pos.append(self.enc.get_position())
                self.vel.append(self.enc.get_velocity())
    
                self.time.append(ticks_diff(now, self.trial_start))
    
                self.collect += 1
    
                if self.collect >= 101:
                    self.mot.disable()

                    print(self.task_label, ":")
                    print("\tThe state is ", self.state)                    
                    self.state = 2 

            elif self.state == 2:
                if self.printcollect < len(self.time):
    
                    print("{},{},{},{}".format(self.effort[self.idx], self.time[self.printcollect], self.pos[self.printcollect], self.vel[self.printcollect]))
    
                    self.printcollect += 1
    
    
            else:
                self.idx += 1
    
                if self.idx >= len(self.effort):
                    self.mot.disable()
                    print("Done")
                    self.done = True
                else:
                    print(self.task_label, ":")
                    print("\tThe state is ", self.state)
                    self.state = 0

            # Yield the value of the next state to run
            yield self.state



