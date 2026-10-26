""" MECHA 15 - ROMI """

class TaskLine:
    def __init__(self, read_values, calibration, positions,
                 left_sp, right_sp, base_speed, line_kp,
                 line_go, l_go, r_go):

        self.read_values = read_values
        self.calibration = calibration
        self.positions = positions

        self.left_sp = left_sp
        self.right_sp = right_sp
        self.base_speed = base_speed
        self.line_kp = line_kp

        self.line_go = line_go
        self.l_go = l_go
        self.r_go = r_go


    def stop(self):
        self.left_sp.value = 0
        self.right_sp.value = 0
        self.l_go.value = False
        self.r_go.value = False


    def run(self):
        while True:

            if not self.line_go.value:
                yield 0
                continue
            
            calibration = self.calibration.value

            if calibration is None:
                print("no calibration loaded")
                self.line_go.value = False
                self.stop()
                yield 1
                continue

            readings = self.read_values()
            weights = []
            valid_calibration = True

            for i in range(len(readings)):
                white = calibration["white"][i]
                black = calibration["black"][i]
                span = black - white

                if span <= 0:
                    valid_calibration = False
                    break

                weight = (readings[i] - white) / span

                if weight < 0:
                    weight = 0
                elif weight > 1:
                    weight = 1

                weights.append(weight)

            if not valid_calibration:
                self.line_go.value = False
                self.stop()
                yield 1
                continue

            total = sum(weights)

            # Lost line: stop safely.
            if total < 0.2:
                print("line lost: total = {}".format(total))
                self.line_go.value = False
                self.stop()
                yield 1
                continue

            centroid_mm = 0
            for i in range(len(weights)):
                centroid_mm += self.positions[i] * weights[i]
            centroid_mm /= total

            # Units: line_kp is (rad/s)/mm.
            correction = self.line_kp.value * centroid_mm

            self.left_sp.value = self.base_speed.value - correction
            self.right_sp.value = self.base_speed.value + correction

            # Let the motor tasks begin/continue continuous control.
            self.l_go.value = True
            self.r_go.value = True

            yield 2