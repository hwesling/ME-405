""" MECHA 15 - ROMI """

class TaskLine:
    def __init__(self, read_values, calibration, positions, left_sp, right_sp, base_speed, line_go):
        self.read_values = read_values
        self.calibration = calibration
        self.positions = positions
        self.left_sp = left_sp
        self.right_sp = right_sp
        self.base_speed = base_speed
        self.line_go = line_go

    def run(self):
        while True:
            if not self.line_go.value:
                self.left_sp.value = 0
                self.right_sp.value = 0
                yield 0
                continue

            readings = self.read_values()
            weights = []

            for i in range(len(readings)):
                white = self.calibration["white"][i]
                black = self.calibration["black"][i]
                weight = (readings[i] - white) / (black - white)

                if weight < 0:
                    weight = 0
                elif weight > 1:
                    weight = 1

                weights.append(weight)

            total = sum(weights)

            # Lost line: stop safely
            if total < 0.5:
                self.left_sp.value = 0
                self.right_sp.value = 0
                yield 1
                continue

            centroid = sum(
                self.positions[i] * weights[i]
                for i in range(len(weights))
            ) / total

            correction = 0.5 * centroid  # starting value; tune later
            self.left_sp.value = self.base_speed.value - correction
            self.right_sp.value = self.base_speed.value + correction

            yield 2