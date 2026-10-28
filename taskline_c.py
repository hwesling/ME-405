""" MECHA 15 - ROMI """
"""Line following and IMU lane-change task controlled by man_go/man_done."""

from math import pi, sin
from time import ticks_diff, ticks_us


S0_WAIT = 0
S1_LINE_FOLLOW = 1
S2_MAN_FOLLOW_START = 2
S3_DEPART = 3
S4_TURN_BACK = 4
S5_FOLLOW_TARGET = 5
S6_DONE = 6


class TaskLineC:
    def __init__(self, read_values, calibration, positions,
                 left_sp, right_sp, base_speed, line_kp,
                 line_go, l_go, r_go, imu, left_enc, right_enc,
                 man_go, man_done):

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
        self.imu = imu
        self.left_enc = left_enc
        self.right_enc = right_enc
        self.man_go = man_go
        self.man_done = man_done

        # Lane-change settings. +1 is left; use -1 for a right lane change.
        self.d = 0.20
        self.turn_sign = 1
        self.depart_after_s = 1.5
        self.yaw_rate_sp = 1
        self.yaw_kp = .3
        self.timeout_s = 4
        #####################################################################

        self.wheel_radius = 0.035
        self.counts_per_rev = 1440
        self.max_wheel_speed = 15.0
        self.lost_line_sum = 0.20
        self.target_line_sum = 0.80
        self.white_sum_max = 0.25
        self.black_weight_min = 0.08
        self.white_samples_required = 3

        self.state = S0_WAIT
        self.start_follow_time = 0
        self.maneuver_start_time = 0
        self.previous_update_time = 0
        self.heading_start = 0.0
        self.lateral_displacement = 0.0
        self.white_samples = 0
        self.white_cleared = False


    def stop(self):
        self.left_sp.value = 0
        self.right_sp.value = 0
        self.line_go.value = False
        self.l_go.value = False
        self.r_go.value = False


    def finish_maneuver(self, message):
        print(message)
        self.stop()
        self.man_go.value = False
        self.man_done.value = True
        self.state = S6_DONE


    def limit_wheel_speed(self, speed):
        if speed > self.max_wheel_speed:
            return self.max_wheel_speed
        elif speed < -self.max_wheel_speed:
            return -self.max_wheel_speed
        return speed


    def get_weights(self):
        calibration = self.calibration.value
        if calibration is None:
            return None

        readings = self.read_values()
        if len(readings) != len(self.positions):
            return None

        weights = []
        for i in range(len(readings)):
            white = calibration["white"][i]
            black = calibration["black"][i]
            span = black - white
            if span <= 0:
                return None

            weight = (readings[i] - white) / span
            if weight < 0:
                weight = 0
            elif weight > 1:
                weight = 1
            weights.append(weight)

        return weights


    def run_line_controller(self, weights, total):
        centroid_mm = 0
        for i in range(len(weights)):
            centroid_mm += self.positions[i] * weights[i]
        centroid_mm /= total

        correction = self.line_kp.value * centroid_mm
        self.left_sp.value = self.limit_wheel_speed(
            self.base_speed.value - correction
        )
        self.right_sp.value = self.limit_wheel_speed(
            self.base_speed.value + correction
        )
        self.l_go.value = True
        self.r_go.value = True


    def run_yaw_controller(self, yaw_rate_request):
        # IMU yaw-rate P control: error becomes a wheel-speed correction.
        yaw_rate = self.imu.yaw_rate_rad_s()
        correction = self.yaw_kp * (yaw_rate_request - yaw_rate)

        self.left_sp.value = self.limit_wheel_speed(
            self.base_speed.value - correction
        )
        self.right_sp.value = self.limit_wheel_speed(
            self.base_speed.value + correction
        )
        self.l_go.value = True
        self.r_go.value = True


    def update_lateral_displacement(self, now):
        dt = ticks_diff(now, self.previous_update_time) / 1_000_000
        self.previous_update_time = now
        if dt <= 0:
            return

        counts_to_rad = 2 * pi / self.counts_per_rev
        left_omega = self.left_enc.get_velocity() * counts_to_rad * 1_000_000
        right_omega = self.right_enc.get_velocity() * counts_to_rad * 1_000_000
        forward_speed = self.wheel_radius * (left_omega + right_omega) / 2

        psi = self.imu.heading_rad() - self.heading_start
        psi = (psi + pi) % (2 * pi) - pi
        self.lateral_displacement += forward_speed * sin(psi) * dt


    def run(self):
        while True:

            if self.state == S0_WAIT:
                if self.man_go.value:
                    # M was pressed: this is the only entry into the maneuver.
                    self.man_done.value = False
                    self.line_go.value = True
                    self.start_follow_time = ticks_us()
                    self.state = S2_MAN_FOLLOW_START
                    print("Lane-change maneuver started")

                elif self.line_go.value:
                    # F was pressed: ordinary line following only.
                    self.state = S1_LINE_FOLLOW
                    print("Line following started")


            elif self.state == S1_LINE_FOLLOW:
                if not self.line_go.value:
                    self.stop()
                    self.state = S0_WAIT
                    yield self.state
                    continue

                weights = self.get_weights()
                if weights is None or sum(weights) < self.lost_line_sum:
                    print("Line lost or calibration invalid")
                    self.stop()
                    self.state = S0_WAIT
                else:
                    self.run_line_controller(weights, sum(weights))


            elif self.state == S2_MAN_FOLLOW_START:
                if not self.man_go.value or not self.line_go.value:
                    self.finish_maneuver("Lane-change maneuver cancelled")
                else:
                    weights = self.get_weights()
                    if weights is None or sum(weights) < self.lost_line_sum:
                        self.finish_maneuver("Starting line lost")
                    else:
                        self.run_line_controller(weights, sum(weights))

                        if (ticks_diff(ticks_us(), self.start_follow_time) /
                                1_000_000 >= self.depart_after_s):
                            self.heading_start = self.imu.heading_rad()
                            self.lateral_displacement = 0.0
                            self.previous_update_time = ticks_us()
                            self.maneuver_start_time = self.previous_update_time
                            self.white_samples = 0
                            self.white_cleared = False
                            self.state = S3_DEPART
                            print("IMU departure started")


            elif self.state == S3_DEPART:
                now = ticks_us()
                if not self.man_go.value or not self.line_go.value:
                    self.finish_maneuver("Lane-change maneuver cancelled")
                elif (ticks_diff(now, self.maneuver_start_time) /
                        1_000_000 > self.timeout_s):
                    self.finish_maneuver("Lane-change timeout")
                else:
                    self.update_lateral_displacement(now)
                    self.run_yaw_controller(self.turn_sign * self.yaw_rate_sp)

                    if self.turn_sign * self.lateral_displacement >= self.d/2:
                        self.state = S4_TURN_BACK
                        print("d = {:.3f} m; turn back".format(
                            self.lateral_displacement
                        ))


            elif self.state == S4_TURN_BACK:
                now = ticks_us()
                if not self.man_go.value or not self.line_go.value:
                    self.finish_maneuver("Lane-change maneuver cancelled")
                #elif (ticks_diff(now, self.maneuver_start_time) /
                #        1_000_000 > self.timeout_s):
                #    self.finish_maneuver("Target line missed")
                else:
                    self.update_lateral_displacement(now)
                    self.run_yaw_controller(-self.turn_sign * self.yaw_rate_sp)

                    weights = self.get_weights()

                    if weights is None:
                        self.finish_maneuver("Line calibration invalid")

                    else:
                        total = sum(weights)
                        if not self.white_cleared:
                            if total <= self.white_sum_max:
                                self.white_samples += 1
                            else:
                                self.white_samples = 0

                            if self.white_samples >= self.white_samples_required:
                                self.white_cleared = True
                                print("Original line cleared")

                        if total >= self.target_line_sum:
                            print("bingus")
                            print("Target line found")
                            self.run_line_controller(weights, total)
                            self.state = S5_FOLLOW_TARGET


            elif self.state == S5_FOLLOW_TARGET:
                if not self.man_go.value or not self.line_go.value:
                    self.finish_maneuver("Lane-change maneuver cancelled")
                else:
                    weights = self.get_weights()
                    
                    if weights is None:
                        self.finish_maneuver("Line calibration invalid")

                    elif all(weight >= self.black_weight_min for weight in weights):
                        print('bingus2')
                        self.finish_maneuver("Stop bar detected")

                    elif sum(weights) < self.lost_line_sum:
                        self.finish_maneuver("Target line lost")

                    else:
                        self.run_line_controller(weights, sum(weights))


            elif self.state == S6_DONE:
                # TaskUser acknowledges this flag, then this task returns idle.
                if not self.man_done.value:
                    self.state = S0_WAIT


            else:
                self.finish_maneuver("TaskLineC state error")

            yield self.state
