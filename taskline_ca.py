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


class TaskLineCA:
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

        # ---- Lane-change settings (+1 = left, -1 = right) ----
        self.d = 0.200              # lane spacing [m]
        self.turn_sign = 1
        self.depart_after_s = 0.5   # line following time before departing
        self.timeout_s = 6.0        # whole maneuver (S3 + S4)
        self.back_timeout_s = 2.5   # S4 alone: missed-target stop

        # ---- Yaw-rate feedback ----
        self.yaw_rate_sp = 2.0      # omega_0 [rad/s] for the departure ramp
        self.yaw_kp = 2.0           # [rad/s wheel] per [rad/s yaw error]
        self.maneuver_speed = 6     # v0 [rad/s wheel] during S3/S4
        self.heading_kp = 3.0       # S3: [rad/s yaw] per [rad heading]
        self.psi_cap = 1.0          # [rad] heading held during the S
        self.psi_limit = 1.3        # [rad] safety: force turn-back
        self.heading_tol = 0.10     # [rad] "back at original heading" band

        # S4 return law (gentler so it settles instead of swinging)
        self.back_kp = 1.5
        self.back_max = 1.5         # [rad/s]

        # ---- Geometry ----
        self.wheel_radius = 0.035
        self.track_width = 0.141
        self.counts_per_rev = 1440
        self.max_wheel_speed = 15.0

        # ---- Line detection thresholds ----
        self.lost_line_sum = 0.20
        self.target_line_sum = 0.50
        self.white_sum_max = 0.25
        self.black_weight_min = 0.80
        self.white_samples_required = 1
        self.clear_lateral = 0.04   # [m] old line is behind the array

        self.debug = True           # prints S4 status every 5th pass

        self.state = S0_WAIT
        self.start_follow_time = 0
        self.maneuver_start_time = 0
        self.back_start_time = 0
        self.previous_update_time = 0
        self.heading_start = 0.0
        self.lateral_displacement = 0.0
        self.white_samples = 0
        self.white_cleared = False
        self.ramp_done = False
        self.d_ramp = 0.0
        self.dbg = 0


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


    def run_yaw_controller(self, omega_sp):
        """IMU yaw-rate feedback: wheel difference = feedforward + P(error).
        Positive omega_sp turns left (right wheel faster)."""
        omega_meas = self.imu.yaw_rate_rad_s()

        feedforward = omega_sp * self.track_width / (2 * self.wheel_radius)
        delta = feedforward + self.yaw_kp * (omega_sp - omega_meas)

        v0 = self.maneuver_speed
        self.left_sp.value = self.limit_wheel_speed(v0 - delta)
        self.right_sp.value = self.limit_wheel_speed(v0 + delta)
        self.l_go.value = True
        self.r_go.value = True


    def relative_yaw(self):
        psi = self.imu.heading_rad() - self.heading_start
        return (psi + pi) % (2 * pi) - pi


    def update_lateral_displacement(self, now, psi):
        dt = ticks_diff(now, self.previous_update_time) / 1_000_000
        self.previous_update_time = now
        if dt <= 0:
            return

        counts_to_rad = 2 * pi / self.counts_per_rev
        left_omega = self.left_enc.get_velocity() * counts_to_rad * 1_000_000
        right_omega = self.right_enc.get_velocity() * counts_to_rad * 1_000_000
        forward_speed = self.wheel_radius * (left_omega + right_omega) / 2

        self.lateral_displacement += forward_speed * sin(psi) * dt


    def run(self):
        while True:

            if self.state == S0_WAIT:
                if self.man_go.value:
                    self.man_done.value = False
                    self.line_go.value = True
                    self.start_follow_time = ticks_us()
                    self.state = S2_MAN_FOLLOW_START
                    print("Lane-change maneuver started")

                elif self.line_go.value:
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
                            self.ramp_done = False
                            self.d_ramp = 0.0
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
                    psi = self.relative_yaw()
                    self.update_lateral_displacement(now, psi)

                    # ramp to +/- psi_cap at up to omega_0, then hold heading
                    cmd = self.heading_kp * (self.turn_sign * self.psi_cap - psi)
                    if cmd > self.yaw_rate_sp:
                        cmd = self.yaw_rate_sp
                    elif cmd < -self.yaw_rate_sp:
                        cmd = -self.yaw_rate_sp
                    self.run_yaw_controller(cmd)

                    lateral = self.turn_sign * self.lateral_displacement

                    if (not self.ramp_done
                            and self.turn_sign * psi >= self.psi_cap - 0.1):
                        self.ramp_done = True
                        self.d_ramp = lateral

                    # the ramp-down adds about d_ramp again, so turn back early
                    if ((self.ramp_done and lateral >= self.d - self.d_ramp)
                            or abs(psi) >= self.psi_limit):
                        self.back_start_time = now
                        self.state = S4_TURN_BACK
                        print("d_hat = {:.3f} m, psi = {:.2f} rad; turn back".format(
                            self.lateral_displacement, psi))


            elif self.state == S4_TURN_BACK:
                now = ticks_us()
                if not self.man_go.value or not self.line_go.value:
                    self.finish_maneuver("Lane-change maneuver cancelled")
                elif (ticks_diff(now, self.maneuver_start_time) /
                        1_000_000 > self.timeout_s):
                    self.finish_maneuver("Target line missed (maneuver timeout)")
                elif (ticks_diff(now, self.back_start_time) /
                        1_000_000 > self.back_timeout_s):
                    self.finish_maneuver("Target line missed")
                else:
                    # 1) update measurements first, every pass
                    psi = self.relative_yaw()
                    self.update_lateral_displacement(now, psi)

                    # 2) steer heading back toward 0 (yaw-rate feedback inside)
                    cmd = -self.back_kp * psi
                    if cmd > self.back_max:
                        cmd = self.back_max
                    elif cmd < -self.back_max:
                        cmd = -self.back_max
                    self.run_yaw_controller(cmd)

                    # 3) look for the target line
                    weights = self.get_weights()
                    if weights is None:
                        self.finish_maneuver("Line calibration invalid")
                    else:
                        total = sum(weights)

                        if self.debug:
                            self.dbg += 1
                            if self.dbg % 5 == 0:
                                print("S4 tot={:.2f} clr={} psi={:.2f} d={:.3f}".format(
                                    total, self.white_cleared, psi,
                                    self.lateral_displacement))

                        if not self.white_cleared:
                            if total <= self.white_sum_max:
                                self.white_samples += 1
                            else:
                                self.white_samples = 0

                            # old line cleared by: white seen, lateral travel,
                            # or heading back near the original direction
                            if (self.white_samples >= self.white_samples_required
                                    or self.turn_sign * self.lateral_displacement
                                    >= self.clear_lateral
                                    or abs(psi) <= self.heading_tol):
                                self.white_cleared = True
                                print("Original line cleared")

                        # same pass: hand off as soon as the target is seen
                        if self.white_cleared and total >= self.target_line_sum:
                            self.state = S5_FOLLOW_TARGET
                            print("Target line found")
                            self.run_line_controller(weights, total)


            elif self.state == S5_FOLLOW_TARGET:
                if not self.man_go.value or not self.line_go.value:
                    self.finish_maneuver("Lane-change maneuver cancelled")
                else:
                    weights = self.get_weights()
                    if weights is None:
                        self.finish_maneuver("Line calibration invalid")
                    elif all(weight >= self.black_weight_min for weight in weights):
                        self.finish_maneuver("Stop bar detected")
                    elif sum(weights) < self.lost_line_sum:
                        self.finish_maneuver("Target line lost")
                    else:
                        self.run_line_controller(weights, sum(weights))


            elif self.state == S6_DONE:
                if not self.man_done.value:
                    self.state = S0_WAIT


            else:
                self.finish_maneuver("TaskLineCA state error")

            yield self.state
