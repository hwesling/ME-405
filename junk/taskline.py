
""" MECHA 15 - ROMI """
""" Outer-loop controller with line following and IMU yaw-rate feedback """

import time
import math


S0_FOLLOWING = 0
S1_DEPART = 1
S2_TURNING = 2
S3_STOPPING = 3
S4_ERROR = 4


class TaskLine:

    def __init__(self, read_values, calibration, positions,
                 left_sp, right_sp, base_speed, line_kp,
                 line_go, l_go, r_go, imu, man_go, man_done):

        self.state = S0_FOLLOWING

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

        self.man_go = man_go
        self.man_done = man_done

        # Lane-change configuration
        self.direction = 1            # 1 = left, -1 = right
        self.departure_delay_s = 2.0
        self.max_maneuver_time_s = 20.0

        # S-turn configuration
        self.turn_angle = math.radians(45)
        self.turn_start_heading = 0.0
        self.heading_start = 0.0
        self.searching_for_line = False

        # Yaw-rate controller
        self.yaw_rate_command = 0.6
        self.yaw_rate_kp = 0.8

        # Sensor thresholds
        self.black_threshold = 0.8
        self.white_threshold = 0.2
        self.white_gap_time_s = 0.05

        self.white_since = None
        self.white_seen = False

        # Maneuver tracking
        self.lateral_disp = 0.0
        self.last_update_time = None
        self.maneuver_start_time = None
        self.line_follow_start_time = None
        self.line_detected_start_time = None

        self.new_line = False
        self.lane_change_done = False
        self.run_active = False
        self.maneuver_requested = False

    def stop(self):
        self.left_sp.value = 0
        self.right_sp.value = 0
        self.l_go.value = False
        self.r_go.value = False

    def set_wheel_speeds(self, left, right):
        """Apply wheel-speed setpoints and enable both motors."""

        self.left_sp.value = left
        self.right_sp.value = right
        self.l_go.value = True
        self.r_go.value = True

    def set_yaw_rate(self, commanded_rate):
        measured_rate = self.imu.yaw_rate_rad_s()
    
        print("Yaw command:", commanded_rate,
              "Measured yaw rate:", measured_rate)
    
        yaw_correction = self.yaw_rate_kp * (
            commanded_rate - measured_rate
        )
    
        # Limit the correction to reduce aggressive wheel-speed changes.
        max_correction = 1.0
        yaw_correction = max(
            -max_correction,
            min(max_correction, yaw_correction)
        )
    
        base = abs(self.base_speed.value)
    
        left_sp = base - yaw_correction
        right_sp = base + yaw_correction
    
        print("Correction:", yaw_correction,
              "Left SP:", left_sp,
              "Right SP:", right_sp)
    
        self.set_wheel_speeds(left_sp, right_sp)
     
    def calibration_valid(self, calibration, readings):

        if not isinstance(calibration, dict):
            return False

        if "white" not in calibration or "black" not in calibration:
            return False

        if len(readings) != len(self.positions):
            return False

        if (len(calibration["white"]) != len(readings) or
                len(calibration["black"]) != len(readings)):
            return False

        for i in range(len(readings)):
            if calibration["black"][i] <= calibration["white"][i]:
                return False

        return True

    def sensor_weights(self, readings, calibration):
        """Normalize sensor readings: white ~= 0, black ~= 1."""

        weights = []

        for i in range(len(readings)):
            white = calibration["white"][i]
            black = calibration["black"][i]

            weight = (readings[i] - white) / (black - white)
            weight = max(0.0, min(1.0, weight))
            weights.append(weight)

        return weights

    def all_black(self, weights):
        """True when every sensor detects black."""

        if len(weights) != len(self.positions):
            return False

        return all(
            weight >= self.black_threshold for weight in weights
        )

    def all_white(self, weights):
        """True when every sensor detects the white background."""

        if len(weights) != len(self.positions):
            return False

        return all(
            weight <= self.white_threshold for weight in weights
        )

    def centroid_calculation(self, weights):
        """Return the weighted line centroid, or None if no line is found."""

        total = sum(weights)

        if total < self.white_threshold:
            return None

        centroid_mm = 0.0

        for i in range(len(weights)):
            centroid_mm += self.positions[i] * weights[i]

        return centroid_mm / total


    def relative_heading(self):
        """Heading relative to the start of the lane-change maneuver."""

        heading = self.imu.heading_rad()

        return (
            (heading - self.heading_start + math.pi)
            % (2 * math.pi)
        ) - math.pi

    def heading_change(self, reference_heading):
        """Wrapped heading change relative to a reference heading."""

        heading = self.imu.heading_rad()

        return (
            (heading - reference_heading + math.pi)
            % (2 * math.pi)
        ) - math.pi

    def forward_speed(self):
        """Estimate forward speed from wheel-speed setpoints."""

        omega_left = self.left_sp.value
        omega_right = self.right_sp.value
        wheel_radius_m = 0.035

        return wheel_radius_m * (omega_left + omega_right) / 2.0

    def update_lateral_disp(self, now):
        """Integrate estimated lateral displacement during the maneuver."""

        if self.last_update_time is None:
            self.last_update_time = now
            return

        dt_ms = time.ticks_diff(now, self.last_update_time)
        self.last_update_time = now

        if dt_ms <= 0:
            return

        dt = dt_ms / 1000.0
        velocity = self.forward_speed()
        heading = self.relative_heading()

        self.lateral_disp += velocity * math.sin(heading) * dt

   
    def start_lane_change(self, now):
        """Initialize the first turn."""

        self.heading_start = self.imu.heading_rad()
        self.turn_start_heading = self.heading_start

        self.lateral_disp = 0.0
        self.last_update_time = now
        self.maneuver_start_time = now

        self.white_since = None
        self.white_seen = False
        self.searching_for_line = False

        self.state = S1_DEPART
        print("Starting first turn")

    def update_white_gap(self, weights, now):
        """Require continuous all-white readings before confirming a gap."""

        if self.white_seen:
            return

        if self.all_white(weights):

            if self.white_since is None:
                self.white_since = now

            elapsed = time.ticks_diff(now, self.white_since)

            if elapsed >= self.white_gap_time_s * 1000:
                self.white_seen = True
                print("White gap confirmed")

        else:
            self.white_since = None

    def finish_run(self):
        """Stop the robot and signal that the run has ended."""

        self.stop()
        self.line_go.value = False
        self.man_go.value = False
        self.man_done.value = True

        self.run_active = False
        self.state = S0_FOLLOWING


    def run(self):

        while True:

            # Immediate user stop
            if not self.line_go.value:
                self.stop()
                self.man_done.value = True
                self.man_go.value = False

                self.state = S0_FOLLOWING
                self.run_active = False

                yield 2
                continue

            calibration = self.calibration.value

            if calibration is None:
                print("No line calibration loaded")
                self.stop()
                self.line_go.value = False
                self.man_done.value = True
                self.run_active = False

                yield 2
                continue

            now = time.ticks_ms()
            # Accept a new M request while already following the line.
            
            if self.run_active and self.man_go.value:
                self.maneuver_requested = True
                self.lane_change_done = False
                self.line_follow_start_time = None
                self.man_done.value = False
                self.man_go.value = False

            # Initialize each new F or M run.
            if not self.run_active:

                self.maneuver_requested = self.man_go.value

                self.state = S0_FOLLOWING
                self.run_active = True

                self.new_line = False
                self.lane_change_done = False
                self.white_since = None
                self.white_seen = False
                self.searching_for_line = False

                self.heading_start = 0.0
                self.turn_start_heading = 0.0
                self.lateral_disp = 0.0
                self.last_update_time = None
                self.maneuver_start_time = None
                self.line_follow_start_time = None
                self.line_detected_start_time = None

                if self.maneuver_requested:
                    self.man_done.value = False
                else:
                    self.man_done.value = True

                # Capture the request before clearing the shared flag.
                self.man_go.value = False

            readings = self.read_values()

            # Validate sensor count and calibration.
            if not self.calibration_valid(calibration, readings):
                print("Invalid line calibration or sensor configuration")
                self.stop()
                self.line_go.value = False
                self.man_done.value = True
                self.run_active = False

                yield 2
                continue

            weights = self.sensor_weights(readings, calibration)
            centroid = self.centroid_calculation(weights)
            
   

            # Stop on the final black bar after reacquiring the new line.
            if self.new_line and self.all_black(weights):
                self.state = S3_STOPPING

            if self.state == S3_STOPPING:
                self.finish_run()
                yield 2
                continue

            if self.state == S4_ERROR:
                self.finish_run()
                yield 2
                continue

            centroid_mm = self.centroid_calculation(weights)

            # Update displacement while the maneuver is active.
            if self.state in (S1_DEPART, S2_TURNING):
                self.update_lateral_disp(now)


            if self.state == S0_FOLLOWING:

                if centroid_mm is None:
                    self.line_detected_start_time = None
                    self.line_follow_start_time = None
                    self.stop()

                    yield 2
                    continue

                if self.line_detected_start_time is None:
                    self.line_detected_start_time = now
                    self.line_follow_start_time = now
                    print("Line detected")


                # Assumes positive centroid means the line is to the
                # right, so the robot should steer right.
                CENTROID_OFFSET_MM = 1.25

                corrected_centroid = centroid_mm - CENTROID_OFFSET_MM
                line_correction = self.line_kp.value * corrected_centroid
                base = self.base_speed.value

                
                line_correction = max(-base, min(base, line_correction))
                
                self.set_wheel_speeds(
                    base - line_correction,
                    base + line_correction
                )
                                

                elapsed = time.ticks_diff(
                    now, self.line_follow_start_time
                )

                # Only M requests the lane-change maneuver.
                if (self.maneuver_requested
                        and not self.lane_change_done
                        and elapsed >= self.departure_delay_s * 1000):

                    self.lane_change_done = True
                    self.start_lane_change(now)

       


            elif self.state == S1_DEPART:
                commanded_rate = (
                    self.direction * self.yaw_rate_command
                )

                self.set_yaw_rate(commanded_rate)

                angle = self.heading_change(
                    self.turn_start_heading
                )

                print(
                    "FIRST TURN | command:", commanded_rate,
                    "| angle deg:", math.degrees(angle),
                    "| target deg:", math.degrees(self.turn_angle),
                    "| heading:", self.imu.heading_rad()
                )

                if self.direction * angle >= self.turn_angle:
                    self.turn_start_heading = self.imu.heading_rad()
                    self.white_since = None
                    self.white_seen = False
                    self.state = S2_TURNING
                    print("Switching to reverse turn")


            elif self.state == S2_TURNING:

                if not self.searching_for_line:
                    # Turn in the opposite direction from the first turn.
                    self.set_yaw_rate(
                        -self.direction * self.yaw_rate_command
                    )
               
                    angle = self.heading_change(self.turn_start_heading)
               
                    print("Second-turn angle:", math.degrees(angle))
               
                    if abs(angle) >= self.turn_angle:
                        self.searching_for_line = True
                        self.white_since = None
                        self.white_seen = False
                        print("S-turn angles reached; searching for new line")
               
                else:
                    # Move forward slowly to reacquire the line.
                    search_speed = min(abs(self.base_speed.value), 2.0)
               
                    self.set_wheel_speeds(search_speed, search_speed)
               
                    self.update_white_gap(weights, now)
               
                    if self.white_seen and centroid_mm is not None:
                        print("Target line detected:", centroid_mm)
               
                        self.state = S0_FOLLOWING
                        self.new_line = True
                        self.searching_for_line = False
                        self.maneuver_requested = False
               
                        self.line_detected_start_time = now
                        self.line_follow_start_time = now

            if self.state in (S1_DEPART, S2_TURNING):

                maneuver_elapsed = time.ticks_diff(
                    now, self.maneuver_start_time
                )

                if maneuver_elapsed >= self.max_maneuver_time_s * 1000:
                    print("Lane-change timeout: target line not acquired")

                    self.state = S4_ERROR
                    self.finish_run()

                    yield 2
                    continue

            yield 2
