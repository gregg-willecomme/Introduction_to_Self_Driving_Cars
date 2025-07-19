#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np
import math
class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the brake command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('v_previous', 0.0)

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """

            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            Ku = 2.0
            Pu = 1.7
            Kp_t = 0.6 * Ku
            Ki_t = 2.0 * Kp_t / Pu
            Kd_t = Kp_t * Pu / 8.0 
            Kp_b = 0.05
            Kff = 0.5

            self.vars.create_var("throttle_integral", 0.0)
            self.vars.create_var("brake_integral", 0.0)
            self.vars.create_var('v_error_last_throttle', 0.0)
            self.vars.create_var('v_error_last_brake', 0.0)
            self.vars.create_var("t_last", t)
            self.vars.create_var('v_desired_last', 0.0)

            dt = t - self.vars.t_last if t > self.vars.t_last else 1e-5
            v_error_prop = v_desired - v

            a_ff = (v_desired - self.vars.v_desired_last) / dt
            x0_ff = Kff * a_ff


            throttle_derivative = (v_error_prop - self.vars.v_error_last_throttle) / dt

            # Calcul provisoire de l’intégrale
            provisional_integral = self.vars.throttle_integral + v_error_prop * dt

            # Calcul de la commande
            throttle_output = Kp_t * v_error_prop + Ki_t * provisional_integral + Kd_t * throttle_derivative
            throttle_output += x0_ff
            # Saturation
            throttle_output_clamped = max(0.0, min(1.0, throttle_output))

            # Anti-windup : n’accumule l’intégrale que si pas saturé
            if throttle_output == throttle_output_clamped:
                self.vars.throttle_integral = provisional_integral

            throttle_output = throttle_output_clamped
            brake_output = 0.0
            self.vars.v_error_last_throttle = v_error_prop


            if v_error_prop < 0:
                v_error_brake = -v_error_prop

                brake_output = Kp_b * v_error_brake
                brake_output = max(0.0, min(1.0, brake_output))

                self.vars.v_error_last_brake = v_error_brake

            self.vars.t_last = t
            self.vars.v_desired_last = v_desired

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            
            # Change the steer output with the lateral controller.

            # Vehicle parameters
            L = 2.9  # wheelbase [m]
            k = 0.1  # gain for dynamic lookahead
            Lfc = 2.5  # minimum lookahead distance [m]

            # Persistent variable to store last target index
            self.vars.create_var('last_index', 0)

            # Define a search window ahead of the last index
            ind_start = self.vars.last_index
            ind_end = min(len(waypoints), ind_start + 100)

            ind = None
            for i in range(ind_start, ind_end):
                dx = waypoints[i][0] - x
                dy = waypoints[i][1] - y
                distance = np.sqrt(dx**2 + dy**2)
                if distance > Lfc:
                    ind = i
                    break

            # If no valid waypoint found, use the last one in the window
            if ind is None:
                ind = ind_end - 1

            # Update last index for the next iteration
            self.vars.last_index = ind

            tx = waypoints[ind][0]
            ty = waypoints[ind][1]

            # Heading error between current yaw and target direction
            alpha_hat = math.atan2(ty - y, tx - x)
            alpha = alpha_hat - yaw
            alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # normalize to [-π, π]
            
            # Dynamic lookahead distance based on speed
            Lf = k * v + Lfc

            # Pure Pursuit steering command
            steer_output = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)







            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        # Logging de la vitesse pendant les 20 premières secondes
        # Logging de la vitesse pendant les 20 premières secondes
        self.vars.create_var('log_time', [])
        self.vars.create_var('log_speed', [])
        self.vars.create_var('logged_csv', False)

        if t <= 20.0:
            self.vars.log_time.append(t)
            self.vars.log_speed.append(v)

        # Écrit le fichier CSV une seule fois après t > 20 s
        if t > 20.0 and not self.vars.logged_csv:
            import csv
            with open("log.csv", "w", newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "speed_mps"])
                for ti, vi in zip(self.vars.log_time, self.vars.log_speed):
                    writer.writerow([f"{ti:.4f}", f"{vi:.4f}"])
            self.vars.logged_csv = True
            print("✅ Fichier log.csv écrit à t > 20 s")

        self.vars.v_previous = v  # Store forward speed to be used in next step
