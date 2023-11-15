from collections import deque
import math
import numpy as np
from sdc_course.utils.utility import *


class PIDLongitudinalController:
    """
    PIDLongitudinalController implements longitudinal control using a PID.
    """

    def __init__(self, vehicle, dt, K_P, K_D, K_I):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param dt: time differential in seconds
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        """

        self._vehicle = vehicle
        self._dt = dt
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._error_buffer = deque(maxlen=10) # Whenever we append new element, "oldest" one is being removed (so we are storing last 10 error values always)

    # Calls PID Control function and returns control i.e. acceleration
    def run_step(self, target_velocity_ms, debug=False):
        """
        Execute one step of longitudinal control to reach a given target velocity.

        :param target_velocity_ms: target velocity in m/s
        :param debug: boolean for debugging
        :return: throttle control
        """
        current_velocity_ms = get_velocity_ms(self._vehicle)

        if debug:
            print("Current velocity = {}".format(current_velocity_ms))

        return self._pid_control(target_velocity_ms, current_velocity_ms)

    # Generates control u (which is acceleration)
    def _pid_control(self, target_velocity_ms, current_velocity_ms):
        """
        Estimate the throttle/brake of the vehicle based on the PID equations

        :param target_velocity_ms:  target velocity in m/s
        :param current_velocity_ms: current velocity of the vehicle in m/s
        :return: throttle/brake control
        """
        acceleration = 0.0
        #######################################################################
        ################## TODO: IMPLEMENT LONGITUDINAL PID CONTROL HERE ######
        e_current = target_velocity_ms - current_velocity_ms
        self._error_buffer.append(e_current)

        P_control = self._k_p * e_current
        I_control = self._k_i * np.sum(np.multiply(self._error_buffer, self._dt))
        if(len(self._error_buffer) == 1):
            D_control = 0
        else:
            # D = Kd * ((e_curr - e_prev) / dt)
            D_control = self._k_d * np.divide(self._error_buffer[-1] - self._error_buffer[-2], self._dt)
        #######################################################################
        acceleration = P_control + I_control + D_control
        print(acceleration)
        # Converting real acceleration to the range -1 to 1 that represents throttle 
        # (because later in compute_control function in controller.py it compares it to max_throttle which is 0.75)
        # There is no logic comparing some acceleration that can be 10m/s^2 or higher with 0.75 => result will always be max_throttle => press max gas pedal)
        #acceleration_throttle = np.tanh(acceleration) # -max_thottle and max
        return acceleration


class PIDLateralController:
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self, vehicle, dt, K_P, K_D, K_I):
        """
        Constructor method.

        :param vehicle: actor to apply to local planner logic onto
        :param dt: time differential in seconds
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        """
        self._vehicle = vehicle
        self._dt = dt
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._error_buffer = deque(maxlen=10) # Whenever we append new element, "oldest" one is being removed (so we are storing last 10 error values always)

    # Calls PID Control function and returns control i.e. steering angle
    def run_step(self, waypoints):
        """
        Execute one step of lateral control to steer
        the vehicle towards a certain waypoint.

        :param waypoint: target waypoint
        :return: steering control
        """
        return self._pid_control(waypoints, self._vehicle.get_transform())

    def _get_steering_direction(self, v1, v2):
        """
        Note that Carla uses a left hand coordinate system, this is why a positive
        cross product requires a negative steering direction.
        :param v1: vector between vehicle and waypoint
        :param v2: vector in direction of vehicle
        :return: steering direction
        """
        cross_prod = v1[0] * v2[1] - v1[1] * v2[0]
        if cross_prod >= 0:
            return -1
        return 1

    def lateral_error(self, waypoints, vehicle_transform):
        # Extract current vehicle position
        vehicle_position = np.array([vehicle_transform.location.x, vehicle_transform.location.y])

        # Find the nearest waypoint to the current vehicle position
        nearest_waypoint = min(waypoints, key=lambda waypoint: np.linalg.norm([waypoint[0] - vehicle_position[0], waypoint[1] - vehicle_position[1]]))

        # Calculate steering direction
        v1 = [nearest_waypoint[0] - vehicle_transform.location.x, nearest_waypoint[1] - vehicle_transform.location.y]  
        v2 = [np.cos(np.radians(vehicle_transform.rotation.yaw)), np.sin(np.radians(vehicle_transform.rotation.yaw))]
        st_dir = self._get_steering_direction(v2,v1)

        # Calculate lateral error as a distance between vehicle position and nearest waypoint found above
        lat_error = st_dir * np.linalg.norm([nearest_waypoint[0] - vehicle_position[0], nearest_waypoint[1] - vehicle_position[1]])

        return lat_error

    # Generates control u (which is steering angle NOT steering rate)
    def _pid_control(self, waypoints, vehicle_transform):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoints: local waypoints
        :param vehicle_transform: current transform of the vehicle
        :return: steering control
        """
        steering = 0.0
        ######################################################################
        ################## TODO: IMPLEMENT LATERAL PID CONTROL HERE ###########
        target_cte = 0.0
        current_cte = self.lateral_error(waypoints, vehicle_transform)
        # Calculate steering_direction
        e_current = target_cte - current_cte
        #e_current = e_current * (-1)
        self._error_buffer.append(e_current)

        P_control = self._k_p * e_current
        I_control = self._k_i * np.sum(np.multiply(self._error_buffer, self._dt))
        if(len(self._error_buffer) == 1):
            D_control = 0
        else:
            # D = Kd * ((e_curr - e_prev) / dt)
            D_control = self._k_d * np.divide(self._error_buffer[-1] - self._error_buffer[-2], self._dt)
        #######################################################################
        # Calculate steering direction
        
        steering = P_control + D_control + I_control
        # Same explanation for steering angle as for acceleration above
        #steering_command = np.tanh(steering)
        return steering
