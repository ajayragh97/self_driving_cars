import math
import numpy as np
from sdc_course.utils.utility import *


class PurePursuitLateralController:
    """
    PurePursuitLateralController implements lateral control using the pure pursuit controller.
    """

    def __init__(self, vehicle, L, ld, K_pp):
        self._vehicle = vehicle
        self._L = L
        self._ld = ld
        self._k_pp = K_pp

    def run_step(self, waypoints):
        return self._pure_pursuit_control(waypoints, self._vehicle.get_transform())

    def _get_goal_waypoint_index(self, vehicle, waypoints, lookahead_dist):
        for i in range(len(waypoints)):
            dist = compute_distance_to_waypoint(vehicle, waypoints[i])
            if dist >= lookahead_dist:
                return max(0, i)
        return len(waypoints) - 1

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

    def _pure_pursuit_control(self, waypoints, vehicle_transform):
        """
        :param waypoint: list of waypoints
        :param vehicle_transform: current transform of the vehicle
        :return: steering control
        """
        steering = 0.0
        #######################################################################
        ################## TODO: IMPLEMENT PURE-PURSUIT CONTROL HERE ##########
        #######################################################################
        velocity            = get_velocity_ms(self._vehicle) # velocity of vehicle in m/s

        ld            = self._k_pp * velocity # lookahead distance in meters
        if(ld == 0):
            ld = self._ld


        goal_waypoint_index = self._get_goal_waypoint_index(self._vehicle, waypoints, ld)
        goal_waypoint       = waypoints[goal_waypoint_index]
        
        # calculating the angle alpha between vehicle's body heading and the look ahead line 
        x_offset = goal_waypoint[0] - vehicle_transform.location.x
        y_offset = goal_waypoint[1] - vehicle_transform.location.y
        
        

        alpha = np.arctan(y_offset / x_offset) - np.radians(vehicle_transform.rotation.yaw)
        
        if alpha > np.pi/2:
            alpha -= np.pi
        if alpha < - np.pi/2:
            alpha += np.pi

        steering = np.arctan((2 * self._L * np.sin(alpha)) / ld)
        
        return steering
