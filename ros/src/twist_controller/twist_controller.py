#!/usr/bin/env python

from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0
        mx = accel_limit
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.last_vel = 0
        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        #rospy.logwarn(angular_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        current_time = rospy.get_time()

        desired_accel = self.throttle_controller.step(vel_error, current_time-self.last_time)

        self.last_time = current_time
        decel = max(vel_error, self.decel_limit)
        torque = abs(decel) * (self.vehicle_mass + self.fuel_capacity*GAS_DENSITY) * self.wheel_radius
        brake = 0.0


        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0.0
            brake = torque

        elif desired_accel < 0.1 and vel_error < 0:
            throttle = 0.0
            brake = torque
        else:
            throttle = desired_accel


        return throttle, brake, steering
