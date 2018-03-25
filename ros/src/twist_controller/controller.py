
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

import math
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter


class Controller(object):
    def __init__(self, car_data):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base=car_data.wheel_base,
                                            steer_ratio=car_data.steer_ratio,
                                            min_speed=car_data.min_speed,
                                            max_lat_accel=car_data.max_lat_accel,
                                            max_steer_angle=car_data.max_steer_angle)
        self.car_data = car_data
        self.brake_deadband = car_data.brake_deadband
        self.prev_time = None
        self.pid = PID(kp=0.7, ki=0.003, kd = 0.1, mn = car_data.decel_limit, mx = car_data.accel_limit)
        self.steer_lpf = LowPassFilter(tau = 0.9, ts = 0.8)
        #self.throttle_lpf = LowPassFilter(tau = 3, ts = 1)
        self.dbw_enabled = False
        

    def control(self, current_time, ref_lin_vel, ref_ang_vel, act_lin_vel, act_ang_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs

        if not self.dbw_enabled and dbw_enabled:
            self.pid.reset()
            
        self.dbw_enabled = dbw_enabled

        if not dbw_enabled or self.prev_time is None:
            self.prev_time = current_time
            return 0.0, 0.0, 0.0

        time_diff = current_time - self.prev_time
        self.prev_time = current_time

        value = self.pid.step(ref_lin_vel - act_lin_vel, time_diff)

        if value > 0:
            throttle = value
            brake = 0.0
        elif math.fabs(value) > self.brake_deadband:
            throttle = 0.0
            brake = (self.car_data.vehicle_mass + (self.car_data.fuel_capacity * GAS_DENSITY)) * -value * self.car_data.wheel_radius
        else:
            throttle = 0.0
            brake = 0.0

        if ref_lin_vel == 0 and act_lin_vel < 0.5:
            throttle = 0.0
            temp_brake_val = (self.car_data.vehicle_mass + (self.car_data.fuel_capacity * GAS_DENSITY)) * -1.0 * self.car_data.wheel_radius
            brake = max(brake, temp_brake_val)

        steer = self.yaw_controller.get_steering(ref_lin_vel, ref_ang_vel, act_lin_vel)
        steer = self.steer_lpf.filt(steer)

        # Return throttle, brake, steer
        return throttle, brake, steer

    #def update_gains(self, kp, ki, kd):
    #    self.pid.set_gain_values(kp=kp, ki=ki, kd=kd)
