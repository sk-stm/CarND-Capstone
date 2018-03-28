from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
from cplotter import Cplotter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, wheel_radius, decel_limit,
                 accel_limit, max_steer_angle, wheel_base, steer_ratio,
                 max_lat_accel, loop_rate):
        self._sample_time = 1.0/loop_rate

        mass = (vehicle_mass + fuel_capacity*GAS_DENSITY)

        # Implement the PID controller for the throttle and brake combined.
        # For positive values of control, the throttle will be used.
        # For negative values of control, it will be converted to brake.
        v_kp = 0.2
        v_ki = 0.002
        v_kd = 0.005
        self.vel_pid = PID(kp=v_kp, ki=v_ki, kd=v_kd, mn=-1.0, mx=1.0)
        self.max_brake = decel_limit * mass * wheel_radius

        min_speed = 10
        self._yaw_controller = YawController(wheel_base, steer_ratio, min_speed,
                                             max_lat_accel, max_steer_angle)

        self.lpf_steering = LowPassFilter(tau=2, ts=5)

    def control(self, proposed_lin_vel, proposed_ang_vel, current_lin_vel):
        steering_angle = self._yaw_controller.get_steering(proposed_lin_vel,
                                                           proposed_ang_vel,
                                                           current_lin_vel)
        final_steering_angle = self.lpf_steering.filt(steering_angle)

        vel_error = proposed_lin_vel - current_lin_vel
        vel_cmd = self.vel_pid.step(vel_error, self._sample_time)
        if proposed_lin_vel >= 1.5:
            throttle = vel_cmd
            brake = 0
        else:
            throttle = 0
            brake = - self.max_brake * 0.4

        # Return throttle, brake, steer
        return throttle, brake, final_steering_angle
