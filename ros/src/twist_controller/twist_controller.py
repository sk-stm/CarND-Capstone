from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, wheel_radius,
                 decel_limit, accel_limit, max_steer_angle,
                 wheel_base, steer_ratio, max_lat_accel):

        self._sample_time = 0.02 # 50Hz
        # Not sure we need to add mass of fuel, besides how do we know the
        # gas tank is full?
        mass = (vehicle_mass + fuel_capacity*GAS_DENSITY)

        # Implement speed control using a PID controller with control output
        # in the range [-1.0 1.0]
        # Since the simulator expects the throttle in the range [0 1.0],
        # we can simply use any non-negative values of PID control for
        # the throttle.
        # The brake expects values in torque (Nm), so we scale the the
        # negative values of the PID control in the range [-1.0 0] so that
        # the maximum control (-1.0) corresponds to decel_limit.
        # So the max brake torque value is as follows (note: this is a negative value
        # since decel_limit is negative, but that's okay since we will scale the
        # -ve PID output values with this to give +ve brake values):

        self.max_brake = decel_limit * mass * wheel_radius

        v_kp = 0.2
        v_ki = 0.005
        v_kd = 0.01
        self.vel_pid = PID(kp=v_kp, ki=v_ki, kd=v_kd, mn=-1.0, mx=1.0)  # speed controller

        self._yaw_controller = YawController(wheel_base=wheel_base, steer_ratio=steer_ratio,
                                             min_speed=10, max_lat_accel=max_lat_accel,
                                             max_steer_angle=max_steer_angle)

        self.lpf_steering = LowPassFilter(tau=2, ts=5)
        pass

    def control(self, proposed_lin_vel, proposed_ang_vel, current_lin_vel):
        steering_angle = self._yaw_controller.get_steering(linear_velocity=proposed_lin_vel,
                                                           angular_velocity=proposed_ang_vel,
                                                           current_velocity=current_lin_vel)
        final_steering_angle = self.lpf_steering.filt(steering_angle)

        throttle = 0
        brake = 0
        vel_error = proposed_lin_vel - current_lin_vel
        vel_cmd = self.vel_pid.step(vel_error, self._sample_time)
        if vel_cmd >= 0:
            throttle = vel_cmd
            brake = 0
        else:
            throttle = 0
            brake = self.max_brake * vel_cmd

        # TODO: Not sure what to do about brake_deadband??

        # Return throttle, brake, steer
        return throttle, brake, final_steering_angle
