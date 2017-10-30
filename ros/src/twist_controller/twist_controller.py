from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController

GAS_DENSITY = 2.858
MPH_TO_MPS = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.vehicle_mass = args[0]
        self.fuel_capacity = args[1]
        self.brake_deadband = args[2]
        self.decel_limit = args[3]
        self.accel_limit = args[4]
        self.wheel_radius = args[5]
        self.wheel_base = args[6]
        self.steer_ratio = args[7]
        self.max_lat_accel = args[8]
        self.max_steer_angle = args[9]

        self.min_speed = 0.0

        self.Kp_speed = 0.5
        self.Ki_speed = 0.005
        self.Kd_speed = 0.2
        self.Kp_steering = 5.0
        self.Ki_steering = 0.005
        self.Kd_steering = 1.0

        # PID controller
        self.speed_controller = PID(self.Kp_speed, self.Ki_speed, self.Kd_speed, self.decel_limit, self.accel_limit)
        self.steering_controller = PID(self.Kp_steering, self.Ki_steering, self.Kd_steering, -1.0, 1.0)

        # Lowpass filter
        self.filter_steering = LowPassFilter(tau=4,ts=1)

        # Yaw controller: convert target linear and angular velocity to steering commands
        self.yaw_contoller = YawController(self.wheel_base, self.steer_ratio, self.min_speed, self.max_lat_accel, self.max_steer_angle)

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        target_linear_vel = args[0]
        target_angular_vel = args[1]
        current_linear_vel = args[2]
        current_angular_vel = args[3]
        dbw_enabled = args[4]
        delta_time = args[5]

        # Errors of linear velocity and angular velocity
        delta_linear_vel = (target_linear_vel - current_linear_vel)
        delta_angular_vel = (target_angular_vel - current_angular_vel)

        # Manual mode
        if dbw_enabled is False:
            self.speed_controller.reset()
            self.steering_controller.reset()

        #linear_vel = self.speed_controller.step(delta_linear_vel, delta_time)
        #angular_vel = self.steering_controller.step(delta_angular_vel, delta_time)

        # throttle = 0
        # brake = 0
        # steer = 0
        acceleration = max(min(delta_linear_vel, self.accel_limit), self.decel_limit)

        # Calculate throttle and brake
        linear_vel = self.speed_controller.step(acceleration, delta_time)
        # Slow down to 0
        if target_linear_vel < 0.01 and current_linear_vel < MPH_TO_MPS:
            brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * abs(self.decel_limit)
            throttle = 0
        # Keep moving
        elif linear_vel > 0:
            throttle = linear_vel
            brake = 0
        # Just slow down and keep the current_velocity
        else:
            brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * abs(linear_vel)
            throttle = 0

        # Calculate steer = correct + predict
        corrective_steer = self.steering_controller.step(delta_angular_vel, delta_time)
        predictive_steer = self.yaw_contoller.get_steering(target_linear_vel, target_angular_vel, current_linear_vel)
        steer = corrective_steer + predictive_steer
        # Add Lowpass filter
        steer = self.filter_steering.filt(steer)

        return throttle, brake, steer
