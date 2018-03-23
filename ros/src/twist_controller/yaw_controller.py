from math import atan

class YawController(object):
    def __init__(self, **kwargs):
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.min_speed = kwargs['min_speed']
        self.max_lateral_acceleration = kwargs['max_lateral_acceleration']
        self.max_steer_angle = kwargs['max_steer_angle']

        self.min_angle = -self.max_steer_angle
        self.max_angle = self.max_steer_angle


    def get_angle(self, radius):
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))

    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.

        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lateral_acceleration / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))

        return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
