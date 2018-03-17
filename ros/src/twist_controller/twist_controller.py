from pid import PID
from yaw_controller import YawController
from params import PARAMETERS

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = PARAMETERS['min_speed']
        self.max_steer_angle = max_steer_angle
        self.max_lat_accel = max_lat_accel

    def control(self, linear_velocity, angular_velocity, current_linear_velocity, 
                dbw_enabled):
        throttle_pid = PID()
        pid_value = throttle_pid.step(throttle_pid.last_error,1)	
        yaw_controller = YawController(self.wheel_base,
                                       self.steer_ratio, 
                                       self.min_speed,
                                       self.max_lat_accel,
                                       self.max_steer_angle)

        steer_value = yaw_controller.get_steering(linear_velocity,
                                                  angular_velocity,
                                                  current_linear_velocity)

        brake_value = 0
        if (pid_value <0. ):
	        brake_value = -pid_value
        return pid_value, brake_value, steer_value
