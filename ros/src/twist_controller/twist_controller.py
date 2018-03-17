import rospy

from lowpass import LowPassFilter
from params import PARAMETERS
from pid import PID
from yaw_controller import YawController



class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = PARAMETERS['min_speed']
        self.max_steer_angle = max_steer_angle
        self.max_lat_accel = max_lat_accel

        self.last_timestamp = None

    def get_throttle(self, cte, dt):
        '''returns the throttle given a cte and a dt.
            Params:
                - `cte` : error
                - `dt` : timestamp
        '''
        throttle_pid = PID()
        
        throttle_value = throttle_pid.step(cte, dt)
        
        return throttle_value

    def get_steering(self, target_v, target_w, current_v):
        '''returns the steering value.
            Params:
                - `target_v` : target linear velocity
                - `target_w` : target angular velocity
                - `current_v` : current linear velocity
        '''
        yaw_controller = YawController(self.wheel_base,
                                       self.steer_ratio, 
                                       self.min_speed,
                                       self.max_lat_accel,
                                       self.max_steer_angle)
        steering = yaw_controller.get_steering(linear_velocity=target_v.x,
                                               angular_velocity=target_w.z,
                                               current_velocity=current_v.x)
        return steering
    
    def get_brake(self, cte):
        brake = 0.
        if cte < 0:
            brake = max(-10.0 * cte, 1.0)            
        return brake

    def control(self, target_v, target_w, current_v, dbw_enabled):
        '''controls the car
            - `target_v` is the target linear velocity
            - `target_w` is the target angular velocity
            - `current_v` is the car's current velocity.
            - `dbw_enabled` is the state of the drive-by-wire
        '''
        if not self.last_timestamp or not target_v or not current_v:
            self.last_timestamp = rospy.get_time()
            return 0.0, 0.0, 0.0

        now = rospy.get_time()
        dt = now - self.last_timestamp
        
        target_velocity = target_v.x
        if target_velocity > PARAMETERS['max_speed']:
            target_velocity = PARAMETERS['max_speed']

        cte = target_velocity - current_v.x
        throttle = self.get_throttle(cte, dt) if cte >= 0 else 0.
        brake = self.get_brake(cte) if cte < 0 else 0.
        steer = self.get_steering(target_v, target_w, current_v)
        self.last_timestamp = now
        
        return throttle, brake, steer
