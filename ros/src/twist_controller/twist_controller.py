import rospy

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
KP = 1.0
KI = 0.05
KD = 0.25
TAU = 3
TS = 0.1


class Controller(object):
    
    def __init__(self, **kwargs):
        
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lateral_acceleration = kwargs['max_lateral_acceleration']
        self.max_steer_angle = kwargs['max_steer_angle']
        self.vehicle_mass = kwargs['vehicle_mass']
        self.wheel_radius = kwargs['wheel_radius']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.acceleration_limit = kwargs['acceleration_limit']
        self.deceleration_limit = kwargs['deceleration_limit']
        self.brake_deadband = kwargs['brake_deadband']

        # add mass of fuel to vehicle mass
        self.vehicle_mass = self.vehicle_mass + (self.fuel_capacity * GAS_DENSITY)

        self.brake_torque = self.vehicle_mass * self.wheel_radius

        # get max allowed velocity in kmph
        self.MAX_VELOCITY = rospy.get_param('/waypoint_loader/velocity')

        self.throttle_PID = PID(KP, KI, KD, -1*self.acceleration_limit, self.acceleration_limit)

        yaw_controller_init_kwargs = {
            'wheel_base' : self.wheel_base,
            'steer_ratio' : self.steer_ratio,
            'max_lateral_acceleration' : self.max_lateral_acceleration,
            'min_speed' : 0.0,
            'max_steer_angle' : self.max_steer_angle
        }
        self.yaw_controller = YawController(**yaw_controller_init_kwargs)
        self.lowpass = LowPassFilter(TAU,TS)


    def control(self, **kwargs):
        '''
        controls the vehicle taking into consideration the twist_cmd
        
        Params:
            - `target_linear_velocity` is the desired linear velocity the car should reach
            - `target_angular_velocity` is the desired angular velocity the car should reach
            - `current_velocity` is the current velocity of the car
            - `dbw_enabled` - Boolean, whether auto-drive is enabled or not
            - `dt` - time in seconds, between each twist_cmd

        Returns throttle, brake and steer values 
        '''
        target_linear_velocity = kwargs['target_linear_velocity']
        target_angular_velocity = kwargs['target_angular_velocity']
        current_velocity = kwargs['current_velocity']
        dbw_enabled = kwargs['dbw_enabled']
        dt = kwargs['dt']

        # if driver has taken control, reset throttle PID to avoid accumulation
        # of error
        if not dbw_enabled:
            self.throttle_PID.reset()

        error = min(target_linear_velocity, self.MAX_VELOCITY) - current_velocity
        throttle = self.throttle_PID.step(error, dt)
        throttle = self.lowpass.filter(throttle)

        steer = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, 
                                                 current_velocity)

        # if target linear velocity is very less, apply hard brake
        if target_linear_velocity < 0.1:
            brake = abs(self.deceleration_limit) * self.brake_torque
            return 0.0, brake, 0.0

        if throttle > 0:
            return throttle, 0.0, steer

        brake = 0
        if abs(throttle) > self.brake_deadband:
            brake = abs(throttle) * self.brake_torque

        return 0.0, brake, steer
