import rospy
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
Kp = 1.0
Ki = 0.05
Kd = 0.25
TAU = 3
TS = 0.1


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, 
    				max_steer_angle, vehicle_mass, wheel_radius, fuel_capacity,
    				accel_limit, decel_limit, brake_deadband):
        
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.fuel_capacity = fuel_capacity
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.brake_deadband = brake_deadband

        #add mass of fuel to vehicle mass
        self.vehicle_mass = self.vehicle_mass + (fuel_capacity * GAS_DENSITY)

        self.brake_torque = self.vehicle_mass * self.wheel_radius

        #get max allowed velocity in kmph
        self.MAX_VELOCITY = rospy.get_param('/waypoint_loader/velocity')

        self.throttle_PID = PID(Kp, Ki, Kd, -1*self.accel_limit, self.accel_limit)
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0.0, 
        									self.max_lat_accel, self.max_steer_angle)
        self.lowpass = LowPassFilter(TAU,TS)


    def control(self, target_linear_vel, target_angular_vel, current_vel, dbw_status, dt):
    	'''
    	if driver has taken control, reset throttle PID to avoid accumulation of error
    	'''
    	if dbw_status is False:
    		self.throttle_PID.reset()

    	error = min(target_linear_vel, self.MAX_VELOCITY) - current_vel
    	proposed_throttle = self.throttle_PID.step(error, dt)
    	proposed_throttle = self.lowpass.filt(proposed_throttle)

    	proposed_steer = self.yaw_controller.get_steering(target_linear_vel, target_angular_vel, current_vel)

    	#if target linear velocity is very less, apply hard brake
    	if target_linear_vel < 0.1:
    		throttle = 0.0
    		brake = abs(self.decel_limit) * self.brake_torque
    		steer = 0.0

    	else:

    		#if proposed_throttle > 0, implies accelerating
    		if proposed_throttle > 0:
    			throttle = proposed_throttle
    			brake = 0.0
    			steer = proposed_steer

    		else:
    			throttle = 0.0
    			steer = proposed_steer

    			if abs(proposed_throttle) > self.brake_deadband:
    				brake = abs(proposed_throttle) * self.brake_torque

    			else :
    				brake = 0.0
        
        return throttle, brake, steer
