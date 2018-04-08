import rospy

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

THROTTLE_KP = 0.3
THROTTLE_KI = 0.1
THROTTLE_KD = 0.005
THROTTLE_TAU = 0.5 # cut-off freq = 1/(2*pi*tau)
THROTTLE_TS = 0.2 # sample time (1/RATE)




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

		# convert velocity to m/s
		self.MAX_VELOCITY = self.MAX_VELOCITY * 0.2778

		self.throttle_PID = PID(THROTTLE_KP, THROTTLE_KI, THROTTLE_KD, 
								-1*self.acceleration_limit, self.acceleration_limit)

		yaw_controller_init_kwargs = {
			'wheel_base' : self.wheel_base,
			'steer_ratio' : self.steer_ratio,
			'max_lateral_acceleration' : self.max_lateral_acceleration,
			'min_speed' : 0.1,
			'max_steer_angle' : self.max_steer_angle
		}
		self.yaw_controller = YawController(**yaw_controller_init_kwargs)
		self.throttle_lowpass = LowPassFilter(THROTTLE_TAU,THROTTLE_TS)

		self.last_time = rospy.get_time()

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

		rospy.loginfo('Current velocity: {}'.format(current_velocity))
		rospy.loginfo('target_linear_velocity : {}'.format(target_linear_velocity))
		# pass current_velocity through low pass filter to reduce high freq variations
		current_velocity = self.throttle_lowpass.filter(current_velocity)
		# if driver has taken control, reset throttle PID to avoid accumulation
		# of error
		if not dbw_enabled:
			self.throttle_PID.reset()
			self.throttle_lowpass.reset()


		error = target_linear_velocity - current_velocity
		throttle = self.throttle_PID.step(error, dt)

		steer = self.yaw_controller.get_steering(target_linear_velocity, target_angular_velocity, 
												 current_velocity)
		#steer = self.steer_PID.step(steer,dt)
		#steer = self.steer_lowpass.filter(steer)

		# if target linear velocity is very less, apply hard brake
		brake = 0.0
		if target_linear_velocity == 0.0 and current_velocity < 0.1:
			brake = 400
			throttle = 0.0
		
		elif throttle < 0.1 and error < 0:
			throttle = 0.0
			decel = min(abs(error/dt), abs(self.deceleration_limit))
			brake = decel * self.brake_torque
		return throttle, brake, steer
