import numpy as np
import rospy
from scipy.spatial import KDTree

from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane, Waypoint

from behaviour import (
	accelerate,
	decelerate,
	distance,
	MAX_DECELERATION,
	MAX_VELOCITY_PERCENT,
	STOP_DISTANCE_RATIO,
	stop
)


class BaseBehaviour(object):
	'''handles the methods shared between the different states.'''

	def update(self, car_index, current_velocity, max_velocity,
		stop_index, waypoints):
		'''update the state.'''
		self.current_velocity = current_velocity
		self.car_index = car_index
		self.max_velocity = max_velocity
		self.stop_index = stop_index
		self.waypoints = waypoints


class Starting(BaseBehaviour):
	'''This class handles the `Starting` state of the Behaviour.

	If there is no traffic light in sight or if the traffic light is green, 
	switch to `Accelerating`.
	Wait for a green signal otherwise.
	'''

	def process(self):
		if self.stop_index == -1:
			rospy.loginfo('switching behaviour : Starting to Accelerating')
			return Accelerating(), accelerate(self.waypoints,
				self.current_velocity,
				self.max_velocity)

		return None, stop(self.waypoints, 0, len(self.waypoints)-1)

		

class Accelerating(BaseBehaviour):
	'''This class handles the `Accelerating` state of the Behaviour.

	If there is a yellow or red signal in sight within the halting distance,
	switch to `Decelerating`.
	Keep accelerating otherwise.
	'''
	
	def process(self):
		# handling yellow traffic light
		# if self.traffic_light_in_sight and self.traffic_light.state in [TrafficLight.YELLOW, TrafficLight.RED]:
		if self.stop_index != -1:
			# if the car can physically stop, stop it; otherwise continue accelerating
			distance_to_halt = self.current_velocity ** 2 / (2 * MAX_DECELERATION)
			if self.stop_index <= distance_to_halt * STOP_DISTANCE_RATIO:
				rospy.loginfo('switching behaviour : Accelerating to Decelerating')
				return Decelerating(), decelerate(waypoints=self.waypoints,
					car_index=self.car_index,
					target_index=self.stop_index)
		
		return None, accelerate(self.waypoints,
				self.current_velocity,
				self.max_velocity)


class Decelerating(BaseBehaviour):
	'''This class handles the `Decelerating` state of the Behaviour.

	If there is no traffic light in sight, switch to `Accelerating`.
	If there is a green traffic light in sight, switch to `Accelerating`.
	Keep decelerating otherwise.
	'''

	def process(self):
		# no traffic light in sight, no reason to decelerate
		if self.stop_index == -1:
			rospy.loginfo('switching behaviour : Decelerating to Accelerating')
			return Accelerating(), accelerate(self.waypoints,
				self.current_velocity,
				self.max_velocity)

		return None, decelerate(waypoints=self.waypoints,
						car_index=self.car_index,
						target_index=self.stop_index)

MAX_ACCELERATION = 5 # m/s^2
MAX_DECELERATION = 5 #m/s^2
TRAFFIC_LIGHT_THRESHOLD_DISTANCE = 5 # m

class DefaultBehaviour(object):
	'''implements the default behaviour of the car.'''

	def __init__(self, base_waypoints, num_look_ahead_waypoints, max_velocity,
		current_car_index, current_velocity):
		rospy.loginfo('Initializing DefaultBehaviour...')

		self.num_look_ahead_waypoints = num_look_ahead_waypoints
		self.base_waypoints = base_waypoints
		self.max_velocity = max_velocity * MAX_VELOCITY_PERCENT
		self.current_velocity = current_velocity
				
		self.current_car_index = current_car_index
		self.state = Starting()

		rospy.loginfo('DefaultBehaviour initialized.')

	def process(self, current_car_index, current_velocity, stop_index):
		'''
		Updates the velocities of the waypoints by considering the current state and traffic_light data.
		Changes the status of the car accordingly.
		Params:
			- `current_car_index` : current index of the car
			- `current_velocity` : current velocity of the car
		'''
		self.current_car_index = current_car_index
		self.current_velocity = current_velocity.twist.linear.x

		waypoints = self.get_next_waypoints()

		self.state.update(car_index=self.current_car_index,
						current_velocity=self.current_velocity,
						max_velocity=self.max_velocity,
						stop_index=stop_index,
						waypoints=waypoints)
		new_state, waypoints = self.state.process()
		if new_state:
			self.state = new_state			
		return waypoints

	def get_next_waypoints(self):
		'''returns the next waypoints.'''
		next_waypoints = []

		all_waypoints = self.base_waypoints
		start_index = self.current_car_index
		end_index = self.current_car_index + self.num_look_ahead_waypoints

		for i in range(start_index, end_index):
			wp = Waypoint()
			index = i % len(all_waypoints)

			wp.pose.pose.position = all_waypoints[index].pose.pose.position
			wp.pose.pose.orientation = all_waypoints[index].pose.pose.orientation
			wp.twist.twist.linear.x = all_waypoints[index].twist.twist.linear.x
		
			next_waypoints.append(wp)

		return next_waypoints
