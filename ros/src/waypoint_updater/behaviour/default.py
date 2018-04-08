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

	def update(self, car_index, current_velocity, final_velocity, distance_stop_line,
		stop_index, traffic_light_in_sight, traffic_light, waypoints):
		'''update the state.'''
		self.current_velocity = current_velocity
		self.car_index = car_index
		self.final_velocity = final_velocity
		self.distance_stop_line = distance_stop_line
		self.stop_index = stop_index
		self.traffic_light_in_sight = traffic_light_in_sight
		self.traffic_light = traffic_light
		self.waypoints = waypoints


class Starting(BaseBehaviour):
	'''This class handles the `Starting` state of the Behaviour.

	If there is no traffic light in sight or if the traffic light is green, 
	switch to `Accelerating`.
	Wait for a green signal otherwise.
	'''

	def process(self):
		if not(self.traffic_light_in_sight) \
			or self.traffic_light.state is TrafficLight.GREEN:
			rospy.loginfo('switching behaviour : Starting to Accelerating')
			return Accelerating(), accelerate(self.waypoints,
				self.current_velocity,
				self.final_velocity)

		return None, stop(self.waypoints, 0, len(self.waypoints)-1)

		

class Accelerating(BaseBehaviour):
	'''This class handles the `Accelerating` state of the Behaviour.

	If there is a yellow or red signal in sight within the halting distance,
	switch to `Decelerating`.
	Keep accelerating otherwise.
	'''
	
	def process(self):
		# handling yellow traffic light
		if self.traffic_light_in_sight and self.traffic_light.state in [TrafficLight.YELLOW, TrafficLight.RED]:
			# if the car can physically stop, stop it; otherwise continue accelerating
			distance_to_halt = self.current_velocity ** 2 / (2 * MAX_DECELERATION)
			if self.distance_stop_line <= distance_to_halt * STOP_DISTANCE_RATIO:
				rospy.loginfo('switching behaviour : Accelerating to Decelerating')
				return Decelerating(), decelerate(waypoints=self.waypoints,
					car_index=self.car_index,
					target_index=self.distance_stop_line)
		
		return None, accelerate(self.waypoints,
				self.current_velocity,
				self.final_velocity)


class Decelerating(BaseBehaviour):
	'''This class handles the `Decelerating` state of the Behaviour.

	If there is no traffic light in sight, switch to `Accelerating`.
	If there is a green traffic light in sight, switch to `Accelerating`.
	Keep decelerating otherwise.
	'''

	def process(self):
		# no traffic light in sight, no reason to decelerate
		if not self.traffic_light_in_sight:
			rospy.loginfo('switching behaviour : Decelerating to Accelerating')
			return Accelerating(), accelerate(self.waypoints,
				self.current_velocity,
				self.final_velocity)

		# the traffic light goes green, go go go
		if self.traffic_light_in_sight and self.traffic_light.state is TrafficLight.GREEN:
			rospy.loginfo('switching behaviour : Decelerating to Accelerating')
			return Accelerating(), accelerate(self.waypoints,
				self.current_velocity,
				self.final_velocity)

		return None, decelerate(waypoints=self.waypoints,
						car_index=self.car_index,
						target_index=self.distance_stop_line)


class DefaultBehaviour(object):
	'''implements the default behaviour of the car.'''

	def __init__(self, base_waypoints, num_look_ahead_waypoints, stop_line_positions, max_velocity,
        current_position, current_velocity, all_traffic_lights):
		rospy.loginfo('Initializing DefaultBehaviour...')

		self.num_look_ahead_waypoints = num_look_ahead_waypoints
		self.base_waypoints = base_waypoints
		self.stop_line_positions = stop_line_positions
		self.max_velocity = max_velocity * MAX_VELOCITY_PERCENT
		self.current_position = current_position
		self.current_velocity = current_velocity
				
		self.current_car_index = None
		self.distance_nearest_stop_line = None
		self.num_waypoints_to_traffic_light = None
		self.nearest_traffic_light_index = None
		self.nearest_stop_line_index = None
		self.nearest_traffic_light = None
		self.time_current_light_state = 0.0
		self.traffic_light_in_sight = True
		self.all_traffic_lights = all_traffic_lights.lights
		self.all_traffic_light_indices = []
		self.stop_line_indices = []

		self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.base_waypoints]
		self.waypoint_tree = KDTree(self.waypoints_2d)

		# initializing the traffic light indices
		self.init_traffic_light_indices(self.all_traffic_lights)
		self.init_stop_line_indices()

		self.state = Starting()

		rospy.loginfo('DefaultBehaviour initialized.')

	def update_traffic_light_(self):
		'''updates the nearest traffic light status.'''
		for i in range(len(self.all_traffic_light_indices)):
			traffic_light_index = self.all_traffic_light_indices[i]

			if traffic_light_index > self.current_car_index:
				self.nearest_traffic_light_index = traffic_light_index
				self.nearest_stop_line_index = self.stop_line_indices[i]
				self.nearest_traffic_light = self.all_traffic_lights[i]
				return


	def update(self, current_position, current_velocity, all_traffic_lights):
		'''
		updates the traffic lights data and current (position,velocity) of the car
		Params:
			- `current_position` : current position of the car
			- `current_velocity` : current velocity of the car
			- `all_traffic_lights` : traffic_lights data published by /vehicle/traffic_lights topic
		'''
		self.current_position = current_position.pose.position
		self.current_velocity = current_velocity.twist.linear.x
		self.all_traffic_lights = all_traffic_lights.lights

		self.current_car_index = self.get_nearest_waypoint_index(self.current_position.x,
																 self.current_position.y)

		# update nearest traffic_light and stop_line data
		self.update_traffic_light_()

		# check if traffic_light is within the next_waypoints list
		self.traffic_light_in_sight = False
		if (self.nearest_traffic_light_index - self.current_car_index <= self.num_look_ahead_waypoints):
			self.traffic_light_in_sight = True
			self.distance_nearest_stop_line = distance(self.base_waypoints,
													   self.current_car_index,
													   self.nearest_stop_line_index)
	
	def process(self):
		'''
		Updates the velocities of the waypoints by considering the current state and traffic_light data.
		Changes the status of the car accordingly.
		'''
		waypoints = self.get_next_waypoints()

		self.state.update(car_index=self.current_car_index,
						current_velocity=self.current_velocity,
						final_velocity=self.max_velocity,
						distance_stop_line=self.distance_nearest_stop_line,
						stop_index=self.nearest_stop_line_index,
						traffic_light=self.nearest_traffic_light,
						traffic_light_in_sight=self.traffic_light_in_sight,
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
	
	def init_traffic_light_indices(self, tl_list):
		'''Initializes the traffic light indices.'''
		for tl in tl_list:
			tl_position = tl.pose.pose.position
			nearest_index = self.get_nearest_waypoint_index(x=tl_position.x, 
															y=tl_position.y)
			self.all_traffic_light_indices.append(nearest_index - 2)

	def init_stop_line_indices(self):
		'''Initializes the stop line indices.'''
		for sl in self.stop_line_positions:
			nearest_index = self.get_nearest_waypoint_index(x=sl[0], y=sl[1])
			self.stop_line_indices.append(nearest_index)

	def get_nearest_waypoint_index(self, x, y):
		'''Returns the nearest(ahead of the current position) waypoint index from the current position.'''
		nearest_index = self.waypoint_tree.query([x, y], 1)[1]

		#check if closest is ahead or behind vehicle
		nearest_coordinate = self.waypoints_2d[nearest_index]
		prev_coordinate = self.waypoints_2d[nearest_index-1]

		# equation for hyperplane through closest coords
		cl_vect = np.array(nearest_coordinate)
		prev_vect = np.array(prev_coordinate)
		pos_vect = np.array([x, y])

		val = np.dot((cl_vect - prev_vect),(pos_vect - cl_vect))

		if val > 0:
			nearest_index = (nearest_index + 1) % len(self.waypoints_2d)

		return nearest_index
