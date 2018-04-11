#!/usr/bin/env python
import yaml

import rospy
from scipy.spatial import KDTree
import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int32
from styx_msgs.msg import (
	Lane,
	TrafficLightArray,
	TrafficLight,
	Waypoint
)

from behaviour.default import DefaultBehaviour


LOOKAHEAD_WPS = 200
RATE_HZ = 10


class WaypointUpdater(object):

	def __init__(self):
		rospy.loginfo('Initializing WaypointUpdater...')
		rospy.init_node('waypoint_updater')
		self.default_velocity = rospy.get_param('~velocity', 1)
		
		self.current_velocity = None
		self.current_position = None
		self.behaviour = None
		self.base_waypoints = None
		self.current_car_index = None
		self.traffic_waypoint = -1
		self.waypoints_2d = None
		self.waypoint_tree = None

		self.ros_setup_()
		self.loop()

	def ros_setup_(self):
		self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane,
												   self.waypoints_cb)
		rospy.Subscriber('/current_pose', PoseStamped, self.position_cb)

		rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
		rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
		rospy.Subscriber('/current_velocity', TwistStamped,
						 self.current_velocity_cb)
		
		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane,
												   queue_size=1)

		self.max_velocity = rospy.get_param('/waypoint_loader/velocity') * 0.2778 # m/s

	def current_velocity_cb(self, msg):
		'''callback of the `/current_velocity` topic.'''
		self.current_velocity = msg
	
	def position_cb(self, msg):
		'''callback of the `/current_pose` topic.'''
		self.current_position = msg
		current_x = self.current_position.pose.position.x
		current_y = self.current_position.pose.position.y
		if self.waypoints_2d:
			self.current_car_index = self.get_nearest_waypoint_index(current_x,
																	 current_y)

	def waypoints_cb(self, msg):
		'''callback of the `/obstacle_waypoint` topic.'''
		self.base_waypoints = msg.waypoints
		self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.base_waypoints]
		self.waypoint_tree = KDTree(self.waypoints_2d)

		# stop listening to /base_waypoints as the base_waypoints are not
		# changing for the project.
		if self.base_waypoints and self.waypoints_2d:
			self.base_waypoints_sub.unregister()

	def traffic_cb(self, msg):
		'''callback of the `/traffic_waypoint` topic.'''
		self.traffic_waypoint = msg.data

	def obstacle_cb(self, msg):
		'''callback of the `/obstacle_waypoint` topic.'''
		self.obstacle_waypoint = msg.data

	def init_behaviour(self):
		'''initializes the behaviour when we get all required data.'''
		ready = self.base_waypoints \
			and self.max_velocity \
			and self.current_position \
			and self.current_velocity \
			and self.current_car_index
		if not ready:
			return
		self.behaviour = DefaultBehaviour(base_waypoints=self.base_waypoints,
										  num_look_ahead_waypoints=LOOKAHEAD_WPS,
										  max_velocity=self.max_velocity,
										  current_velocity=self.current_velocity,
										  current_car_index=self.current_car_index)

	def loop(self):
		rate = rospy.Rate(RATE_HZ)
		while not rospy.is_shutdown():
			if not self.behaviour:
				self.init_behaviour()
				continue

			waypoints = self.behaviour.process(
				current_car_index=self.current_car_index,
				current_velocity=self.current_velocity,
				stop_index=self.traffic_waypoint)

			if waypoints:
				lane = Lane()
				lane.header.frame_id = self.current_position.header.frame_id
				lane.header.stamp = rospy.Time(0)
				lane.waypoints =  waypoints
				self.final_waypoints_pub.publish(lane)
			rate.sleep()
	
	def get_nearest_waypoint_index(self, x, y):
		'''Returns the nearest(ahead of the current position) waypoint index from the current pose.'''
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

if __name__ == '__main__':
	try:
		WaypointUpdater()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint updater node.')
