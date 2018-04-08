#!/usr/bin/env python
import yaml

import rospy

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
RATE_HZ = 50


class WaypointUpdater(object):

	def __init__(self):
		rospy.loginfo('Initializing WaypointUpdater...')
		rospy.init_node('waypoint_updater')
		self.default_velocity = rospy.get_param('~velocity', 1)
		
		self.current_velocity = None
		self.current_position = None
		self.all_traffic_lights = None
		self.behaviour = None
		self.base_waypoints = None
		self.stop_line_positions = None

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
		rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.all_traffic_lights_cb)

		config_string = rospy.get_param('/traffic_light_config')
		traffic_light_config = yaml.load(config_string)

		self.stop_line_positions = traffic_light_config['stop_line_positions']
		self.max_velocity = rospy.get_param('/waypoint_loader/velocity') * 0.2778 # m/s
		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane,
												   queue_size=1)

	def current_velocity_cb(self, msg):
		'''callback of the `/current_velocity` topic.'''
		self.current_velocity = msg
	
	def position_cb(self, msg):
		'''callback of the `/current_pose` topic.'''
		self.current_position = msg

	def waypoints_cb(self, msg):
		'''callback of the `/obstacle_waypoint` topic.'''
		self.base_waypoints = msg.waypoints

		# stop listening to /base_waypoints as the base_waypoints are not
		# changing for the project.
		self.base_waypoints_sub.unregister()

	def traffic_cb(self, msg):
		'''callback of the `/traffic_waypoint` topic.'''
		self.traffic_waypoint = msg.data

	def all_traffic_lights_cb(self, msg):
		'''callback of /vehicle/traffic_lights topic. Used only for driving in the simulator.
		   For actual track, use data from traffic_cb
		'''
		self.all_traffic_lights = msg
	
	def obstacle_cb(self, msg):
		'''callback of the `/obstacle_waypoint` topic.'''
		self.obstacle_waypoint = msg.data

	def init_behaviour(self):
		'''initializes the behaviour when we get all required data.'''
		is_ready = self.base_waypoints \
			and self.stop_line_positions \
			and self.max_velocity \
			and self.current_position \
			and self.current_velocity \
			and self.all_traffic_lights
		if not is_ready:
			return
		self.behaviour = DefaultBehaviour(base_waypoints=self.base_waypoints,
										  num_look_ahead_waypoints=LOOKAHEAD_WPS,
										  stop_line_positions = self.stop_line_positions,
										  max_velocity=self.max_velocity,
										  current_position=self.current_position,
										  current_velocity=self.current_velocity,
										  all_traffic_lights=self.all_traffic_lights)

	def loop(self):
		rate = rospy.Rate(RATE_HZ)
		while not rospy.is_shutdown():
			if not self.behaviour:
				self.init_behaviour()
				continue

			self.behaviour.update(current_position=self.current_position,
								  current_velocity=self.current_velocity,
								  all_traffic_lights = self.all_traffic_lights)

			waypoints = self.behaviour.process()
			if waypoints:
				lane = Lane()
				lane.header.frame_id = self.current_position.header.frame_id
				lane.header.stamp = rospy.Time(0)
				lane.waypoints =  waypoints
				self.final_waypoints_pub.publish(lane)

			rate.sleep()

if __name__ == '__main__':
	try:
		WaypointUpdater()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint updater node.')
