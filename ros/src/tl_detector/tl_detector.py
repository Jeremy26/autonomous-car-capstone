#!/usr/bin/env python
import rospy
import numpy as np
from scipy.spatial import KDTree
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3
LOOKAHEAD_WPS = 200
RATE_HZ = 50

DEBUG_MODE=False

class TLDetector(object):
	def __init__(self):
		rospy.init_node('tl_detector')
		if DEBUG_MODE:
			self.image_count = 0

		self.current_position = None
		self.waypoints = None
		self.camera_image = None
		self.all_traffic_lights = []
		self.state = TrafficLight.UNKNOWN
		self.current_car_index = None
		self.base_waypoints = None
		self.waypoints_2d = None
		self.waypoint_tree = None
		self.state = TrafficLight.UNKNOWN		
		self.state_count = 0
		
		self.all_traffic_light_indices = []
		self.stop_line_indices = []
		config_string = rospy.get_param("/traffic_light_config")
		traffic_light_config = yaml.load(config_string)
		self.stop_line_positions = traffic_light_config['stop_line_positions']

		self.ros_setup_()

		# check in which mode(simulator/carla)
		mode = 'SIMULATOR' if rospy.get_param('~traffic_light_simulator_mode') else 'SITE'
		self.light_classifier = TLClassifier(mode=mode)
		self.bridge = CvBridge()
		self.listener = tf.TransformListener()

		self.loop()

	def ros_setup_(self):
		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		self.base_wp_subscriber = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
		# /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
		# helps you acquire an accurate ground truth data source for the traffic light
		# classifier by sending the current color state of all traffic lights in the
		# simulator. When testing on the vehicle, the color state will not be available. You'll need to
		# rely on the position of the light and the camera image to predict it.
		rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
		rospy.Subscriber('/image_color', Image, self.image_cb)

		self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

	def pose_cb(self, msg):
		self.current_position = msg

	def waypoints_cb(self, msg):
		self.base_waypoints = msg.waypoints
		self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.base_waypoints]
		self.waypoint_tree = KDTree(self.waypoints_2d)

		if self.base_waypoints and self.waypoints_2d:
			self.base_wp_subscriber.unregister()

	def traffic_cb(self, msg):
		self.all_traffic_lights = msg.lights

	def image_cb(self, msg):
		self.camera_image = msg

	def loop(self):
		rate = rospy.Rate(RATE_HZ)
		while not rospy.is_shutdown():
			if not self.camera_image:
				continue

			# we want to sleep at each iteration of the loop. We sleep first in 
			# order to prevent multiple iteration without sleeping due to the 
			# "continue" pattern
			rate.sleep()

			stop_line_index, state = self.process_traffic_lights()
			if self.state != state:
				self.state_count = 0
				self.state = state
				continue
			
			if self.state_count < STATE_COUNT_THRESHOLD:
				self.state_count += 1
				continue

			if state in [TrafficLight.GREEN, TrafficLight.UNKNOWN]:
				stop_line_index = -1
			self.upcoming_red_light_pub.publish(Int32(stop_line_index))

	def get_light_state(self):
		"""Determines the current color of the traffic light

		Args:
			light (TrafficLight): light to classify

		Returns:
			int: ID of traffic light color (specified in styx_msgs/TrafficLight)

		"""
		cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
		classification_result = self.light_classifier.get_classification(cv_image)
		if DEBUG_MODE:
			cv2.imwrite('traffic_image_{}_state_{}.jpeg'.format(self.image_count, classification_result), cv_image)
			self.image_count += 1
		return classification_result

	def process_traffic_lights(self):
		"""Finds closest visible traffic light, if one exists, and determines its
			location and color

		Returns:
			int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
			int: ID of traffic light color (specified in styx_msgs/TrafficLight)

		"""
		ready = self.waypoints_2d and \
			self.waypoint_tree and \
			self.current_position and \
			self.current_car_index
		if not ready:
			return -1, TrafficLight.UNKNOWN

		current_x = self.current_position.pose.position.x
		current_y = self.current_position.pose.position.y
		self.current_car_index = self.get_nearest_waypoint_index(current_x, current_y)		

		if len(self.all_traffic_light_indices) == 0 and self.base_waypoints:
			self.update_traffic_light_indices()
			self.update_stop_line_indices()
		
		tl_index, stop_line_index = self.get_nearest_traffic_light()
		if not tl_index or \
			tl_index - self.current_car_index > LOOKAHEAD_WPS:
			rospy.loginfo('No traffic light in sight')
			return -1, TrafficLight.UNKNOWN

		state = self.get_light_state()
		rospy.loginfo('Traffic light in state : {}'.format(state))
		return stop_line_index, state

	def get_nearest_traffic_light(self):
		for i in range(len(self.all_traffic_light_indices)):
			traffic_light_index = self.all_traffic_light_indices[i]
			if traffic_light_index > self.current_car_index:
				rospy.loginfo('Current Car Index : {}'.format(self.current_car_index))
				rospy.loginfo('Nearest Stop line index : {}'.format(self.stop_line_indices[i]))
				return traffic_light_index, self.stop_line_indices[i]
		return None, None

	def update_traffic_light_indices(self):
		all_traffic_lights = self.all_traffic_lights
		if all_traffic_lights:
			for i in range(len(all_traffic_lights)):
				traffic_light = all_traffic_lights[i]
				traffic_light_position = traffic_light.pose.pose.position
				traffic_light_position_x = traffic_light_position.x
				traffic_light_position_y = traffic_light_position.y
				nearest_index = self.get_nearest_waypoint_index(x=traffic_light_position_x, 
																y=traffic_light_position_y)
				self.all_traffic_light_indices.append(nearest_index)
	
	def update_stop_line_indices(self):
		for i in range(len(self.stop_line_positions)):
			stop_line_x = self.stop_line_positions[i][0]
			stop_line_y = self.stop_line_positions[i][1]

			nearest_index = self.get_nearest_waypoint_index(x=stop_line_x, y=stop_line_y)
			self.stop_line_indices.append(nearest_index)
	
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
		TLDetector()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start traffic node.')
