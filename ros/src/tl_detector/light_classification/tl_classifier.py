from styx_msgs.msg import TrafficLight
# Import packages
import os
import numpy as np
import tensorflow as tf
from io import StringIO
import cv2
import sys
import rospy
sys.path.append("..")
# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util

class TLClassifier(object):
	def __init__(self, simulator):
		#load classifier
		self.current_light = TrafficLight.UNKNOWN

		# Grab path to current working directory
		CWD_PATH = os.getcwd()
		self.PATH_TO_CKPT = None
		self.simulator = simulator
		rospy.loginfo("%s",self.simulator)

		## Load the correct model
		if (self.simulator == True):
			rospy.loginfo("SIMULATOR MODE")
			self.PATH_TO_CKPT = os.path.join(CWD_PATH,'light_classification/simulator','frozen_inference_graph_SIMULATOR.pb')
		elif (self.simulator == False):
			rospy.loginfo("SITE MODE")
			self.PATH_TO_CKPT = os.path.join(CWD_PATH,'light_classification/real','frozen_inference_graph_SITE.pb')

		# Path to label map file
		PATH_TO_LABELS = os.path.join(CWD_PATH,'light_classification/training','Traffic_light_label_map.pbtxt')
		# Number of classes the object detector can identify
		NUM_CLASSES = 4
		
		## Load the label map.
		# Label maps map indices to category names
		# Here we use internal utility functions, but anything that returns a
		# dictionary mapping integers to appropriate string labels would be fine
		label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
		categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
		self.category_index = label_map_util.create_category_index(categories)
		self.sess, self.detection_graph = self.load_graph()	
		# Define input and output tensors (i.e. data) for the object detection classifier
		# Input tensor is the image
		self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
		# Output tensors are the detection boxes, scores, and classes
		# Each box represents a part of the image where a particular object was detected
		self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
		# Each score represents level of confidence for each of the objects.
		# The score is shown on the result image, together with the class label.
		self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
		self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
		# Number of objects detected
		self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
	
	def load_graph(self):
		detection_graph = tf.Graph() # Load the Tensorflow model into memory.
		with detection_graph.as_default():
			od_graph_def = tf.GraphDef()
			with tf.gfile.GFile(self.PATH_TO_CKPT, 'rb') as fid:
				serialized_graph = fid.read()
				od_graph_def.ParseFromString(serialized_graph)
				tf.import_graph_def(od_graph_def, name='')
			sess = tf.Session(graph=detection_graph)
			return sess, detection_graph

	def get_classification(self, image):
		"""Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

		"""
		#TODO implement light color prediction
		frame_expanded = np.expand_dims(image, axis=0)
		# Perform the actual detection by running the model with the image as input
		(boxes, scores, classes, num) = self.sess.run([self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections], feed_dict={self.image_tensor: frame_expanded})
		boxes = np.squeeze(boxes)
		scores = np.squeeze(scores)
		classes = np.squeeze(classes)
		min_score_thresh = .5
		for i in range(boxes.shape[0]):
			if scores is None or scores[i]>min_score_thresh :
				class_name = self.category_index[classes[i]]['name']
				self.current_light = TrafficLight.UNKNOWN
				if class_name == 'red':
					self.current_light = TrafficLight.RED
				elif class_name == 'yellow' :
					self.current_light = TrafficLight.YELLOW
				elif class_name == 'green':
					self.current_light = TrafficLight.GREEN 
		
				rospy.loginfo("Light Detected : %s",self.current_light)
		return self.current_light
