from styx_msgs.msg import TrafficLight
# Import packages
import os
import numpy as np
import tensorflow as tf
from io import StringIO
import cv2
import sys
sys.path.append("..")
# Import utilites
from utils import label_map_util
from utils import visualization_utils as vis_util

class TLClassifier(object):
	def __init__(self):
		#load classifier		
		# Name of the directory containing the object detection module we're using
		MODEL_NAME = 'inference_graph'
		# Grab path to current working directory
		CWD_PATH = os.getcwd()
		# Path to frozen detection graph .pb file, which contains the model that is used
		# for object detection.
		PATH_TO_CKPT = os.path.join(CWD_PATH,'light_classification/frozen_inference_graph.pb')
		# Path to label map file
		PATH_TO_LABELS = os.path.join(CWD_PATH,'light_classification/training','Traffic_light_label_map.pbtxt')
		# Number of classes the object detector can identify
		NUM_CLASSES = 4
		self.PATH_TO_CKPT = PATH_TO_CKPT
		## Load the label map.
		# Label maps map indices to category names, so that when our convolution
		# network predicts `5`, we know that this corresponds to `king`.
		# Here we use internal utility functions, but anything that returns a
		# dictionary mapping integers to appropriate string labels would be fine
		label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
		categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
		category_index = label_map_util.create_category_index(categories)
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
		(boxes, scores, classes, num) = sess.run([self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections], feed_dict={self.image_tensor: frame_expanded})
		boxes = np.squeeze(boxes)
		scores = np.squeeze(scores)
		classes = np.squeeze(classes)
		rospy.loginfo("%s",classes)
		rospy.loginfo("%s",scores)
		rospy.loginfo("%s",boxes)
		return TrafficLight.UNKNOWN