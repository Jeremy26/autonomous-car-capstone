import rospy
import rospkg
import numpy as np
import os
import sys
import tensorflow as tf
from collections import defaultdict
from utils import label_map_util
from utils import visualization_utils as vis_util
import time

from styx_msgs.msg import TrafficLight

SIM_MODEL_PATH = 'light_classification/model_files/frozen_inference_graph_SIMULATOR.pb'
SITE_MODEL_PATH = 'light_classification/model_files/frozen_inference_graph_SITE.pb'
LABELS_PATH = 'light_classification/model_files/labels.pbtxt'
NUM_CLASSES = 4

class TLClassifier(object):
	def __init__(self, mode='SIMULATOR'):
		self.current_light = TrafficLight.UNKNOWN

		CWD_PATH = os.getcwd()

		model = os.path.join(CWD_PATH, SIM_MODEL_PATH)
		if mode is 'SITE':
			model = os.path.join(CWD_PATH, SITE_MODEL_PATH)

		labels_path = os.path.join(CWD_PATH, LABELS_PATH)
		label_map = label_map_util.load_labelmap(labels_path)
		categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
																	use_display_name=True)

		self.category_index = label_map_util.create_category_index(categories)

		self.image_np_deep = None
		self.detection_graph = tf.Graph()

		config = tf.ConfigProto()
		config.gpu_options.allow_growth = True
		jit_level = tf.OptimizerOptions.ON_1
		config.graph_options.optimizer_options.global_jit_level = jit_level

		with self.detection_graph.as_default():
			od_graph_def = tf.GraphDef()

			with tf.gfile.GFile(model, 'rb') as fid:
				serialized_graph = fid.read()
				od_graph_def.ParseFromString(serialized_graph)
				tf.import_graph_def(od_graph_def, name='')
			self.sess = tf.Session(graph=self.detection_graph, config=config)
		
		# Definite input and output Tensors for detection_graph
		self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

		# Each box represents a part of the image where a particular object was detected.
		self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

		# Each score represent how level of confidence for each of the objects.
		# Score is shown on the result image, together with the class label.
		self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
		self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
		self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

		print("Loaded frozen model graph for mode = {}".format(mode))

	def get_classification(self, image):
		"""Determines the color of the traffic light in the image

		Args:
			image (cv::Mat): image containing the traffic light

		Returns:
			int: ID of traffic light color (specified in styx_msgs/TrafficLight)

		"""
		self.current_light = TrafficLight.UNKNOWN

		image_expanded = np.expand_dims(image, axis=0)

		time1 = time.time()
		with self.detection_graph.as_default():
			(boxes, scores, classes, num) = self.sess.run([self.detection_boxes,
														   self.detection_scores,
														   self.detection_classes,
														   self.num_detections],
														   feed_dict={self.image_tensor:image_expanded})
		
		time2 = time.time()

		
		boxes = np.squeeze(boxes)
		scores = np.squeeze(scores)
		classes = np.squeeze(classes).astype(np.int32)

		min_score_threshold = 0.5

		for i in range(boxes.shape[0]):
			if scores is None or scores[i] > min_score_threshold:
				class_name = self.category_index[classes[i]]['name']
				rospy.loginfo('Light Color : {}'.format(class_name))
				rospy.loginfo('Time for inference : {}ms'.format((time2-time1)*1000))
				if class_name == 'Red':
					self.current_light = TrafficLight.RED

				elif class_name == 'Yellow':
					self.current_light = TrafficLight.YELLOW

				elif class_name == 'Green':
					self.current_light = TrafficLight.GREEN
				else:
					self.current_light = TrafficLight.UNKNOWN

			#self.image_np_deep = image

		return self.current_light
