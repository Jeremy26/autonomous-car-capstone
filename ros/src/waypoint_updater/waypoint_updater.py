#!/usr/bin/env python
import math

import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

from behaviour.default import DefaultBehaviour

# Number of waypoints we will publish. You can change this number
LOOKAHEAD_WPS = 200
ONE_MPH = 0.44704
RATE_HZ = 50


class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.ros_setup_()
        
        self.default_velocity = rospy.get_param('~velocity', 1)
        
        self.current_velocity = None
        self.current_position = None
        self.behaviour = None
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

    def current_velocity_cb(self, msg):
        '''callback of the `/current_velocity` topic.'''
        self.current_velocity = msg
    
    def position_cb(self, msg):
        '''callback of the `/current_pose` topic.'''
        self.current_position = msg

    def waypoints_cb(self, msg):
        '''callback of the `/obstacle_waypoint` topic.'''
        self.behaviour = DefaultBehaviour(base_waypoints=msg.waypoints,
                                          look_ahead_waypoints=LOOKAHEAD_WPS)
        # stop listening to /base_waypoints as the base_waypoints are not
        # changing for the project.
        self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        '''callback of the `/traffic_waypoint` topic.'''
        self.traffic_waypoint = msg.data
        rospy.loginfo("Traffic waypoints %s : ",self.traffic_waypoint)

    def obstacle_cb(self, msg):
        '''callback of the `/obstacle_waypoint` topic.'''
        self.obstacle_waypoint = msg.data

    def loop(self):
        rate = rospy.Rate(RATE_HZ) # 50Hz
        while not rospy.is_shutdown():
            if not self.behaviour:
                continue

            self.behaviour.update(current_position=self.current_position,
                                  current_velocity=self.current_velocity)
            
            waypoints = self.behaviour.process()
            if waypoints:
                lane = Lane()
                lane.header = self.current_position.header
                lane.waypoints =  waypoints
                self.final_waypoints_pub.publish(lane)

            rate.sleep()

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')