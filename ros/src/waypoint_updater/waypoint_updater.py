#!/usr/bin/env python
import math

import rospy

from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

# Number of waypoints we will publish. You can change this number
LOOKAHEAD_WPS = 200

ONE_MPH = 0.44704


class WaypointUpdater(object):

    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.ros_setup_()
        
        self.default_velocity = rospy.get_param('~velocity', 1)
        
        self.current_velocity = None
        rospy.spin()

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
        self.publish_final_waypoints(msg)

    def waypoints_cb(self, msg):
        '''callback of the `/obstacle_waypoint` topic.'''
        self.base_waypoints = msg.waypoints

        # stop listening to /base_waypoints as the base_waypoints are not changing for the project
        self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        '''callback of the `/traffic_waypoint` topic.'''
        self.traffic_waypoint = msg.data

    def obstacle_cb(self, msg):
        '''callback of the `/obstacle_waypoint` topic.'''
        self.obstacle_waypoint = msg.data

    def get_nearest_waypoint_index(self, car):
        '''Returns the nearest waypoint index from the current pose.'''
        min_distance = 10000000

        for i in range(len(self.base_waypoints)):
            wp = self.base_waypoints[i].pose.pose.position
            dist = math.sqrt((wp.x - car.x) ** 2 + (wp.y - car.y) ** 2 + (wp.z - car.z) ** 2)

            if dist < min_distance:
                min_distance = dist
                index_to_return = i
        return index_to_return

    def get_final_waypoints(self, car_position):
        '''Returns the next set of waypoints to be published as final_waypoints.'''
        nearest_waypoint_index = self.get_nearest_waypoint_index(car_position)
        
        waypoints = self.base_waypoints[nearest_waypoint_index:]
        wp_len = len(waypoints)
        if wp_len > LOOKAHEAD_WPS:
            return waypoints[:LOOKAHEAD_WPS]

        return waypoints + self.base_waypoints[:LOOKAHEAD_WPS - wp_len]

    def publish_final_waypoints(self, current_position):
        lane = Lane()
        lane.header = current_position.header
        lane.waypoints = self.get_final_waypoints(current_position.pose.position)
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
