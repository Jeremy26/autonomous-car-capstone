#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint

import math
from copy import deepcopy
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


class WaypointUpdater(object):
    def __init__(self):

        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.waypoints_cb)
        # TODO : uncomment when changing velocities for waypoints
        # rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    '''callback method for saving current_pose message. (Complete messages is stored, not just the pose value!!)'''
    def pose_cb(self, msg):
        self.current_pose = msg
        self.publish_final_waypoints()

    # callback method for saving base waypoints
    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints

        # stop listening to /base_waypoints as the base_waypoints are not changing for the project
        self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        self.traffic_waypoint = msg.data

    def obstacle_cb(self, msg):
        self.obstacle_waypoint = msg.data

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    # calculate euclidean distance between index wp1 and wp2
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    # Returns the euclidian distance between 2 points (P11 - Path Planning)
    def dist(self, p1, p2):
        return math.sqrt((p2.x-p1.x)**2 + (p2.y-p1.y)**2 + (p2.z-p1.z)**2)

    # Get nearest waypoint index from the current pose
    def get_nearest_waypoint_index(self, pose):
        min_distance_to_next_point = 10000000

        p1 = pose.position
        all_waypoints = self.base_waypoints.waypoints

        for i in range(len(all_waypoints)):
            p2 = all_waypoints[i].pose.pose.position
            distance_to_p2 = self.dist(p1, p2)

            if distance_to_p2 < min_distance_to_next_point:
                min_distance_to_next_point = distance_to_p2
                index_to_return = i

        return index_to_return

    '''Get next set of waypoints to be pusblished as final_waypoints'''
    # TODO : include traffic lights and obstacle detection
    def get_final_waypoints(self, next_waypoint_index):
        final_waypoints = []
        len_all_basepoints = len(self.base_waypoints.waypoints)

        for i in range(LOOKAHEAD_WPS):
            wp = Waypoint()

            # populate waypoint values
            index = (next_waypoint_index+i) % len_all_basepoints
            base_waypoint = self.base_waypoints.waypoints[index]
            wp = deepcopy(base_waypoint)
            final_waypoints.append(wp)
        return final_waypoints

    # publish final waypoints
    def publish_final_waypoints(self):
        current_pose = self.current_pose.pose
        nearest_waypoint_index = self.get_nearest_waypoint_index(current_pose)
        final_waypoints = self.get_final_waypoints(nearest_waypoint_index)

        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = final_waypoints
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
