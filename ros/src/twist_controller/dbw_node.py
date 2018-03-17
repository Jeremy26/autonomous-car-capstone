#!/usr/bin/env python

import math

import rospy

from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped

from twist_controller import Controller
from params import PARAMETERS

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):

    def __init__(self):
        rospy.init_node('dbw_node')

        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.target_linear_velocity = None
        self.target_angular_velocity = None
        self.current_velocity = None
        self.dbw_enabled = False

        self.ros_setup_()
        self.controller = Controller(self.wheel_base,
                                     self.steer_ratio,
                                     self.max_lat_accel,
                                     self.max_steer_angle)
        
        self.loop()

    def ros_setup_(self):
        '''Setup the ROS interface.
            Publisher:
                - `/vehicle/steering_cmd` sends steering command.
                - `/vehicle/throttle_cmd` sends throttle command.
                - `/vehicle/brake_cmd` sends brake command.
            Subscriber:
                - `/vehicle/dbw_enabled` captures whether the dbw is enabled.
                - `/current_velocity` captures the current velocity.
                - `/twist_cmd` captures the commands from twist.
        '''
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_callback)
        rospy.Subscriber('/current_velocity', TwistStamped, 
                         self.current_velocity_callback)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_callback)

    def dbw_enabled_callback(self, msg):
        '''callback of the `/vehicle/dbw_enabled` topic.'''
        self.dbw_enabled = msg.data

    def current_velocity_callback(self, msg):
        '''callback of the `/vehicle/current_velocity` topic.'''
        self.current_velocity = msg.twist.linear

    def twist_cmd_callback(self, msg):
        '''callback of the `/twist_cmd` topic.'''
        self.target_linear_velocity = msg.twist.linear
        self.target_angular_velocity = msg.twist.angular

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            throttle, brake, steering = \
                self.controller.control(self.target_linear_velocity,
                                        self.target_angular_velocity,
                                        self.current_velocity,
                                        self.dbw_enabled)
            if self.dbw_enabled:
                self.publish(throttle, brake, steering)
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
