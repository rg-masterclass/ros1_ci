#! /usr/bin/env python

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import rospy
import unittest
import rostest
import math
import actionlib
from geometry_msgs.msg import Point, Quaternion
from tortoisebot_waypoints_interface.msg import WaypointActionAction, WaypointActionGoal
import tf.transformations as tf

PKG = 'tortoisebot_waypoints'
NAME = 'waypoints_test'

class TestWaypointsActionServer(unittest.TestCase):

    def setUp(self):

        rospy.init_node('test_waypoints_action_server_node')

        self.action_client = actionlib.SimpleActionClient("tortoisebot_as", WaypointActionAction)
        self.action_client.wait_for_server()

        self.current_position = Point()
        self.initial_position = Point() 
        self.current_orientation = Quaternion()

        self.destination_position = WaypointActionGoal()
        self.action_result = None

        self.current_yaw = 0.0

        self.dist_precision = 0.25
        self.yaw_precision = 5.0

        self.error_position = 0.0
        self.error_yaw = 0.0

        self.odom_listener = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.action_call()

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position

    def quaternion_to_euler(self, msg):
        orientation_list = [msg.x, msg.y, msg.z, msg.w]
        (_, _, yaw) = euler_from_quaternion (orientation_list)
        return yaw

    def action_call(self):
        self.action_client.cancel_goal()

        self.destination_position.position.x = 0.2
        self.destination_position.position.y = 0.2

        self.action_client.send_goal(self.destination_position)
        self.action_client.wait_for_result(rospy.Duration(60))

        self.action_result = self.action_client.get_result()

    def test_robot_end_position(self):
        self.assertTrue(self.action_result)

        x_error = abs(self.destination_position.position.x - self.current_position.x)
        y_error = abs(self.destination_position.position.y - self.current_position.y)

        self.assertTrue(x_error and y_error <= self.dist_precision)

    def test_robot_end_orientation(self):
        self.assertTrue(self.action_result)
        
        yaw = math.atan2(self.destination_position.position.y - self.initial_position.y, self.destination_position.position.x - self.initial_position.x)
        yaw_error = abs(yaw - self.quaternion_to_euler(self.current_orientation))

        self.assertTrue(yaw_error <= self.yaw_precision)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypointsActionServer)
