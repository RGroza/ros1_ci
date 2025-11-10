#!/usr/bin/env python
import rospy
import unittest
import rostest
import actionlib
import math
from nav_msgs.msg import Odometry
from tortoisebot_msgs.msg import WaypointActionAction, WaypointActionGoal


class TestEndPosition(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_end_position', anonymous=True)
        self.pos_tolerance = 0.15
        self.position = None
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.client = actionlib.SimpleActionClient(
            'tortoisebot_as', WaypointActionAction)
        self.assertTrue(self.client.wait_for_server(
            rospy.Duration(20)), "Action server not available")

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position

    def test_end_position(self):
        goal_x = -0.4
        goal_y = -0.4

        goal = WaypointActionGoal()
        goal.position.x = goal_x
        goal.position.y = goal_y

        self.client.send_goal(goal)
        finished = self.client.wait_for_result(rospy.Duration(30))
        self.assertTrue(finished, "Timed out waiting for result")

        result = self.client.get_result()
        self.assertIsNotNone(result, "No result returned from action server")
        self.assertTrue(result.success, "Action did not succeed")

        rospy.sleep(1.0)
        self.assertIsNotNone(self.position, "No odom data received")

        dist_error = math.sqrt((self.position.x - goal_x)
                               ** 2 + (self.position.y - goal_y)**2)
        rospy.loginfo(f"Distance error: {dist_error:.3f}")
        self.assertLess(dist_error, self.pos_tolerance,
                        "End position error too large")

        rospy.sleep(2.0)


if __name__ == '__main__':
    rostest.rosrun('tortoisebot_waypoints',
                   'end_position_test', TestEndPosition)
