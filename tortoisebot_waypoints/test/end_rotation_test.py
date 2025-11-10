#!/usr/bin/env python
import rospy
import unittest
import rostest
import actionlib
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tortoisebot_msgs.msg import WaypointActionAction, WaypointActionGoal


class TestEndRotation(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_end_rotation', anonymous=True)
        self.yaw_tolerance = math.radians(25)
        self.pos_tolerance = 0.15
        self.position = None
        self.init_position = None
        self.yaw = None
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.client = actionlib.SimpleActionClient(
            'tortoisebot_as', WaypointActionAction)
        self.assertTrue(self.client.wait_for_server(
            rospy.Duration(20)), "Action server not available")

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = yaw
        if self.init_position is None:
            self.init_position = self.position

    def test_end_rotation(self):
        goal_x = -0.4
        goal_y = 0.4

        goal = WaypointActionGoal()
        goal.position.x = goal_x
        goal.position.y = goal_y

        self.client.send_goal(goal)
        finished = self.client.wait_for_result(rospy.Duration(30))
        self.assertTrue(finished, "Timed out waiting for result")

        result = self.client.get_result()
        self.assertIsNotNone(result, "No result returned from action server")
        self.assertTrue(result.success, "Action did not succeed")

        self.assertTrue(self.client.get_result().success,
                        "Action did not succeed")

        rospy.sleep(1.0)
        self.assertIsNotNone(self.yaw, "No yaw data received")

        dist_error = math.sqrt((self.position.x - goal_x)
                               ** 2 + (self.position.y - goal_y)**2)
        rospy.loginfo(f"Distance error: {dist_error:.3f}")
        self.assertLess(dist_error, self.pos_tolerance,
                        "End position error too large")
        expected_yaw = math.atan2(
            goal_y - self.init_position.y, goal_x - self.init_position.x)
        yaw_error = abs((self.yaw - expected_yaw + math.pi) %
                        (2 * math.pi) - math.pi)
        rospy.loginfo(f"Yaw error: {yaw_error:.3f}")
        self.assertLess(yaw_error, self.yaw_tolerance,
                        "End rotation error too large")


if __name__ == '__main__':
    rostest.rosrun('tortoisebot_waypoints',
                   'end_rotation_test', TestEndRotation)
