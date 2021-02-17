#!/usr/bin/env python
import rospy
import actionlib

from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped

class Head:

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame):
        #The goal message will be sending
        goal = PointHeadGoal()
        point = PointStamped()

        #The target point, expressed in the requested frame
        point.header.frame_id = frame
        point.point.x = x
        point.point.y = y
        point.point.z = z

        goal.target = point

        #We are pointing the high-def camera frame to default X-axis
        goal.pointing_frame = "base_link"
        goal.pointing_axis.x = 1
        goal.pointing_axis.y = 0
        goal.pointing_axis.z = 0

        #Take at least 0.5 seconds to get there
        goal.min_duration = rospy.Duration(0.5)
        #Go no faster than 1 rad/s
        goal.max_velocity = 1.0

        #send the goal
        self.client.send_goal_and_wait(goal, rospy.Duration(3))
        #self.client.wait_for_result()