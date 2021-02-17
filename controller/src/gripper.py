#!/usr/bin/env python
import rospy
import actionlib

import control_msgs.msg

class Gripper:
    def __init__(self):
        self.grip_complete = 0
        self._client = actionlib.SimpleActionClient('gripper_controller/gripper_action', control_msgs.msg.GripperCommandAction)
        rospy.loginfo("Waiting for Gripper controller...")
        self._client.wait_for_server()

    def Close(self):
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = 0.04
        self._client.send_goal(goal)
        self._client.wait_for_result()
        self.result = self._client.get_result()

    def Open(self):
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = 0.1
        self._client.send_goal(goal)
        self._client.wait_for_result()
        self.result = self._client.get_result()

