#!/usr/bin/env python
import rospy
import tf

#Import the interface in Moveit package
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Arm:
    def __init__(self):
        self.moveGroup = MoveGroupInterface("arm_with_torso", "base_link")
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -0.6)
        self.planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -0.6)
        self.planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -0.6)
        self.planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -0.6)
        # self.planning_scene.removeCollisionObject("my_front_ground")
        # self.planning_scene.removeCollisionObject("my_back_ground")
        # self.planning_scene.removeCollisionObject("my_right_ground")
        # self.planning_scene.removeCollisionObject("my_left_ground")
        

    def MoveToPose(self,X,Y,Z,Roll,Pitch,Yaw):
        orientation = tf.transformations.quaternion_from_euler(Roll,Pitch,Yaw)

        self.poseStamped = PoseStamped()
        self.poseStamped.header.frame_id = 'base_link'
        self.poseStamped.header.stamp = rospy.Time.now()

        self.poseStamped.pose.position.x = X
        self.poseStamped.pose.position.y = Y 
        self.poseStamped.pose.position.z = Z
        
        self.poseStamped.pose.orientation.x = orientation[0]
        self.poseStamped.pose.orientation.y = orientation[1]
        self.poseStamped.pose.orientation.z = orientation[2]
        self.poseStamped.pose.orientation.w = orientation[3]

        self.moveGroup.moveToPose(self.poseStamped, 'gripper_link', max_velocity_scaling_factor = 0.3)
        self.result = self.moveGroup.get_move_action().get_result()

    def Tuck(self):
        joints = ["torso_lift_joint","shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.0, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]    

        self.moveGroup.moveToJointPosition(joints, pose, 0.02, max_velocity_scaling_factor = 0.3)
        while not rospy.is_shutdown():  
            self.result = self.moveGroup.get_move_action().get_result()
            if self.result.error_code.val == MoveItErrorCodes.SUCCESS:
                break

    def Stow(self):
        joints = ["torso_lift_joint","shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.0, 1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]

        self.moveGroup.moveToJointPosition(joints, pose, 0.02, max_velocity_scaling_factor = 0.3)
        while not rospy.is_shutdown():
            self.result = self.moveGroup.get_move_action().get_result()
            if self.result.error_code.val == MoveItErrorCodes.SUCCESS:
                break
