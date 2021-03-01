#!/usr/bin/env python 

import rospy
import time
import math
import cv2
import tf

from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from fetch import Fetch

if ((__name__ == "__main__") and (not rospy.is_shutdown())):
    rospy.init_node("fetch_builder",anonymous=True)

    Fetch_Robot = Fetch()
    rospy.loginfo("Initialization")

    Fetch_Robot.Head.look_at(0.7,0,0.5,"base_link")
    rospy.loginfo("Till head")

    Fetch_Robot.Gripper.Open()
    rospy.loginfo("Gripper Open")

    Fetch_Robot.Arm.Tuck()
    rospy.loginfo("Tuck Arm")

    rospy.sleep(rospy.Duration(2))
    #Take images
    RGB_image = Fetch_Robot.GetRGBImage()
    rospy.loginfo("Get RGB image")

    Depth_image = Fetch_Robot.GetDepthImage()
    rospy.loginfo("Get Depth image")

    #Take point cloud
    ArPose = Fetch_Robot.GetArPoses()
    rospy.sleep(rospy.Duration(2))
    ArPose = Fetch_Robot.GetArPoses()

    while (ArPose.header.frame_id is None):
        ArPose = Fetch_Robot.GetArPoses()
        print("aaaaa")
    
    rospy.loginfo("Get ArPoses")
    print(ArPose)

    #Calculate Poses 
    Fetch_Robot.PoseProcessing.SetRGBImage(RGB_image)
    Fetch_Robot.PoseProcessing.SetDepthImage(Depth_image)
    Fetch_Robot.PoseProcessing.SetArPose(ArPose)

    Fetch_Robot.PoseProcessing.DetermineBlocks("green")
    Green_pose = Fetch_Robot.PoseProcessing.GetBlockPose()

    Fetch_Robot.PoseProcessing.DeterminePlace()
    Place_pose = Fetch_Robot.PoseProcessing.GetPlacePose()

    print("Green_block: ")
    print(Green_pose.pose)

    print("Place pose: ")
    print(Place_pose.pose)

    Green_Yaw = tf.transformations.euler_from_quaternion([Green_pose.pose.orientation.x, Green_pose.pose.orientation.y, Green_pose.pose.orientation.z, Green_pose.pose.orientation.w])[2]
    print("Green yaw: ")
    print(Green_Yaw)

    Place_Yaw = tf.transformations.euler_from_quaternion([Place_pose.pose.orientation.x, Place_pose.pose.orientation.y, Place_pose.pose.orientation.z, Place_pose.pose.orientation.w])[2]
    print("Place yaw: ")
    print(Place_Yaw)

    # Execute the path
    Fetch_Robot.Arm.AddDice("Dice",Green_pose.pose.position.x +0.04,Green_pose.pose.position.y +0.055, Green_pose.pose.position.z + 0.1)

    Fetch_Robot.Arm.MoveToPose(Green_pose.pose.position.x +0.04, Green_pose.pose.position.y +0.055, Green_pose.pose.position.z +0.3, 0, math.pi/4, Green_Yaw + math.pi/2)
    rospy.loginfo("Move arm")

    Fetch_Robot.Arm.RemoveDice("Dice")

    Fetch_Robot.Arm.MoveToPose(Green_pose.pose.position.x +0.04, Green_pose.pose.position.y +0.055, Green_pose.pose.position.z +0.1, 0, math.pi/4, Green_Yaw + math.pi/2)
    rospy.loginfo("Move arm")

    Fetch_Robot.Gripper.Close()
    rospy.loginfo("Gripper Close")

    Fetch_Robot.Arm.MoveToPose(Green_pose.pose.position.x +0.04, Green_pose.pose.position.y +0.055, Green_pose.pose.position.z +0.3, 0, math.pi/4, Green_Yaw + math.pi/2)
    rospy.loginfo("Move arm")

    Fetch_Robot.Arm.MoveToPose(Place_pose.pose.position.x+0.01, Place_pose.pose.position.y +0.055, Place_pose.pose.position.z +0.3, 0, math.pi/4, Place_Yaw)
    rospy.loginfo("Move arm")

    Fetch_Robot.Arm.MoveToPose(Place_pose.pose.position.x+0.01, Place_pose.pose.position.y +0.055, Place_pose.pose.position.z +0.2, 0, math.pi/4, Place_Yaw)
    rospy.loginfo("Move arm")
    Fetch_Robot.Gripper.Open()
    rospy.loginfo("Gripper Open")

    Fetch_Robot.Arm.MoveToPose(Place_pose.pose.position.x+0.01, Place_pose.pose.position.y +0.055, Place_pose.pose.position.z +0.3, 0, math.pi/4, Place_Yaw)
    rospy.loginfo("Move arm")

    Fetch_Robot.Arm.AddDice("Dice1",Place_pose.pose.position.x+0.01, Place_pose.pose.position.y +0.055, Place_pose.pose.position.z +0.18)

    Fetch_Robot.Arm.Tuck()
    rospy.loginfo("Tuck arm")
    Fetch_Robot.Gripper.Open()

    Fetch_Robot.Arm.RemoveDice("Dice1")

        
        