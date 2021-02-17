#!/usr/bin/env python 

import rospy
import time
import math
import cv2

from sensor_msgs.msg import Image

from fetch import Fetch

if __name__ == "__main__":
    rospy.init_node("fetch_builder",anonymous=True)

    Fetch_Robot = Fetch()
    rospy.loginfo("Initialization")

    Fetch_Robot.Head.look_at(0.7,0,0.5,"base_link")
    rospy.loginfo("Till head")

    Fetch_Robot.Arm.Tuck()
    rospy.loginfo("Tuck Arm")

    # Fetch_Robot.Gripper.Close()
    # rospy.loginfo("Gripper Close")

    # Fetch_Robot.Gripper.Open()
    # rospy.loginfo("Gripper Open")

    # Take images
    RGB_image = Fetch_Robot.GetRGBImage()
    cv2.imwrite('/home/augustine/fetch_ws/src/controller/Image/RGB_image.jpg',RGB_image)
    rospy.loginfo("Get RGB image")

    Depth_image = Fetch_Robot.GetDepthImage()
    cv2.imwrite('/home/augustine/fetch_ws/src/controller/Image/Depth_image.jpg',Depth_image)
    rospy.loginfo("Get Depth image")

    ArPose = Fetch_Robot.GetArPoses()
    rospy.loginfo("Get ArPoses")

    # Calculate Poses
    Fetch_Robot.PoseProcessing.SetRGBImage(RGB_image)
    Fetch_Robot.PoseProcessing.SetDepthImage(Depth_image)
    Fetch_Robot.PoseProcessing.SetArPose(ArPose)
    # Fetch_Robot.PoseProcessing.DetermineBlocks("red")
    # Red_position = Fetch_Robot.PoseProcessing.GetBlockPosition()
    # Red_orientation = Fetch_Robot.PoseProcessing.GetBlockOrientation()
    # Red_pose = Fetch_Robot.PoseProcessing.GetBlockPose()

    Fetch_Robot.PoseProcessing.DetermineBlocks("green")
    Green_position = Fetch_Robot.PoseProcessing.GetBlockPosition()
    Green_orientation = Fetch_Robot.PoseProcessing.GetBlockOrientation()
    Green_pose = Fetch_Robot.PoseProcessing.GetBlockPose()
    Place_pose = Fetch_Robot.PoseProcessing.GetPlacePose()

    # Fetch_Robot.PoseProcessing.DetermineBlocks("blue")
    # Blue_position = Fetch_Robot.PoseProcessing.GetBlockPosition()
    # Blue_orientation = Fetch_Robot.PoseProcessing.GetBlockOrientation()
    # Blue_pose = Fetch_Robot.PoseProcessing.GetBlockPose()

    # print(Red_pose)
    # print(Red_orientation)

    print(Green_pose)
    print(Green_orientation)

    print(Place_pose.position)

    # print(Blue_pose)
    # print(Blue_orientation)


    # Fetch_Robot.Arm.MoveToPose(Green_pose.pose.position.x+0.03,Green_pose.pose.position.y+0.05, Green_pose.pose.position.z+0.30, 0, math.pi/4, Green_orientation)
    # rospy.loginfo("Move arm")
    # Fetch_Robot.Arm.MoveToPose(Green_pose.pose.position.x+0.03,Green_pose.pose.position.y+0.05, Green_pose.pose.position.z+0.12, 0, math.pi/4, Green_orientation)
    # rospy.loginfo("Move arm")
    # Fetch_Robot.Gripper.Close()
    # rospy.loginfo("Gripper Close")
    # Fetch_Robot.Arm.MoveToPose(Green_pose.pose.position.x+0.03,Green_pose.pose.position.y+0.05, Green_pose.pose.position.z+0.3, 0, math.pi/4, Green_orientation)
    # rospy.loginfo("Move arm")

    # Fetch_Robot.Arm.MoveToPose(Red_pose.pose.position.x,Red_pose.pose.position.y, Red_pose.pose.position.z+0.2, 0, math.pi/8, Red_orientation)
    # rospy.loginfo("Move arm")
    # Fetch_Robot.Arm.MoveToPose(Red_pose.pose.position.x,Red_pose.pose.position.y, Red_pose.pose.position.z+0.09, 0, math.pi/8, Red_orientation)
    # rospy.loginfo("Move arm")
    # Fetch_Robot.Gripper.Open()
    # rospy.loginfo("Gripper Open")
    # Fetch_Robot.Arm.MoveToPose(Red_pose.pose.position.x,Red_pose.pose.position.y, Red_pose.pose.position.z+0.2, 0, math.pi/8, Green_orientation)
    # rospy.loginfo("Move arm")

    # Fetch_Robot.Arm.MoveToPose(Blue_pose.pose.position.x+0.04,Blue_pose.pose.position.y+0.05, Blue_pose.pose.position.z+0.3, 0, math.pi/4, Blue_orientation)
    # rospy.loginfo("Move arm")
    # Fetch_Robot.Arm.MoveToPose(Blue_pose.pose.position.x,Blue_pose.pose.position.y, Blue_pose.pose.position.z+0.01, 0, math.pi/8, Blue_orientation)
    # rospy.loginfo("Move arm")
    # Fetch_Robot.Gripper.Close()
    # rospy.loginfo("Gripper Close")
    # Fetch_Robot.Arm.MoveToPose(Blue_pose.pose.position.x,Blue_pose.pose.position.y, Blue_pose.pose.position.z+0.1, 0, math.pi/8, Green_orientation)
    # rospy.loginfo("Move arm")

    # Fetch_Robot.Arm.MoveToPose(Red_pose.pose.position.x,Red_pose.pose.position.y, Red_pose.pose.position.z+0.3, 0, math.pi/8, Red_orientation)
    # rospy.loginfo("Move arm")
    # Fetch_Robot.Arm.MoveToPose(Red_pose.pose.position.x,Red_pose.pose.position.y, Red_pose.pose.position.z+0.15, 0, math.pi/8, Red_orientation)
    # rospy.loginfo("Move arm")
    # Fetch_Robot.Gripper.Open()
    # rospy.loginfo("Gripper Open")
    # Fetch_Robot.Arm.MoveToPose(Red_pose.pose.position.x,Red_pose.pose.position.y, Red_pose.pose.position.z+0.3, 0, math.pi/8, Red_orientation)
    # rospy.loginfo("Move arm")

    #Fetch_Robot.Arm.Tuck()
    
    # rospy.sleep(rospy.Duration(2))
    # Fetch_Robot.Gripper.Close()

    # Fetch_Robot.Arm.MoveToPose(0.5, 0.5, 0.7, 0, math.pi/4, 0)
    # rospy.loginfo("Move arm")
    # rospy.sleep(rospy.Duration(2))
    # Fetch_Robot.Gripper.Open()
    
    # Fetch_Robot.Arm.Tuck()
    # rospy.loginfo("Tuck arm")
    # Fetch_Robot.Gripper.Open()

    #while not rospy.is_shutdown():
        # Fetch_Robot.Arm.MoveToPose(0.042, 0.384, 1.826, 0.173, -0.693, -0.242, 0.657)
        # Fetch_Robot.Arm.MoveToPose(0.047, 0.545, 1.822, -0.274, -0.701, 0.173, 0.635)
        
        