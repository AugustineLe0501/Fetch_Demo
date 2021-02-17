#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ar_track_alvar_msgs.msg import AlvarMarkers

from arm import Arm
from head import Head
from gripper import Gripper
from poseprocessing import PoseProcessing

class Fetch:
    def __init__ (self):
        rospy.Subscriber("/head_camera/rgb/image_raw",Image,self.RGBImageCallback)
        rospy.Subscriber("/head_camera/depth_registered/image_raw",Image,self.DepthImageCallback)
        rospy.Subscriber("/ar_pose_marker",AlvarMarkers,self.GetArPosesCallBack)
        self.bridge = CvBridge()

        self.Arm = Arm()
        self.Gripper = Gripper()
        self.Head = Head()
        self.PoseProcessing = PoseProcessing()

    def RGBImageCallback(self,data):
        try:
            self.rgbImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def DepthImageCallback(self,data):
        try:
            self.depthImage = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

    def GetArPosesCallBack(self,data):
        self.ARposes = data

    def GetRGBImage(self):
        return self.rgbImage 

    def GetDepthImage(self):
        return self.depthImage 

    def GetArPoses(self):
        return self.ARposes
    
