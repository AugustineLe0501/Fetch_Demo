#!/usr/bin/env python 

import rospy
import time
import math
import cv2
import numpy as np
import geometry_msgs.msg
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from numpy.linalg import inv
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers

class PoseProcessing:
    def __init__(self):
        rospy.loginfo("Waiting for pose processing")
        self.tf = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

    def SetRGBImage(self, Image):
        self.RGBImage = Image

    def SetDepthImage(self, Image):
        self.DepthImage = Image

    def SetArPose(self, AlvarMarkers):
        self.ArPoses = AlvarMarkers

    def DetermineBlocks(self, color):
        HSV_image = cv2.cvtColor(self.RGBImage,cv2.COLOR_BGR2HSV)
        ####### Red
        if (color == "red"):
            # lower mask (0-10)
            lower_red = np.array([0,60,60])
            upper_red = np.array([10,255,255])
            mask0 = cv2.inRange(HSV_image, lower_red, upper_red)
            # upper mask (170-180)
            lower_red = np.array([170,60,60])
            upper_red = np.array([180,255,255])
            mask1 = cv2.inRange(HSV_image, lower_red, upper_red)
            # join my masks 
            Thresh_image = mask0+mask1  

        ####### Green
        if (color == "green"):
            # define range of green color in HSV
            lower_green = np.array([40,60,60])
            upper_green = np.array([80,255,255])
            # Threshold the HSV image to get only blue colors
            Thresh_image = cv2.inRange(HSV_image, lower_green, upper_green)

        ####### Blue
        if (color == "blue"):
            # define range of blue color in HSV
            lower_blue = np.array([100,60,60])
            upper_blue = np.array([140,255,255])
            # Threshold the HSV image HSV_image get only blue colors
            Thresh_image = cv2.inRange(HSV_image, lower_blue, upper_blue)

        ####### Convert to unit8 to find a contour
        Thresh_image_u8 = Thresh_image.astype(np.uint8)
        self.FindCenterPoint(Thresh_image_u8)
        self.CalculateOrientation(self.Center_X,self.Center_Y)
        self.CalculatePosition(self.Center_X,self.Center_Y)
        self.CameraToBaseTransform()

    def FindCenterPoint(self,Image):
        # Noise cancelling
        kernel = np.ones((4, 4), np.uint8)
        erode = cv2.erode(Image, kernel, iterations = 2)
        dilate = cv2.dilate(erode, kernel, iterations = 5)

        # Find contours center
        _, contours, _ = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if (area >2000):
                M = cv2.moments(cnt)
                self.Center_X = int(M["m10"] / M["m00"])
                self.Center_Y = int(M["m01"] / M["m00"])

    def CalculatePosition(self,u,v):
        depth = self.DepthImage[v,u]
        Z = depth + 0.06
        K = np.array([[554.2547,0,320.5],[0,554.2547,240.5],[0,0,1]])
        x = np.array([[u*Z, v*Z,Z]]).T
        self.Position = np.matmul(inv(K),x)
        return np.matmul(inv(K),x)

    def CalculateOrientation(self,u,v):
        counter = 3
        dl = self.DepthImage[v,u-counter]
        dr = self.DepthImage[v,u+counter]
        left = self.CalculatePosition(u-counter,v)
        right = self.CalculatePosition(u+counter,v)
        h = dr-dl
        b = abs(right[0]-left[0])
        self.Yaw = math.atan(h/b)

    def CameraToBaseTransform(self):
        if (self.tf.frameExists("base_link")) and (self.tf.frameExists("head_camera_rgb_optical_frame")):
            orientation = tf.transformations.quaternion_from_euler(0, 0, self.Yaw)
            t = self.tf.getLatestCommonTime("base_link", "head_camera_rgb_optical_frame")
            p = geometry_msgs.msg.PoseStamped()
            p.header.frame_id = "head_camera_rgb_optical_frame"
            p.pose.position.x = self.Position[0]
            p.pose.position.y = self.Position[1]
            p.pose.position.z = 0.47
            p.pose.orientation.x = orientation[0]
            p.pose.orientation.y = orientation[1]
            p.pose.orientation.z = orientation[2]
            p.pose.orientation.w = orientation[3]

            self.BlockPose = self.tf.transformPose("base_link", p)
        else:
            print("aaaaa")
            
    def DeterminePlace(self):
        self.br.sendTransform(self.ARposes.markers.pose[-1],rospy.Time.now(),"ArTag_1","head_camera_rgb_optical_frame")
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "ArTag_1"
        p.pose.position.x = 1
        p.pose.position.y = 0
        p.pose.position.z = 0
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1

        self.PlacePose = self.tf.transformPose("base_link", p)
    
    def GetBlockPosition(self):
        return self.Position

    def GetBlockOrientation(self):
        return self.Yaw

    def GetBlockPose(self):
        return self.BlockPose

    def GetPlacePose(self):
        return self.PlacePose


        