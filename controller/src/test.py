#!/usr/bin/env python 

import rospy
import ros_numpy
import numpy as np
from numpy.linalg import inv
import math
import cv2

if __name__ == "__main__":
    RGB_image = cv2.imread('/home/augustine/fetch_ws/src/controller/Image/RGB_image.jpg')
    HSV_image = cv2.cvtColor(RGB_image,cv2.COLOR_BGR2HSV)

    ####### Red
    # lower mask (170-180)
    lower_red = np.array([0,70,70])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(HSV_image, lower_red, upper_red)
    # upper mask (170-180)
    lower_red = np.array([170,70,70])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(HSV_image, lower_red, upper_red)
    # join my masks 
    Red_image = mask0+mask1  

    ####### Green
    # define range of green color in HSV
    lower_green = np.array([40,50,50])
    upper_green = np.array([80,255,255])
    # Threshold the HSV image to get only blue colors
    Green_image = cv2.inRange(HSV_image, lower_green, upper_green)

    ####### Blue
    # define range of blue color in HSV
    lower_blue = np.array([100,70,70])
    upper_blue = np.array([140,255,255])
    # Threshold the HSV image HSV_image get only blue colors
    Blue_image = cv2.inRange(HSV_image, lower_blue, upper_blue)

    ####### Convert to unit8 to find a contour
    Red_image_u8 = Red_image.astype(np.uint8)
    Green_image_u8 = Green_image.astype(np.uint8)
    Blue_image_u8 = Blue_image.astype(np.uint8)

    # edges = cv2.Canny(RGB_image , 100, 200)
    # mask = edges + Green_image_u8

    kernel = np.ones((4, 4), np.uint8) 
    erode = cv2.erode(Blue_image_u8, kernel, iterations = 2)
    dilate = cv2.dilate(erode, kernel, iterations = 5)

    ####### Noise fillter
    _, contours, _ = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if (area <2000):
            contours.remove(cnt)
        else:
            M = cv2.moments(cnt)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

    print(cX)
    print(cY)

    # Find center point 
    # cv2.drawContours(RGB_image, contours, -1, (0,0,255), 3)
    # cv2.circle(RGB_image, (cX,cY), radius=5, color=(0, 0, 255), thickness=5)
    
    # Calculate the position
    # width = 0.1
    # depth = 0.62
    # Z = depth + width/2 
    # K = np.array([[554.25,0,320.5],[0,554.25,240.5],[0,0,1]])
    # x = np.array([[(cX+2)*Z, cY*Z,Z]]).T

    # right = np.matmul(inv(K),x)
    # print(cam[1])


    #hull = cv2.convexHull(contours)

    # imgray = cv2.cvtColor(RGB_image, cv2.COLOR_BGR2GRAY)
    # ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    # _, contours,_ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(RGB_image, contours, -1, (0,255,0), 3)

    #print(contours)
    #height, width = Red_image.shape
    # print(Red_image.shape)

    # redBand = RGB_image[:,:,2]
    # greenBand = RGB_image[:,:,1]
    # blueBand = RGB_image[:,:,0]

    # counter = 0
    # redPoints = numpy.array([],ndmin=3)
    # print(redPoints.ndim)
    # for i in range(height):
    #     for j in range(width): 
    #         if (greenBand[i,j] < 10 and blueBand[i,j] < 10 and redBand[i,j] > 80):
    #             counter = counter +1 
    #             redPoints[counter,:] = [i,j]
    #         else:
    #             RGB_image[i,j,:] = 0

    # counter = 0 
    # for i in range(height):
    #     for j in range(width):
    #         if (Red_image[i,j] == 255):
    #             counter = counter +1
            
    # print(counter)

    
    #cv2.imshow('Origin',RGB_image)
    #cv2.imshow('Test_R',Red_image)
    cv2.imshow('Test_G',dilate)
    #cv2.imshow('Test_B',Blue_image)
    cv2.waitKey(0)  
    cv2.destroyAllWindows() 