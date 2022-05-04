#!/usr/bin/env python

from __future__ import print_function
import rospy
import cv2 as cv
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import yaml
import utility as utl
from visual_guidance.msg import sixDof
import time


class AvoidanceNode:

    def __init__(self, paramsFile):

        # with open(paramsFile) as f:
        #     data = yaml.load(f, Loader=yaml.FullLoader)
        #     self._verticalCenter = 0.23 if not data.has_key("verticalCenter") else data["verticalCenter"]

        self._verticalCenter = 0.23
        self.bridge = CvBridge()
        self.avoid_sub = rospy.Subscriber("/avoidance/depth", Image, self.avoidCallback)
        self.num_pub = rospy.Publisher('/avoid_nums', sixDof, queue_size=10)
        self._key = -1
        self._image = None
        self._imInfoSet = False
        self._msg = sixDof()
        self._it = 0

        self._lateral = 0.3
        self._vertical = 0.3

        self._yy = [[],[],[]]
        self._zz = [[],[],[]]

        self._exported = False


    def avoidCallback(self,data):

        # print('q')
        self._it += 1

        dataTime = data.header.stamp.to_sec();

        if self._it == 150 and not self._exported:
            utl.exportData(self._yy, 'yy')
            utl.exportData(self._zz, 'zz')
            self._exported = True

        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # print(gray.shape)

        # if self._it > 900:
        # filename = "/home/hamidreza/project_82/frames/6/file_%d.jpg"%self._it
        # cv.imwrite(filename, gray)

        if not self._imInfoSet:
            self.setImInfo(gray)

        self._image = self.processImg(gray, dataTime)

        # cv.imshow('bin', self._image)
        # self._key = cv.waitKey(1)


    def processImg(self, img, dTime):

        _, image = cv.threshold(img,30,255,cv.THRESH_BINARY)
        self.setParams(image, dTime)

        return image


    def setImInfo(self, image):

        self._imInfoSet = True

        self._imageHeight, self._imageWidth = image.shape

        self._uh = int(self._imageHeight*self._verticalCenter)
        self._dh = int(self._imageHeight*(1-self._verticalCenter))
        self._hw = int(self._imageWidth/2)

        self._upNum = self._uh * self._imageWidth
        self._downNum = self._dh * self._imageWidth
        self._leftNum = self._hw * self._imageWidth

        # print('nums', self._upNum, self._downNum, self._leftNum)


    def setParams(self, image, dTime):

        upSeg, _, _ = utl.cropRect((0, 0, self._imageWidth, self._uh), image)
        downSeg, _, _ = utl.cropRect((0, self._uh, self._imageWidth, self._dh), image)
        leftSeg, _, _ = utl.cropRect((0, 0, self._hw, self._imageHeight), image)
        rightSeg, _, _ = utl.cropRect((self._hw, 0, self._hw, self._imageHeight), image)

        u = cv.countNonZero(upSeg)
        d = cv.countNonZero(downSeg)
        l = cv.countNonZero(leftSeg)
        r = cv.countNonZero(rightSeg)


        self._U = np.float32(u)/self._upNum
        self._D = np.float32(d)/self._downNum
        self._L = np.float32(l)/self._leftNum
        self._R = np.float32(r)/self._leftNum


        self._msg.x = self._it
        self._msg.y = (self._R * self._lateral) - (self._L * self._lateral)
        self._msg.z = (self._D * self._vertical) - (self._U * self._vertical)

        # if self._it > 1100:
        utl.saveData(self._yy, self._msg.y, dTime)
        utl.saveData(self._zz, self._msg.z, dTime)






rospy.init_node('avoidance_node', anonymous=True)

an = AvoidanceNode("/home/hamidreza/thesis/workSpace/src/visual_guidance/params/params.yaml")

rospy.spin()
