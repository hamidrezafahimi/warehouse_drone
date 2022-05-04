#!/usr/bin/env python

from __future__ import print_function
import rospy
import roslib
# roslib.load_manifest('visual_guidance')
import yaml
import time
import cv2 as cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty
import shelfDetection as shd
import shelfAnalysis as sha
from visual_guidance.msg import shelfGrid
# from visual_guidance.msg import nodeGrid
import numpy as np


BUILDING_SELECTION_MODE = False

MODES = {
    1: "test",
    2: "simulation",
    3: "video"
    }
MODE = MODES[2]

mainUIName = "Main UI"


class TestNode:

    def __init__(self, paramsFile, calibFile):

        self.bridge = CvBridge()
        if MODE == "simulation":
            self.image_sub = rospy.Subscriber("/quadrotor_1/front/image_raw",Image,self.imageCallback)
        elif MODE == "video":
            self.image_sub = rospy.Subscriber("/movie",Image,self.imageCallback)
        else:
            self.image_sub = rospy.Subscriber("/tello/camera/image_raw",Image,self.imageCallback)

        self._infoPub = rospy.Publisher('/shelfInfo', shelfGrid, queue_size=1)
        # self._nodeInfoPub = rospy.Publisher('/nodesInfo', nodeGrid, queue_size=1)

        self._paramsFile = paramsFile

        with open(paramsFile) as f:

            data = yaml.load(f, Loader=yaml.FullLoader)
            self.lowH = 0 if not data.has_key("lowH") else data["lowH"]
            self.lowS = 0 if not data.has_key("lowS") else data["lowS"]
            self.lowV = 0 if not data.has_key("lowV") else data["lowV"]
            self.highH = 179 if not data.has_key("highH") else data["highH"]
            self.highS = 255 if not data.has_key("highS") else data["highS"]
            self.highV = 255 if not data.has_key("highV") else data["highV"]

            self._extractor = shd.GridExtractor(paramsFile)
            self._analyzer = sha.GridAnalyzer(paramsFile, self._extractor)

            self._extractor.setColorThresholds([[self.lowH]], [self.lowS],
                                             [self.lowV], [[self.highH]],
                                             [self.highS], [self.highV])

        fs = cv2.FileStorage(calibFile, cv2.FILE_STORAGE_READ)
        self._extractor.setCameraParams(fs.getNode("camera_matrix").mat(), fs.getNode("distortion_coefficients").mat())

        self._windowCreated = False
        self._key = -1
        self._trackbarsCreated = False
        # self._entranceMode = self._detector.controller.commandModes[1]
        self._it = 0
        self.sendPermission = True


    def onSlidersChange(self):

        self._extractor.setColorThresholds([[self.lowH]], [self.lowS],
                                     [self.lowV], [[self.highH]],
                                     [self.highS], [self.highV])

        with open(self._paramsFile, "r") as f:
            dict = yaml.load(f, Loader=yaml.FullLoader)

        dict["lowH"] = self.lowH
        dict["lowS"] = self.lowS
        dict["lowV"] = self.lowV
        dict["highH"] = self.highH
        dict["highS"] = self.highS
        dict["highV"] = self.highV

        with open(self._paramsFile, "w") as f:
            yaml.dump(dict, f)


    def lowHChanged(self, value):
        self.lowH = value
        self.onSlidersChange()
    def lowSChanged(self, value):
        self.lowS = value
        self.onSlidersChange()
    def lowVChanged(self, value):
        self.lowV = value
        self.onSlidersChange()
    def highHChanged(self, value):
        self.highH = value
        self.onSlidersChange()
    def highSChanged(self, value):
        self.highS = value
        self.onSlidersChange()
    def highVChanged(self, value):
        self.highV = value
        self.onSlidersChange()


    def imageCallback(self,data):

        self._it += 1

        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # image = cv2.resize(image, (640,480))

        sgmsg = shelfGrid()

        if self.sendPermission:
            nodes = self._extractor.extract(image)
            grid, nGrid = self._analyzer.analyze(image, nodes)
            # print (nGrid)
            # print (type(grid))
            # print (type(grid[0]))
            # print (type(grid[0][0]))
            # print ('---')

            if self._analyzer.OK:
                sgmsg.gridInfo = np.ravel(grid)
                sgmsg.nodeGridInfo = np.ravel(nGrid)
                self._infoPub.publish(sgmsg)
                self.sendPermission = False
            # self._nodeInfoPub.publish(ngmsg)


        # if not self._windowCreated:
        #     cv2.namedWindow(mainUIName, cv2.WINDOW_GUI_NORMAL)
        #     cv2.resizeWindow(mainUIName, 640, 480)
        #     self._windowCreated = True

        # if self._cmdMode==CommandMode.SET:
        #     if not self._trackbarsCreated:
        #         # cv2.namedWindow(mainUIName, )
        #         cv2.createTrackbar("Low H", mainUIName , self.lowH, 180, self.lowHChanged)
        #         cv2.createTrackbar("High H", mainUIName , self.highH, 180, self.highHChanged)
        #         cv2.createTrackbar("Low S", mainUIName , self.lowS, 255, self.lowSChanged)
        #         cv2.createTrackbar("High S", mainUIName , self.highS, 255, self.highSChanged)
        #         cv2.createTrackbar("Low V", mainUIName , self.lowV, 255, self.lowVChanged)
        #         cv2.createTrackbar("High V", mainUIName , self.highV, 255, self.highVChanged)
        #         self._trackbarsCreated = True
        # else:
        #     if self._trackbarsCreated:
        #         cv2.destroyWindow(mainUIName)
        #         self._trackbarsCreated = False
        #         cv2.namedWindow(mainUIName, cv2.WINDOW_GUI_NORMAL)
        #         cv2.resizeWindow(mainUIName, 640,480)

            # else:
            #     cv2.namedWindow(mainUIName)

        cv2.imshow(mainUIName, image)
        cv2.waitKey(1)


rospy.init_node('pycvi', anonymous=True)

if MODE=="simulation":
    testNode = TestNode("/home/hamidreza/thesis/workSpace/src/visual_guidance/params/params.yaml", "/home/hamidreza/thesis/workSpace/src/visual_guidance/params/calib1.yaml")
else:
    testNode = TestNode("/home/hamidreza/thesis/workSpace/src/visual_guidance/params/params.yaml", "/home/hamidreza/thesis/workSpace/src/visual_guidance/params/telloCam.yaml")

rospy.spin()
