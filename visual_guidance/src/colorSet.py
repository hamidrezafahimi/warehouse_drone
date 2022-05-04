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
from visual_guidance.msg import cell
# from visual_guidance.msg import nodeGrid
import numpy as np

MODES = {
    1: "image",
    2: "simulation",
    3: "video"
    }
MODE = MODES[1]

mainUIName = "Main UI"


class TestNode:

    def __init__(self, paramsFile, calibFile):

        self.bridge = CvBridge()

        if MODE == "simulation":
            self.image_sub = rospy.Subscriber("/quadrotor_1/front/image_raw",Image,self.imageCallback)
            self.cmd_pub = rospy.Publisher('/quadrotor_1/cmd_vel1', Twist, queue_size=1)
        elif MODE == "video":
            self.image_sub = rospy.Subscriber("/movie",Image,self.imageCallback)
            self.cmd_pub = rospy.Publisher('/quadrotor_1/cmd_vel', Twist, queue_size=1)
        else:
            self.image_sub = rospy.Subscriber("/tello/camera/image_raw",Image,self.imageCallback)
            self.cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)


        self._paramsFile = paramsFile

        with open(paramsFile) as f:

            data = yaml.load(f, Loader=yaml.FullLoader)
            self.lowH = 0 if not data.has_key("lowH") else data["lowH"]
            self.lowS = 0 if not data.has_key("lowS") else data["lowS"]
            self.lowV = 0 if not data.has_key("lowV") else data["lowV"]
            self.highH = 179 if not data.has_key("highH") else data["highH"]
            self.highS = 255 if not data.has_key("highS") else data["highS"]
            self.highV = 255 if not data.has_key("highV") else data["highV"]


        self._setThresh = False
        self._trackbarsCreated = False
        self._windowCreated = False
        self._key = -1
        # cv2.namedWindow(mainUIName, cv2.WINDOW_GUI_NORMAL)3
        self._extractor = shd.GridExtractor(paramsFile, True)
        self._extractor.setColorThresholds([[self.lowH]], [self.lowS],
                                         [self.lowV], [[self.highH]],
                                         [self.highS], [self.highV])
        # vid = "/home/hamidreza/thesis/videos/20200920_192308.mp4"
        vid = "/home/hamidreza/thesis/videos/20200921_144642.mp4"
        self._cap = cv2.VideoCapture(vid)


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

        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        imageCopy = image.copy()
        nodes = self._extractor.extract(image)

        if not self._windowCreated:
            cv2.namedWindow(mainUIName, cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow(mainUIName, 640,480)
            self._windowCreated = True

        if not self._trackbarsCreated:
            # cv2.namedWindow(mainUIName, )
            cv2.createTrackbar("Low H", mainUIName , self.lowH, 180, self.lowHChanged)
            cv2.createTrackbar("High H", mainUIName , self.highH, 180, self.highHChanged)
            cv2.createTrackbar("Low S", mainUIName , self.lowS, 255, self.lowSChanged)
            cv2.createTrackbar("High S", mainUIName , self.highS, 255, self.highSChanged)
            cv2.createTrackbar("Low V", mainUIName , self.lowV, 255, self.lowVChanged)
            cv2.createTrackbar("High V", mainUIName , self.highV, 255, self.highVChanged)
            self._trackbarsCreated = True

        cv2.imshow(mainUIName, image)
        self._key = cv2.waitKey(1)



    def runIm(self):
        while True:
            # res, image = self._cap.read()
            image = cv2.imread('/home/hamidreza/project_82/frames/2/file_399.jpg')
            # image = cv2.imread('/home/hamidreza/Pictures/fe.png')
            imageCopy = image.copy()
            # self._it += 1
            # print(1)
            # img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # print (img[100,100])
            if self._key == 52:
                self._setThresh = not self._setThresh

            nodes = self._extractor.extract(image)

            if not self._windowCreated:
                cv2.namedWindow(mainUIName, cv2.WINDOW_GUI_NORMAL)
                cv2.resizeWindow(mainUIName, 640,480)
                self._windowCreated = True

            if not self._trackbarsCreated:
                # cv2.namedWindow(mainUIName, )
                cv2.createTrackbar("Low H", mainUIName , self.lowH, 180, self.lowHChanged)
                cv2.createTrackbar("High H", mainUIName , self.highH, 180, self.highHChanged)
                cv2.createTrackbar("Low S", mainUIName , self.lowS, 255, self.lowSChanged)
                cv2.createTrackbar("High S", mainUIName , self.highS, 255, self.highSChanged)
                cv2.createTrackbar("Low V", mainUIName , self.lowV, 255, self.lowVChanged)
                cv2.createTrackbar("High V", mainUIName , self.highV, 255, self.highVChanged)
                self._trackbarsCreated = True

            cv2.imshow(mainUIName, image)
            self._key = cv2.waitKey(1)
            if cv2.waitKey(500) & 0xFF == ord('q'):
                break
            # elif self._windowCreated:
            #     cv2.destroyAllWindows()
            #     self._windowCreated = False




rospy.init_node('pycvi', anonymous=True)

if MODE=="simulation":
    testNode = TestNode("/home/hamidreza/thesis/workSpace/src/visual_guidance/params/params.yaml",
                        "/home/hamidreza/thesis/workSpace/src/visual_guidance/params/calib1.yaml")
elif MODE=="video":
    testNode = TestNode("/home/hamidreza/thesis/workSpace/src/visual_guidance/params/params2.yaml",
                        "/home/hamidreza/thesis/workSpace/src/visual_guidance/params/telloCam.yaml")
else:
    testNode = TestNode("/home/hamidreza/thesis/workSpace/src/visual_guidance/params/params2.yaml",
                        "/home/hamidreza/thesis/workSpace/src/visual_guidance/params/telloCam.yaml")
    testNode.runIm()


rospy.spin()
