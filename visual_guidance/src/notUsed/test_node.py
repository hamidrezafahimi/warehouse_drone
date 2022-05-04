#!/usr/bin/env python

from __future__ import print_function
import windowDetection
import rospy
import yaml
import time
import cv2 as cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
# from simulate.msg import imtodyn
from std_msgs.msg import Empty


BUILDING_SELECTION_MODE = False

MODES = {
    1: "test",
    2: "simulation",
    3: "video"
    }
MODE = MODES[2]

mainUIName = "Main UI"


class CommandMode:

    FULL_AUTO_MATRIX    = 7
    SEMI_AUTO_MATRIX    = 6
    FULL_AUTO_COLOR     = 5
    SEMI_AUTO_COLOR     = 4
    AUTO_2STEP          = 3
    SEMI_AUTO_TRACKER   = 2
    MANUAL              = 1


class TestNode:

    def __init__(self, paramsFile, calibFile):
        self.bridge = CvBridge()
        if MODE == "simulation":
            self.image_sub = rospy.Subscriber("/quadrotor_1/front/image_raw",Image,self.imageCallback)
            # self.cmd_pub = rospy.Publisher('/quadrotor_1/cmd_vel', Twist, queue_size=1)
            self.cmd_pub = rospy.Publisher('/quadrotor_1/cmd_vel1', Twist, queue_size=1)
        elif MODE == "video":
            self.image_sub = rospy.Subscriber("/movie",Image,self.imageCallback)
            self.cmd_pub = rospy.Publisher('/quadrotor_1/cmd_vel', Twist, queue_size=1)
        else:
            self.image_sub = rospy.Subscriber("/tello/camera/image_raw",Image,self.imageCallback)
            self.cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)

        # self.dyn_pub = rospy.Publisher('visual_info', imtodyn, queue_size=1)
        self.pubLand    = rospy.Publisher('/tello/land',Empty,queue_size=10)
        self.pubTakeOff = rospy.Publisher('/tello/takeoff',Empty,queue_size=10)

        # self.msg = imtodyn()
        self._cmdMode = CommandMode.MANUAL

        self._videoWriter = None

        self._paramsFile = paramsFile
        with open(paramsFile) as f:

            data = yaml.load(f, Loader=yaml.FullLoader)
            self.lowH = 0 if not data.has_key("lowH") else data["lowH"]
            self.lowS = 0 if not data.has_key("lowS") else data["lowS"]
            self.lowV = 0 if not data.has_key("lowV") else data["lowV"]
            self.highH = 179 if not data.has_key("highH") else data["highH"]
            self.highS = 255 if not data.has_key("highS") else data["highS"]
            self.highV = 255 if not data.has_key("highV") else data["highV"]
            self.thresh = 30 if not data.has_key("thresh") else data["thresh"]
            self.contourRemovalThreshold = 100 if not data.has_key("contourRemovalThreshold") else data["contourRemovalThreshold"]
            self._verticalCenter = 0.23 if not data.has_key("verticalCenter") else data["verticalCenter"]

            # self._detector = windowDetection.windowDetector(self.thresh, self.contourRemovalThreshold,
            #                                                           deviationFactor, segmentThreshold, segmentationMethod, entranceThresh,
            #                                                           peakNeighbourhood, self._verticalCenter, kernelSize=(verticalKernelSize, horizontalKernelSize))
            self._detector = windowDetection.WindowDetector(paramsFile)

            if not BUILDING_SELECTION_MODE:
                self._detector.extractor.setColorThresholds([[self.lowH]], [self.lowS],
                                             [self.lowV], [[self.highH]],
                                             [self.highS], [self.highV])

        fs = cv2.FileStorage(calibFile, cv2.FILE_STORAGE_READ)
        # self._detector.analyzer.setCameraParams(fs.getNode("camera_matrix").mat(), fs.getNode("distortion_coefficients").mat())

        self._windowCreated = False
        self._key = -1
        self._initRect = ()
        self._initImage = None
        self._firstRoiPoint = None
        self._t1 = None
        self._tempSelectedRoi = None
        self._selectedROI = None
        self._cutAutoModes = False
        self._trackbarsCreated = False
        self._instantCmd = []
        self._entranceMode = self._detector.controller.commandModes[1]
        self._stop = False
        self._it = 0

    def onSlidersChange(self):
        self._detector.extractor.setColorThresholds([[self.lowH]], [self.lowS],
                                     [self.lowV], [[self.highH]],
                                     [self.highS], [self.highV])
        self._detector.extractor.setConfigThresholds(self.thresh, self.contourRemovalThreshold)

        with open(self._paramsFile, "r") as f:
            dict = yaml.load(f, Loader=yaml.FullLoader)

        dict["lowH"] = self.lowH
        dict["lowS"] = self.lowS
        dict["lowV"] = self.lowV
        dict["highH"] = self.highH
        dict["highS"] = self.highS
        dict["highV"] = self.highV
        dict["thresh"] = self.thresh
        dict["contourRemovalThreshold"] = self.contourRemovalThreshold

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
    def threshChanged(self, value):
        self.thresh = value
        self.onSlidersChange()
    def contourRemovalThresholdChanged(self, value):
        self.contourRemovalThreshold = value
        self.onSlidersChange()


    def onMouse(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            print('EVENT_LBUTTONDOWN')
            self._t1 = time.time()
            self._firstRoiPoint = [x,y]
            self._detector.tracker.selectTargetContour(x, y)
        elif event == cv2.EVENT_MOUSEMOVE:
            if self._t1!=None and (time.time()-self._t1)>0.05:
                self._tempSelectedRoi = (self._firstRoiPoint[0], self._firstRoiPoint[1], abs(x-self._firstRoiPoint[0]), abs(y-self._firstRoiPoint[1]))
            pass
        elif event == cv2.EVENT_LBUTTONUP:
            deltaT = time.time()-self._t1
            if deltaT>0.25:
                self._selectedROI = (self._firstRoiPoint[0], self._firstRoiPoint[1], abs(x-self._firstRoiPoint[0]), abs(y-self._firstRoiPoint[1]))
            self._t1 = None
            self._tempSelectedRoi = None
        elif event == cv2.EVENT_RBUTTONDOWN:
            # self._cutAutoModes = True
            self._instantCmd = [x, y]
            pass


    def imageCallback(self,data):

        self._it += 1

        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image = cv2.resize(image, (640,480))
        entrance = False
        safe = False

        if self._key == 118:
            if self._videoWriter == None:
                self._videoWriter = cv2.VideoWriter('sample videos/output.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 30.0, (640,360))
            else:
                self._videoWriter.release()
                self._videoWriter = None

        elif self._key == ord('b'):
            cv2.imwrite("img13.jpg", image)

        elif self._key == 49:# or self._cutAutoModes:
            self.cmd_pub.publish(Twist())
            self._detector.darkExtractionPermission = False

            self._cmdMode = CommandMode.MANUAL
            self._detector.tracker.setTrackingMode(1)#, self._detector.extractor)

            self._cutAutoModes = False

        elif self._key == 50:

            self.cmd_pub.publish(Twist())
            self._detector.darkExtractionPermission = False

            rect = cv2.selectROI(mainUIName, image)

            self._windowCreated = False
            if any(rect):
                self._cmdMode = CommandMode.SEMI_AUTO_TRACKER
                self._detector.tracker.setTrackingMode(2)#, self._detector.extractor)
                self._detector.tracker.initTrackerMF(rect, image)

        elif self._key == 51:

            self.cmd_pub.publish(Twist())
            rect = cv2.selectROI(mainUIName, image)
            self._detector.darkExtractionPermission = False
            self._windowCreated = False
            if any(rect):
                self._cmdMode = CommandMode.AUTO_2STEP
                self._detector.tracker.setTrackingMode(3)#, self._detector.extractor)
                self._detector.tracker.initTrackerMF(rect, image)

        elif self._selectedROI!=None:
            # print('\n\n\n\nttt\n\n\n\n')
            self.cmd_pub.publish(Twist())

            self._cmdMode = CommandMode.SEMI_AUTO_TRACKER
            self._detector.tracker.setTrackingMode(2)#, self._detector.extractor)
            self._detector.darkExtractionPermission = False

            self._windowCreated = False
            self._detector.tracker.initTrackerMF(self._selectedROI, image)

            self._selectedROI = None

        elif self._key == 52:
            self.cmd_pub.publish(Twist())
            self._detector._colorThresholdsSet = True
            self._detector.tracker.setTrackingMode(4)#, self._detector.extractor)
            self._detector.darkExtractionPermission = False

            self._cmdMode = CommandMode.SEMI_AUTO_COLOR
            # self._initRect = cv2.selectROI(mainUIName, image)
            # self._initImage = image.copy()
            self._windowCreated = False
            # if len(self._initRect)!=0:
            #     self._detector.buildingSelected(image, self._initRect)

        elif self._key == 53:
            self.cmd_pub.publish(Twist())
            self._detector.darkExtractionPermission = False

            self._detector.tracker.setTrackingMode(4)#, self._detector.extractor)

            self._cmdMode = CommandMode.FULL_AUTO_COLOR
            self._initRect = cv2.selectROI(mainUIName, image)
            self._initImage = image.copy()
            self._windowCreated = False
            if len(self._initRect)!=0:
                self._detector.buildingSelected(image, self._initRect)

        elif self._key == 54:
            self.cmd_pub.publish(Twist())
            self._detector._colorThresholdsSet = True
            self._detector.tracker.setTrackingMode(5)#, self._detector.extractor)
            self._detector.darkExtractionPermission = False

            self._cmdMode = CommandMode.SEMI_AUTO_MATRIX
            # self._initRect = cv2.selectROI(mainUIName, image)
            # self._initImage = image.copy()
            self._windowCreated = False
            # if len(self._initRect)!=0:
            #     self._detector.buildingSelected(image, self._initRect)

        elif self._key == 55:
            self.cmd_pub.publish(Twist())
            self._detector.darkExtractionPermission = False
            self._detector.tracker.setTrackingMode(5)#, self._detector.extractor)

            self._cmdMode = CommandMode.FULL_AUTO_MATRIX
            self._initRect = cv2.selectROI(mainUIName, image)
            self._initImage = image.copy()
            self._windowCreated = False
            if len(self._initRect)!=0:
                self._detector.buildingSelected(image, self._initRect)

        elif self._key == ord('s'):
            if self._stop:
                self._stop = False
            else:
                self._stop = True

        elif self._key == 13:
            self.pubTakeOff.publish(Empty())

        elif self._key == 32:
            self.pubLand.publish(Empty())

        elif self._key == ord('x'):
            if self._detector.analyzer.darkExtractionPermission:
                 self._detector.analyzer.darkExtractionPermission = False
            else:
                 self._detector.analyzer.darkExtractionPermission = True
                 self._stop = True

        elif self._key == ord('z'):
            if self._detector.analyzer.thresholdTuning:
                 self._detector.analyzer.thresholdTuning = False
            else:
                 self._detector.analyzer.thresholdTuning = True
                 # self._stop = True

        # try:
            # pass
        # except Exception as exp:
        #     print(exp)
        self._detector.detect(image)

        if self._cmdMode!=CommandMode.MANUAL:

            if self._key == 97:
                self._entranceMode = self._detector.controller.commandModes[2]
            elif self._key == 100:
                self._entranceMode = self._detector.controller.commandModes[3]
            elif self._key == 113:
                self._entranceMode = self._detector.controller.commandModes[4]
            elif self._key == 101:
                self._entranceMode = self._detector.controller.commandModes[5]
            elif self._key == 119:
                self._entranceMode = self._detector.controller.commandModes[1]

            if self._entranceMode == self._detector.controller.commandModes[2]:
                vx, vy, vy_linear, vz, safe, entrance = self._detector.controller.command(2)
            elif self._entranceMode == self._detector.controller.commandModes[3]:
                vx, vy, vy_linear, vz, safe, entrance = self._detector.controller.command(3)
            elif self._entranceMode == self._detector.controller.commandModes[4]:
                vx, vy, vy_linear, vz, safe, entrance = self._detector.controller.command(4)
            elif self._entranceMode == self._detector.controller.commandModes[5]:
                vx, vy, vy_linear, vz, safe, entrance = self._detector.controller.command(5)
            elif self._detector.tracker.trackingMode!="2step":
                vx, vy, vy_linear, vz, safe, entrance = self._detector.controller.command(6)
            else:
                vx, vy, vy_linear, vz, safe, entrance = self._detector.controller.command()

            # print('tn Intersects: ', self._detector.analyzer.verticalIntersect, self._detector.analyzer.horizontalIntersect)
            cv2.putText(image,'vertical border Intersect: '+str(self._detector.analyzer.verticalIntersect)\
             ,(10,410), cv2.FONT_HERSHEY_PLAIN, 1, [255,255,255], 2)
            cv2.putText(image,'horizontal border Intersect: '+str(self._detector.analyzer.horizontalIntersect)\
             ,(10,430), cv2.FONT_HERSHEY_PLAIN, 1, [255,255,255], 2)
            cv2.putText(image,'entrance mode: '+str(entrance) ,(10,450), cv2.FONT_HERSHEY_PLAIN,\
             1, [255,255,255], 2)
            cv2.putText(image,'published cmdVel: {:.03f}, {:.03f}, {:.03f}, {:.03f}'.format(vx, vy, vy_linear, vz) + ', '+str(safe)\
             ,(10,470), cv2.FONT_HERSHEY_PLAIN, 1, [255,255,255], 2)

        else:
            cv2.putText(image,'manual mode' ,(10,460), cv2.FONT_HERSHEY_PLAIN, 1, [255,255,255], 2)

        cv2.line(image, (image.shape[1]/2,0), (image.shape[1]/2,image.shape[0]), (255,255,255), 1)
        cv2.line(image, (0,int(image.shape[0]*self._verticalCenter)), (image.shape[1],\
         int(image.shape[0]*self._verticalCenter)), (255,255,255), 1)

        # filename = "/home/hamidreza/project_82/frames/5/file_%d.jpg"%self._it
        # cv2.imwrite(filename, image)

        cmdMsg = Twist()

        if self._stop:
            pass

        elif len(self._instantCmd)!=0:

            _, vy, _, vz, _, _ = self._detector.controller.command(1, self._instantCmd)

            cmdMsg.linear.y = 0
            cmdMsg.linear.z = vz
            cmdMsg.angular.y = 0
            cmdMsg.angular.z = vy

            if MODE == "test":
                VX = cmdMsg.linear.x
                cmdMsg.linear.x = -cmdMsg.linear.y
                cmdMsg.linear.y = VX
                cmdMsg.angular.z = -1*cmdMsg.angular.z

            self._instantCmd = []

        elif self._cmdMode == CommandMode.MANUAL:

            if MODE == "simulation":
                if self._key == ord('j'):
                    cmdMsg.linear.y = 0.6
                elif self._key == ord('i'):
                    cmdMsg.linear.x = 0.6
                elif self._key == ord('l'):
                    cmdMsg.linear.y = -0.6
                elif self._key == ord('k'):
                    cmdMsg.linear.x = -0.6
                elif self._key == ord('u'):
                    cmdMsg.angular.z = 0.5
                elif self._key == ord('o'):
                    cmdMsg.angular.z = -0.5
                elif self._key == ord('y'):
                    cmdMsg.linear.z = 0.5
                elif self._key == ord('h'):
                    cmdMsg.linear.z = -0.5
            else:
                if self._key == ord('i'):
                    cmdMsg.linear.y = 0.5
                elif self._key == ord('l'):
                    cmdMsg.linear.x = 0.5
                elif self._key == ord('k'):
                    cmdMsg.linear.y = -0.5
                elif self._key == ord('j'):
                    cmdMsg.linear.x = -0.5
                elif self._key == ord('o'):
                    cmdMsg.angular.z = 0.3
                elif self._key == ord('u'):
                    cmdMsg.angular.z = -0.3
                elif self._key == ord('y'):
                    cmdMsg.linear.z = 0.8
                elif self._key == ord('h'):
                    cmdMsg.linear.z = -0.8


        elif (self._cmdMode == CommandMode.AUTO_2STEP or self._cmdMode==CommandMode.FULL_AUTO_COLOR\
         or self._cmdMode==CommandMode.FULL_AUTO_MATRIX) and not safe:

            if self._key == 57:
                if self._cmdMode == CommandMode.AUTO_2STEP:
                    success, new = self._detector.tracker.trFindAgain(image)
                else:
                    success, new = self._detector.tracker.trFindAgain(image, True, self._initRect, self._initImage)
                if not success:
                    pass
                else:
                    self._detector.buildingSelected(image, new)

        else:

            if safe:
                cmdMsg.linear.x = vx
                cmdMsg.linear.y = vy_linear
                cmdMsg.linear.z = vz
                cmdMsg.angular.x = 0
                cmdMsg.angular.y = 0
                cmdMsg.angular.z = vy

            if entrance:
                print('entrance')
                cmdMsg.linear.x = 0.4
                cmdMsg.linear.y = vy_linear
                cmdMsg.linear.z = 0
                cmdMsg.angular.x = 0
                cmdMsg.angular.y = 0
                cmdMsg.angular.z = 0

            if MODE == "test":
                VX = cmdMsg.linear.x
                cmdMsg.linear.x = -cmdMsg.linear.y
                cmdMsg.linear.y = VX
                cmdMsg.angular.z = -1*cmdMsg.angular.z

        if safe or self._cmdMode== CommandMode.MANUAL:
            self.cmd_pub.publish(cmdMsg)
        else:
            self.cmd_pub.publish(Twist())

        if not self._windowCreated:
            cv2.namedWindow(mainUIName, cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow(mainUIName, 640,480)
            cv2.setMouseCallback(mainUIName, self.onMouse)
            self._windowCreated = True

        if self._cmdMode==CommandMode.SEMI_AUTO_COLOR or self._cmdMode==CommandMode.SEMI_AUTO_MATRIX:
            if not self._trackbarsCreated:
                # cv2.namedWindow(mainUIName, )
                cv2.createTrackbar("Low H", mainUIName , self.lowH, 180, self.lowHChanged)
                cv2.createTrackbar("High H", mainUIName , self.highH, 180, self.highHChanged)
                cv2.createTrackbar("Low S", mainUIName , self.lowS, 255, self.lowSChanged)
                cv2.createTrackbar("High S", mainUIName , self.highS, 255, self.highSChanged)
                cv2.createTrackbar("Low V", mainUIName , self.lowV, 255, self.lowVChanged)
                cv2.createTrackbar("High V", mainUIName , self.highV, 255, self.highVChanged)
                cv2.createTrackbar("Thresh", mainUIName , self.thresh, 255, self.threshChanged)
                cv2.createTrackbar("contourRemovalThreshold", mainUIName , self.contourRemovalThreshold, 500, self.contourRemovalThresholdChanged)
                self._trackbarsCreated = True
        else:
            if self._trackbarsCreated:
                cv2.destroyWindow(mainUIName)
                self._trackbarsCreated = False
                cv2.namedWindow(mainUIName, cv2.WINDOW_GUI_NORMAL)
                cv2.resizeWindow(mainUIName, 640,480)

            # else:
            #     cv2.namedWindow(mainUIName)

        if self._videoWriter != None:
            self._videoWriter.write(image)
            cv2.putText(image,"Saving Video",(20, image.shape[0] - 20), cv2.FONT_HERSHEY_PLAIN, 1, [0,0,255], 2)

        if self._tempSelectedRoi!=None:
            cv2.rectangle(image, (self._tempSelectedRoi[0],self._tempSelectedRoi[1]), (self._tempSelectedRoi[0]+self._tempSelectedRoi[2], self._tempSelectedRoi[1]+self._tempSelectedRoi[3]), (255,0,0), 2)

        cv2.imshow(mainUIName, image)
        self._key = cv2.waitKey(1)


rospy.init_node('test_node', anonymous=True)

if MODE=="simulation":
    testNode = TestNode("../params/params.yaml", "../params/calib1.yaml")
else:
    testNode = TestNode("../params/params.yaml", "../params/telloCam.yaml")

rospy.spin()
