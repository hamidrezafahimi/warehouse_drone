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
from std_msgs.msg import Bool
import shelfDetection as shd
import shelfAnalysis as sha
from visual_guidance.msg import shelfGrid
from visual_guidance.msg import cell
from visual_guidance.msg import sixDof
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
            self.cmd_pub = rospy.Publisher('/quadrotor_1/cmd_vel', Twist, queue_size=1)
        elif MODE == "video":
            self.image_sub = rospy.Subscriber("/movie",Image,self.imageCallback)
            self.cmd_pub = rospy.Publisher('/quadrotor_1/cmd_vel', Twist, queue_size=1)
        else:
            self.image_sub = rospy.Subscriber("/tello/camera/image_raw",Image,self.imageCallback)
            self.cmd_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)

        self.pubLand    = rospy.Publisher('/tello/land',Empty,queue_size=10)
        self.pubTakeOff = rospy.Publisher('/tello/takeoff',Empty,queue_size=10)
        self._infoPub = rospy.Publisher('/shelfInfo', shelfGrid, queue_size=1)
        self._flagPub = rospy.Publisher('/avoidance/activate', Bool, queue_size=1)
        self.image_pub = rospy.Publisher('/gridImage', Image, queue_size=10)
        self.avoidance_sub = rospy.Subscriber('/avoid_nums', sixDof, self.setAvoids)

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
            if MODE == "simulation":
                self._analyzer = sha.GridAnalyzer(paramsFile, "/home/hamidreza/thesis/workSpace/src/warehouse_drone/visual_guidance/params/controlParams2.yaml", self._extractor)
            else:
                self._analyzer = sha.GridAnalyzer(paramsFile, "/home/hamidreza/thesis/workSpace/src/warehouse_drone/visual_guidance/params/controlParams.yaml", self._extractor)

            self._extractor.setColorThresholds([[self.lowH]], [self.lowS],
                                             [self.lowV], [[self.highH]],
                                             [self.highS], [self.highV])

        self.cell_sub = rospy.Subscriber("cellTopic",cell,self._analyzer.setCell)

        fs = cv2.FileStorage(calibFile, cv2.FILE_STORAGE_READ)
        self._analyzer.setCameraParams(fs.getNode("camera_matrix").mat(), fs.getNode("distortion_coefficients").mat())

        self._windowCreated = True
        self._key = -1
        self._trackbarsCreated = False
        self._it = 0
        self.sendPermission = True
        # self.sendPermission = False
        self._t1 = time.time()
        self._manual = True
        self._sgmsg = shelfGrid()
        self._avoidFlag = Bool()
        self._obsAv = False
        self._infoPubPermission = False
        self._setThresh = False
        self._trackbarsCreated = False
        vid = "/home/hamidreza/thesis/videos/20200921_144642.mp4"
        self._cap = cv2.VideoCapture(vid)
        self._writeVideo = False

        self._avoid_y = 0
        self._avoid_z = 0
        self._avoidFrameId = 0



    def reset(self):

        self.sendPermission = True
        self._infoPubPermission = False
        self._analyzer.trackerStarted = False
	self._analyzer._nearEnough = False
        self._t1 = time.time() - 1



    def setAvoids(self, data):

        # print('setAvoids called', data.y, data.z, data.x, self._it)
        self._avoid_y = data.y
        self._avoid_z = data.z
        self._avoidFrameId = data.x



    def onSlidersChange(self):
        self.extractor.setColorThresholds([[self.lowH]], [self.lowS],
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
        self._it += 1
        # print('\n\n\n')
        cmdMsg = Twist()
        # img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # print (img[100,100])
        if self._key == 52:
            self._setThresh = not self._setThresh

        if self._key == ord('b'):
            self._writeVideo = not self._writeVideo


        if self._infoPubPermission:
            self._infoPub.publish(self._sgmsg)
            self._infoPubPermission = False

        if MODE == "test":
            if self._key == 13:
                self.pubTakeOff.publish(Empty())

            elif self._key == 32:
                self.pubLand.publish(Empty())

        if self._key == 49:
            self._manual = not self._manual


        if MODE == "simulation":
            if self._key == ord('j'):
                cmdMsg.linear.y = 0.6
                vj = 0.6
            elif self._key == ord('i'):
                cmdMsg.linear.x = 0.6
                vi = 0.6
            elif self._key == ord('l'):
                cmdMsg.linear.y = -0.6
                vl = -0.6
            elif self._key == ord('k'):
                cmdMsg.linear.x = -0.6
                vk = -0.6
            elif self._key == ord('u'):
                cmdMsg.angular.z = 0.5
                vu = 0.5
            elif self._key == ord('o'):
                cmdMsg.angular.z = -0.5
                vo = -0.5
            elif self._key == ord('y'):
                cmdMsg.linear.z = 0.5
                vy = 0.5
            elif self._key == ord('h'):
                cmdMsg.linear.z = -0.5
                vh = -0.5
        else:
            if self._key == ord('i'):
                cmdMsg.linear.y = 0.5
                vi = 0.5
            elif self._key == ord('l'):
                cmdMsg.linear.x = 0.5
                vl = 0.5
            elif self._key == ord('k'):
                cmdMsg.linear.y = -0.5
                vk = -0.5
            elif self._key == ord('j'):
                cmdMsg.linear.x = -0.5
                vj = -0.5
            elif self._key == ord('o'):
                cmdMsg.angular.z = 0.3
                vo = 0.3
            elif self._key == ord('u'):
                cmdMsg.angular.z = -0.3
                vu = -0.3
            elif self._key == ord('y'):
                cmdMsg.linear.z = 0.4
                vy = 0.4
            elif self._key == ord('h'):
                cmdMsg.linear.z = -2
                vh = -0.4

        if self._manual:

            # print(cmdMsg)
            self.cmd_pub.publish(cmdMsg)
            cv2.imshow(mainUIName, image)
            self._key = cv2.waitKey(1)
            return


        if self._key == ord('q'):
            self.reset()
        # else:
            #
            # if MODE == "test":
            #     VX = cmdMsg.linear.x
            #     cmdMsg.linear.x = -cmdMsg.linear.y
            #     cmdMsg.linear.y = VX
            #     cmdMsg.angular.z = -1*cmdMsg.angular.z

        # if self._writeVideo:
        #     filename = "/home/hamidreza/project_82/frames/3/file_%d.jpg"%self._it
        #     cv2.imwrite(filename, image)

        nodes = self._extractor.extract(image)
        grid, nGrid = self._analyzer.analyze(image, nodes)

        fl = False

        cmlx, cmly, cmlz, cmaz = 0, 0, 0, 0
        if self.sendPermission and ((time.time() - self._t1)>1):

	    print('y1')
            if self._analyzer.OK:
                print ('analyzer.OK')
                self._sgmsg.gridInfo = np.ravel(grid)
                self._sgmsg.nodeGridInfo = np.ravel(nGrid)
                self.image_pub.publish(CvBridge().cv2_to_imgmsg(imageCopy))
                self._infoPubPermission = True
                self.sendPermission = False
                cv2.imwrite('pyim.jpg', image)

            elif len(self._analyzer.extrm) > 0:
                cmlx, cmly, cmlz, cmaz, _ = self._analyzer.command(image)


        elif self._analyzer.trackerStarted:
	    # print('y2')
            cmlx, cmly, cmlz, cmaz, fl = self._analyzer.command(image, False, True)

        else:
	    # print('y3')
            cmlx, cmly, cmlz, cmaz, fl = self._analyzer.command(image, True)

        # print('commands:', cmlx, cmly, cmlz, cmaz)

        if fl:
            if not self._obsAv:
                self._avoidFlag.data = True
                self._flagPub.publish(self._avoidFlag)
                self._obsAv = True
            cmdMsg.linear.x, cmdMsg.angular.z \
                         = (cmlx+(1*cmdMsg.linear.x)), (cmaz+(1*cmdMsg.angular.z))
            cmdMsg.linear.y, cmdMsg.linear.z = self._avoid_y, self._avoid_z

        else:
            cmdMsg.linear.x, cmdMsg.linear.y, cmdMsg.linear.z, cmdMsg.angular.z \
                         = (cmlx+(1*cmdMsg.linear.x)), (cmly+(1*cmdMsg.linear.y)), \
                         (cmlz+(1*cmdMsg.linear.z)), (cmaz+(1*cmdMsg.angular.z))
                     # = (cmlx+(0.33*cmdMsg.linear.x)), (cmly+(0.33*cmdMsg.linear.y)), \
                     #  (cmlz+(0.33*cmdMsg.linear.z)), (cmaz+(0.33*cmdMsg.angular.z))
        #
        if MODE == "test":
            VX = cmdMsg.linear.x
            cmdMsg.linear.x = -cmdMsg.linear.y
            cmdMsg.linear.y = VX
            cmdMsg.angular.z = -1*cmdMsg.angular.z

        self.cmd_pub.publish(cmdMsg)

        # cv2.line(image, (image.shape[1]/2,0), (image.shape[1]/2,image.shape[0]), (255,255,255), 1)
        # cv2.line(image, (0,int(image.shape[0]*0.25)), (image.shape[1],\
        #  int(image.shape[0]*0.25)), (255,255,255), 1)

        # if self._writeVideo:
        # cv2.putText(image, str(self._avoidFrameId) ,(10,570), cv2.FONT_HERSHEY_PLAIN,\
        #  1, [255,255,255], 2)

#        filename = "/home/hamidreza/project_82/frames/4/file_%d.jpg"%self._it
#        cv2.imwrite(filename, image)

        # print('it --------------', self._it)

        # if self.sendPermission:
        cv2.imshow(mainUIName, image)
        # cv2.imshow('bin', bin)
        self._key = cv2.waitKey(1)
        # elif self._windowCreated:
        #     cv2.destroyAllWindows()
        #     self._windowCreated = False


    # def runIm(self):
    #     while True:
    #         res, image = self._cap.read()
    #         imageCopy = image.copy()
    #         self._it += 1
    #         # print(1)
    #         # img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #         # print (img[100,100])
    #         if self._key == 52:
    #             self._setThresh = not self._setThresh
    #
    #         nodes = self._extractor.extract(image)
    #         grid, nGrid = self._analyzer.analyze(image, nodes)
    #         # print(134)
    #         # _, _, _, _ = self._analyzer.command(image)
    #         # print(325)
    #         # cv2.line(image, (image.shape[1]/2,0), (image.shape[1]/2,image.shape[0]), (255,255,255), 1)
    #         # cv2.line(image, (0,int(image.shape[0]*0.25)), (image.shape[1],\
    #         #  int(image.shape[0]*0.25)), (255,255,255), 1)
    #         filename = "/home/hamidreza/project_82/frames/2/file_%d.jpg"%self._it
    #         cv2.imwrite(filename, image)
    #         # if self.sendPermission:
    #         cv2.imshow(mainUIName, image)
    #         # self._key = cv2.waitKey(1)
    #         if cv2.waitKey(100) & 0xFF == ord('q'):
    #             break
    #         # elif self._windowCreated:
    #         #     cv2.destroyAllWindows()
    #         #     self._windowCreated = False


rospy.init_node('pycvi', anonymous=True)

if MODE=="simulation":
    testNode = TestNode("/home/hamidreza/thesis/workSpace/src/warehouse_drone/visual_guidance/params/params.yaml",
                        "/home/hamidreza/thesis/workSpace/src/warehouse_drone/visual_guidance/params/calib1.yaml")
elif MODE=="video":
    testNode = TestNode("/home/hamidreza/thesis/workSpace/src/warehouse_drone/visual_guidance/params/params2.yaml",
                        "/home/hamidreza/thesis/workSpace/src/warehouse_drone/visual_guidance/params/telloCam.yaml")
    # testNode.runIm()
else:
    testNode = TestNode("/home/hamidreza/thesis/workSpace/src/warehouse_drone/visual_guidance/params/params2.yaml",
                        "/home/hamidreza/thesis/workSpace/src/warehouse_drone/visual_guidance/params/telloCam.yaml")

# if MODE=="simulation":
#     testNode = TestNode("params/params.yaml",
#                         "params/calib1.yaml")
# elif MODE=="video":
#     testNode = TestNode("params/params2.yaml",
#                         "params/telloCam.yaml")
#     # testNode.runIm()
# else:
#     testNode = TestNode("params/params2.yaml",
#                         "params/telloCam.yaml")


rospy.spin()
