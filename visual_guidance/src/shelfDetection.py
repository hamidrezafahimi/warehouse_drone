#!/usr/bin/env python

import time
import cv2 as cv
import numpy as np
import utility as utl
import yaml

class GridExtractor:

    def __init__(self, paramsFile, show = False):

        with open(paramsFile) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            self._thresh = 30 if not data.has_key("thresh") else data["thresh"]
            self._contourRemovalThreshold = 100 if not data.has_key("contourRemovalThreshold") else data["contourRemovalThreshold"]
            horizontalKernelSize = 5 if not data.has_key("horizontalKernelSize") else data["horizontalKernelSize"]
            verticalKernelSize = 10 if not data.has_key("verticalKernelSize") else data["verticalKernelSize"]
            self._contourCornerPointsCheckTresh = 0.1 if not data.has_key("contourCornerPointsCheckTresh") else data["contourCornerPointsCheckTresh"]
            self._lowContourLenFilter = 3 if not data.has_key("lowContourLenFilter") else data["lowContourLenFilter"]
            self._lowContourAreaFilter = 30 if not data.has_key("lowContourAreaFilter") else data["lowContourAreaFilter"]
            self._hugeContourAreaRatioFilter = 0.8 if not data.has_key("hugeContourAreaRatioFilter") else data["hugeContourAreaRatioFilter"]
            self._nodeWidth = 0.075 if not data.has_key("nodeWidth") else data["nodeWidth"]
            self._nodeHeight = 0.075 if not data.has_key("nodeHeight") else data["nodeHeight"]

        self._imageInfoSet = False
        self._kernelSize = (verticalKernelSize, horizontalKernelSize)
        self.init = False
        self._show = show
        self._old = 0



    def setColorThresholds(self, lowH, lowS, lowV, highH, highS, highV):

        self._lowH = lowH
        self._lowS = lowS
        self._lowV = lowV
        self._highH = highH
        self._highS = highS
        self._highV = highV


    def setImageInfo(self, image):

        self._rows, self._cols,_ = image.shape
        self._imArea = self._rows*self._cols
        self._imageInfoSet = True


    def setCameraParams(self, cameraMatrix, distortionMatrix):

        self._cameraMatrix = cameraMatrix
        self._distortionMatrix = distortionMatrix
        self._realRectInfo = np.float32([ [0, 0, 0],
                                          [self._nodeWidth, 0, 0],
                                          [self._nodeWidth, self._nodeHeight, 0],
                                          [0, self._nodeHeight, 0]])


    def extract(self, image):

        if not self._imageInfoSet: self.setImageInfo(image)
        # t1 = time.time()
        preProcessed = self.preProcess(image)
        # t2 = time.time()
        nodes = self.extractContours(preProcessed, image)

        if self._show:
            cv.imshow("denoised", preProcessed)
        # self.validateNodesDistance(windows, image)
        # t3 = time.time()
        # print 'prep time:  {:.05f}'.format(t3-t2)
        # print 'ext time:  {:.04f}'.format(t2-t1)
        return nodes


    def preProcess(self, image):

        self._imageArea = image.shape[0]* image.shape[1]

        imageHSV = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        h,s,v = cv.split(imageHSV)
        v = cv.equalizeHist(v)
        imageHSV = cv.merge([h, s, v])

        imageThresholded = np.zeros(imageHSV.shape[:2], dtype=np.uint8)
        for i in range(len(self._lowS)):
            for idx in range(len(self._lowH[i])):
                # print (int(self._lowH[i][idx]), self._lowS[i], self._lowV[i]), (int(self._highH[i][idx]), self._highS[i], self._highV[i])
                th = cv.inRange(imageHSV, (int(self._lowH[i][idx]), self._lowS[i], self._lowV[i]),
                                          (int(self._highH[i][idx]), self._highS[i], self._highV[i]))
                imageThresholded = cv.bitwise_or(imageThresholded, th)

        # imageThresholded = cv.medianBlur(imageThresholded, 9)
        # imageThresholded = cv.blur(imageThresholded,(29,29))
        # ret,imageThresholded = cv.threshold(imageThresholded,30,255,cv.THRESH_BINARY)
        imageThresholded = cv.erode(imageThresholded, np.ones((2,2),np.uint8))
        # print self._kernelSize
        imageThresholded = cv.dilate(imageThresholded, np.ones((2,2),np.uint8))
        # imageThresholded = cv.dilate(imageThresholded, np.ones((self._kernelSize[0],1),np.uint8))
        # imageThresholded = cv.dilate(imageThresholded, np.ones((1,self._kernelSize[1]),np.uint8))
        # imageThresholded = cv.medianBlur(imageThresholded, 7)
        return imageThresholded


    def extractContours(self, imageBin, image):

        # print '----------------------------------------'
        # t1 = time.time()
        _, contours, hierarchy = cv.findContours(imageBin, cv.RETR_CCOMP , cv.CHAIN_APPROX_TC89_KCOS)
        windows = []
        t2 = time.time()

        miin = 1e6
        maax = -1

        for idx, cnt in enumerate(contours):
            if hierarchy[0][idx][3]!=-1 :
                continue
            if not self.validateContour(cnt):
                continue

            bbox = cv.boundingRect(cnt)

            rect = utl.reconstructRect(cnt, False)

            windows.append([rect, bbox])

            hor, ver, _ = utl.borderPointsCheck(rect, self._cols, self._rows, True)
        #     if not (hor or ver):
        #         area = cv.contourArea(cnt)
        #         if miin > area and area != 0:
        #             miin = area
        #         if maax < area:
        #             maax = area
        #
        # if miin != self._old:
        # #     print miin
        #     print miin/maax
        #     self._old = miin

        nodes = self.validateNodesDistance(windows)
        for nd in nodes:
        # for nd in windows:
            cv.rectangle(image, (nd[1][0], nd[1][1]), (nd[1][0]+nd[1][2], nd[1][1]+nd[1][3]), (0,255,0), 1)
            # cv.rectangle(imageBin, (nd[1][0], nd[1][1]), (nd[1][0]+nd[1][2], nd[1][1]+nd[1][3]), (0,255,0), 1)
            # cv.putText(image, str(idx),(bbox[0]+(bbox[2]/2), bbox[1]+(bbox[3]/2)+5), cv.FONT_HERSHEY_SIMPLEX, 0.25, (255,0,0), lineType=cv.LINE_AA)
        # t3 = time.time()
        # print 'ex time:  {:.06f}'.format(t2-t1)
        # print 'ex loop time:  {:.06f}'.format(t3-t2)
        # cv.drawContours(image, windows, -1, (0,255,0), 1)
        # return windows
        return nodes


    def validateContour(self, contour, cornerCheck=False, w=0, h=0):#, cornerCheck):

        if len(contour)<=self._lowContourLenFilter:
            # print 'len filtered'
            return False
        area = cv.contourArea(contour)
        if area<self._lowContourAreaFilter:
            # print 'low area filtered', area
            return False

        return True


    def validateNodesDistance(self, nodes, image=None):

        areas = [[None] for _ in range(len(nodes))]
        nds = []
        maxArea = -1
        for k, nd in enumerate(nodes):
            areas[k] = cv.contourArea(nd[0])
            if maxArea<areas[k]:
                maxArea = areas[k]
        k = 0
        # for k, nd in enumerate(nodes):

        for k, nd in enumerate(nodes):
            if (areas[k]/maxArea) < 0.25:
                continue
            nds.append(nd)
            # cv.putText(image, '{:.02f}'.format(areas[k]/maxArea), (nd[1][0], nd[1][1]), cv.FONT_HERSHEY_SIMPLEX, 0.25, (255,0,0), lineType=cv.LINE_AA)
        return nds


        # win = np.array(node)
        # detectedRect = np.float32([win[0][0], win[1][0], win[2][0], win[3][0]])
        # _, rotationVec, translationVec = cv.solvePnP(self._realRectInfo, detectedRect, self._cameraMatrix, self._distortionMatrix)
        # t = utl.findRealTranslationNotThePnpOutput(translationVec, rotationVec)
        # if t[2]>4:
        #     return False
        # # print cv.contourArea(node), idx
        # return True
