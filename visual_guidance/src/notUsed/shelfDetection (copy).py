#!/usr/bin/env python

import time
import cv2 as cv
import numpy as np
import utility as utl
import yaml

class GridExtractor:


    def __init__(self, paramsFile):

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

        self._imageInfoSet = False
        self._kernelSize = (verticalKernelSize, horizontalKernelSize)
        self.init = False



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


    def extract(self, image):

        if not self._imageInfoSet: self.setImageInfo(image)
        # t1 = time.time()
        preProcessed = self.preProcess(image)
        # t2 = time.time()
        windows = self.extractContours(preProcessed, image)
        # t3 = time.time()
        # print 'prep time:  {:.05f}'.format(t3-t2)
        # print 'ext time:  {:.04f}'.format(t2-t1)
        return windows


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
        # imageThresholded = cv.erode(imageThresholded, np.ones((2,2),np.uint8))
        # print self._kernelSize
        # imageThresholded = cv.dilate(imageThresholded, np.ones((self._kernelSize[0],1),np.uint8))
        # imageThresholded = cv.dilate(imageThresholded, np.ones((1,self._kernelSize[1]),np.uint8))
        # imageThresholded = cv.medianBlur(imageThresholded, 7)
        cv.imshow("denoised", imageThresholded)
        return imageThresholded


    def extractContours(self, imageBin, image):

        # t1 = time.time()
        _, contours, hierarchy = cv.findContours(imageBin, cv.RETR_CCOMP , cv.CHAIN_APPROX_TC89_KCOS)
        nodes = []
        t2 = time.time()
        for idx, cnt in enumerate(contours):
            if hierarchy[0][idx][3]!=-1 :
                continue
            if not self.validateContour(cnt):
                continue
            center = utl.findWindowCenter(cnt)
            nodes.append(center)
            image = cv.circle(image,(center[0],center[1]),5,(255,255,255),-1)

        # print 'ex time:  {:.06f}'.format(t2-t1)
        # print 'ex loop time:  {:.06f}'.format(t3-t2)
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
