#!/usr/bin/env python

import cv2 as cv
import numpy as np
import utility as utl
import math
import yaml

class FrameAnalyzer:

    def __init__(self, paramsFile, controllerParams, trackerLink):

        self._paramsFile = paramsFile
        with open(paramsFile) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            self._windowWidth = 0.5 if not data.has_key("windowWidth") else data["windowWidth"]
            self._windowHeight = 0.5 if not data.has_key("windowHeight") else data["windowHeight"]
            self._darkSegmentThresh = 0.12 if not data.has_key("darkSegmentThresh") else data["darkSegmentThresh"]
            self._trackerCorrectionPixelOffset = 20 if not data.has_key("trackerCorrectionPixelOffset") else data["trackerCorrectionPixelOffset"]
            self._trackerInitPixelOffset = 10 if not data.has_key("trackerInitPixelOffset") else data["trackerInitPixelOffset"]
            self._autoVerticalizationAreaRatio = 0.01 if not data.has_key("autoVerticalizationAreaRatio") else data["autoVerticalizationAreaRatio"]
            self._borderPointsAutoFilterPixelOffset = 5 if not data.has_key("borderPointsAutoFilterPixelOffset") else data["borderPointsAutoFilterPixelOffset"]

        with open(controllerParams) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            self._V = 0.5 if not data.has_key("V") else data["V"]
            self._Py = -0.004 if not data.has_key("P_y") else data["P_y"]
            self._Pz = -0.008 if not data.has_key("P_z") else data["P_z"]


        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self._newRectSet = True
        self._trackerLink = trackerLink
        self.centroid = None
        self._simple = True
        self._maxWindowVal = None
        self._it = 0
        self.areaRatio = 0
        self._tuneWindowCreated = False
        self._tuneWindowName = 'Threshold Tuning'
        self._threshPercent = 100 * self._darkSegmentThresh
        self.thresholdTuning = False
        self.autoUpdateTracker = True
        self._lastMoments = []
        self._paramsReset = True
        self.darkExtractionPermission = False
        self.verticalIntersect = False
        self.horizontalIntersect = False
        self.verticalIntersectBefore = False
        self.horizontalIntersectBefore = False
        self.Vz = None
        self.Vy = None
        self._count = 0
        self._lastInfoSet = False

        self.setProjectionParams()


    def setCameraParams(self, cameraMatrix, distortionMatrix):

        self._cameraMatrix = cameraMatrix
        self._distortionMatrix = distortionMatrix


    def setProjectionParams(self):

        if not self._newRectSet:
            return

        self._realRectInfo = np.float32([ [0, 0, 0],
                                          [self._windowWidth, 0, 0],
                                          [self._windowWidth, self._windowHeight, 0],
                                          [0, self._windowHeight, 0]])
        self._axis3D = np.float32([ [0, 0, 0],
                                    [0.5, 0, 0],
                                    [0, 0.5, 0],
                                    [0, 0, 0.5]])
        self._newRectSet = False


    def setTemporaryProjectionParams(self, w, h):

        ar = np.float32(h)/w
        self._realRectInfo = np.float32([ [0, 0, 0],
                                          [1, 0, 0],
                                          [1, ar, 0],
                                          [0, ar, 0]])
        self._newRectSet = True


    def setData(self, contour=[], rvecs=None, br=None, setArea=False):

        if len(contour)!=0:
            self.centroid = utl.findWindowCenter(contour)
        elif br!=None:
            self.centroid = [br[0]+br[2]/2, br[1]+br[3]/2]
        else:
            self.centroid = None

        if setArea:
            if len(contour)!=0:
                self.areaRatio = np.float32(cv.contourArea(np.array(contour)))/self._trackerLink.imArea
            elif len(br)!=0:
                self.areaRatio = np.float32(br[2]*br[3])/self._trackerLink.imArea
            self._autoVerticalization = self.areaRatio>self._autoVerticalizationAreaRatio
        else:
            self._autoVerticalization = False
            self.areaRatio = 0

        if rvecs==None or not self._autoVerticalization:
            self.roll = 0
            self.pitch = 0
            self.yaw = 0
        else:
            self.roll = rvecs[0]
            self.pitch = rvecs[1]
            self.yaw = rvecs[2]


    def analyze(self, image, extractOpenPart, win=[], segment=[], boundingRect=()):

        # img = cv.cvtColor(image, cv.COLOR_BGR2HSV)[:,:,2]
        # print 'mean', np.mean(img)
        window = None
        if extractOpenPart:
            # print 'ana 1'
            window = self.extractDarkArea(segment)

            if window!=None:
                # print 'ana 2'
                self._window = window
                self._simple = False
                self.percept3D(image, True)

            elif self._trackerLink.trackingMode=="trmf" or self._trackerLink.trackingMode=="2step":
                # print 'ana 3'
                self._simple = True

            else:
                # print 'ana 4'
                self._simple = False
                self.setProjectionParams()
                self._window = utl.reconstructRect(win, True)
                self.percept3D(image, False)

        else:
            if len(segment)!=0 and self.thresholdTuning:
                self.tuneThreshold(segment)
            elif self._tuneWindowCreated:
                cv.destroyWindow(self._tuneWindowName)
                self._tuneWindowCreated = False

        if self._simple or not extractOpenPart:
            self.setData([], None, boundingRect, True)

        if window==None:
            self.resetParams()
        else:
            self.horizontalIntersectBefore = self.horizontalIntersect
            self.verticalIntersectBefore = self.verticalIntersect
            self.horizontalIntersect, self.verticalIntersect, self._intersectionData = utl.borderPointsCheck(window, self._trackerLink.cols, self._trackerLink.rows, True, 2)
            self.setLastInfo()
            # xc, yc = self.reconstructCentroid(window)
            # if xc!=None:
            #     print 'xc  ', xc
            #     self.centroid[0] = xc
            # elif yc!=None:
            #     print 'yc  ', yc, self.centroid[1]
            #     self.centroid[1] = yc


    def setLastInfo(self):

        # if self._lastVerInfoSet:
        #     return
        if self.translationVector[2][0]>3:
            return

        t_inv = np.float32(self._V)/self.translationVector[2][0]
        if (self.verticalIntersect and not (self.verticalIntersectBefore or self._lastInfoSet)) or\
         (self.horizontalIntersect and not (self.horizontalIntersectBefore or self._lastInfoSet)):
            print 'translationVector: ', self.translationVector
            print 't_inv ', t_inv
            setpointZ = (self.translationVector[1][0] - (np.float32(self._windowHeight)/2))
            setpointY = (self.translationVector[0][0] - (np.float32(self._windowWidth)/2))
            print 'sp', setpointZ, setpointY
            self.Vz = -setpointZ*t_inv
            self.Vy = setpointY*t_inv
            print 'vz ', self.Vz
            self._lastInfoSet = True
        # if self.horizontalIntersect and not (self.horizontalIntersectBefore or self._lastHorInfoSet):
        #     print 'translationVector: ', self.translationVector
        #     print 't ', t_inv
        #     self.Vy = -self.translationVector[0][0]*t_inv
        #     print 'vy ', self.Vy
        #     self._lastHorInfoSet = True

        # if self.horizontalIntersect and not self.horizontalIntersectBefore:



    def resetParams(self):
        print 'rp1'
        if not self._paramsReset:
            print 'rp2'
            self._lastMoments = []
            self._count = 0
            self.verticalIntersect = False
            self.horizontalIntersect = False
            self.verticalIntersectBefore = False
            self.horizontalIntersectBefore = False
            self._paramsReset = True


    def percept3D(self, img, setArea=False):

        if len(self._window)==0:
            self.setData()
            return

        win = np.array(self._window)
        detectedRect = np.float32([win[0][0], win[1][0], win[2][0], win[3][0]])
        _, rotationVec, translationVec = cv.solvePnP(self._realRectInfo, detectedRect, self._cameraMatrix, self._distortionMatrix)

        self.translationVector = utl.findRealTranslationNotThePnpOutput(translationVec, rotationVec)
        print 'translation Vector', self.translationVector
        print 'j Vector', translationVec


        axis2D, j = cv.projectPoints(self._axis3D, rotationVec, translationVec, self._cameraMatrix, self._distortionMatrix)

        cv.drawContours(img, [win], -1, (0,0,255), 2)
        # print 'rotation Vector', rotationVec

        rvecs = utl.eulerFromRodrigues(rotationVec)
        self.setData(self._window, rvecs, None, setArea)
        # print '\n\n\n\n\n\n\n\nrotation Vector - e', rvecs
        if self._autoVerticalization:
            cv.line(img, (axis2D[0][0][0], axis2D[0][0][1]), (axis2D[1][0][0],axis2D[1][0][1]), (255,255,0), 2)
            cv.line(img, (axis2D[0][0][0], axis2D[0][0][1]), (axis2D[2][0][0],axis2D[2][0][1]), (255,0,255), 2)
            cv.line(img, (axis2D[0][0][0], axis2D[0][0][1]), (axis2D[3][0][0],axis2D[3][0][1]), (0,255,255), 2)


    def tuneThreshold(self, image):

        if not self._tuneWindowCreated:
            cv.namedWindow(self._tuneWindowName)
            self._tuneWindowCreated = True

        cv.createTrackbar('threshold', self._tuneWindowName, np.int16(self._threshPercent), 60, self.onThreshChange)
        cv.imshow(self._tuneWindowName, self.preProccessSegment(image))


    def onThreshChange(self, value):

        self._threshPercent = value
        self._darkSegmentThresh = 0.01 * self._threshPercent
        self.onSliderChange()
    #

    def onSliderChange(self):

        with open(self._paramsFile, "r") as f:
            dict = yaml.load(f, Loader=yaml.FullLoader)

        dict["darkSegmentThresh"] = self._darkSegmentThresh

        with open(self._paramsFile, "w") as f:
            yaml.dump(dict, f)


    def preProccessSegment(self, segment):

        img = cv.cvtColor(segment[0], cv.COLOR_BGR2HSV)[:,:,2]
        minVal = np.min(img)
        maxVal = np.max(img)
        diff = maxVal - minVal
        # print 'mean', np.mean(img)
        offset = self._darkSegmentThresh * diff
        ret,imgThresholded = cv.threshold(img, minVal+offset, 255, cv.THRESH_BINARY_INV)
        imageThresholded = cv.erode(imgThresholded, np.ones((2,2),np.uint8))

        return imageThresholded


    def extractDarkArea(self, segment):

        self._it += 1
        # print len(segment)
        if not self.darkExtractionPermission:
            return None

        rows,cols,_ = segment[0].shape
        totalArea = rows*cols
        if totalArea==0:
            return None

        imageThresholded = self.preProccessSegment(segment)

        _, contours, hierarchy = cv.findContours(imageThresholded, cv.RETR_CCOMP , cv.CHAIN_APPROX_TC89_KCOS)
        if len(contours)==0:
            return None
        maxArea = -1
        i = None
        for idx, cnt in enumerate(contours):
            area = cv.contourArea(cnt)
            if area<30 or len(cnt)<=3:
                continue
            if area>maxArea:
                maxArea = area
                i = idx
        if i==None:
            return None
        poly = cv.approxPolyDP(contours[i],15, True)
        mu = cv.moments(contours[i])
        mxx = mu['mu20']
        myy = mu['mu02']
        mc = (mu['m10'] / (mu['m00'] + 1e-5), mu['m01'] / (mu['m00'] + 1e-5))
        ratio = np.max([mxx,myy])/np.min([mxx,myy])

        if len(self._lastMoments)<30:
            self._lastMoments.append(ratio)
        else:
            self._lastMoments.remove(self._lastMoments[0])
            self._lastMoments.append(ratio)

        stdDev = np.std(np.array(self._lastMoments))
        # print 'stdDev--------------', stdDev
        if stdDev>1:
            self.darkExtractionPermission = False
            return None

        self._paramsReset = False

        # print

        if len(poly)==1:
            return None
        win = utl.reconstructRect(poly, True)
        w = utl.calcEuclideanDistance(win[0][0], win[1][0])
        h = utl.calcEuclideanDistance(win[0][0], win[3][0])
        a, b, W, H = cv.boundingRect(win)

        # self.setTemporaryProjectionParams(w, h)

        openPart = [None]*4
        for i, pt in enumerate(win):
            openPart[i] = [[pt[0][0]+segment[1][0], pt[0][1]+segment[1][1]]]

        hor, ver, _ = utl.borderPointsCheck(win, cols, rows, True, self._trackerCorrectionPixelOffset)
        if hor or ver:
            if self._count<=31:
                self._count += 1
                print 'count ', self._count
            else:
                print '***** initTrackerMF *****'
                self._count = 0
                br = (a+segment[1][0]-self._trackerInitPixelOffset, b+segment[1][1]-self._trackerInitPixelOffset,\
                 W+(2*self._trackerInitPixelOffset), H+(2*self._trackerInitPixelOffset))
                self._trackerLink.initTrackerMF(br)
        else:
            self._count = 0

        cv.imshow('binary segment', imageThresholded)

        # filename = "/home/hamidreza/project_82/frames/4/file_%d.jpg"%self._it
        # cv.imwrite(filename, imageThresholded)
        return openPart


    def reconstructCentroid(self, sortedWindow):

        xc, yc = None, None
        coeff = 0.5
        if self.horizontalIntersect!=self.verticalIntersect:
            if self.verticalIntersect:

                upIntersect = None

                if (self._intersectionData[1][0] and self._intersectionData[1][1])\
                 and not (self._intersectionData[1][2] or self._intersectionData[1][3]):
                    upIntersect = False
                    width = utl.calcEuclideanDistance(sortedWindow[0][0],sortedWindow[1][0])
                elif (self._intersectionData[1][2] and self._intersectionData[1][3])\
                 and not (self._intersectionData[1][0] or self._intersectionData[1][1]):
                    upIntersect = True
                    width = utl.calcEuclideanDistance(sortedWindow[2][0],sortedWindow[3][0])
                elif (self._intersectionData[1][2] or self._intersectionData[1][3])\
                 and (self._intersectionData[1][0] or self._intersectionData[1][1]):
                    return xc, self._trackerLink.imCenter[1]
                else:
                    return xc, yc

                height = (np.float32(self._windowHeight)/self._windowWidth)*width

                if upIntersect:
                    yc = ((sortedWindow[0][0][1]+sortedWindow[1][0][1])/2)-(coeff*height)
                else:
                    yc = ((sortedWindow[2][0][1]+sortedWindow[3][0][1])/2)+(coeff*height)

            else:

                leftIntersect = None

                if (self._intersectionData[0][0] and self._intersectionData[0][3])\
                 and not (self._intersectionData[0][1] or self._intersectionData[0][2]):
                    leftIntersect = True
                    height = utl.calcEuclideanDistance(sortedWindow[1][0],sortedWindow[2][0])
                elif (self._intersectionData[0][1] and self._intersectionData[0][2])\
                 and not (self._intersectionData[0][0] or self._intersectionData[0][3]):
                    leftIntersect = False
                    height = utl.calcEuclideanDistance(sortedWindow[0][0],sortedWindow[3][0])
                elif (self._intersectionData[0][1] or self._intersectionData[0][2])\
                 and (self._intersectionData[0][0] or self._intersectionData[0][3]):
                    return self._trackerLink.imCenter[0], yc
                else:
                    return xc, yc

                width = (np.float32(self._windowWidth)/self._windowHeight)*height

                if leftIntersect:
                    xc = ((sortedWindow[1][0][0]+sortedWindow[2][0][0])/2)-(coeff*width)
                else:
                    xc = ((sortedWindow[0][0][0]+sortedWindow[3][0][0])/2)+(coeff*width)

        return xc, yc
