#!/usr/bin/env python

import cv2 as cv
import time
import math
import numpy as np
import utility as utl
import yaml
import windowTracking as wt
import windowsExtraction as we
import windowFrameAnalysis as wfa



class MovingAverageFilter:

    def __init__(self, windowSize):
        self._windowSize = windowSize
        self._window = []

    def update(self, newValue):

        if len(self._window) >= self._windowSize:
            self._window.remove(self._window[0])

        self._window.append(newValue)

        return sum(self._window) / len(self._window)



class MovingAverageFilter2D:

    def __init__(self, windowSize):
        self._windowSize = windowSize
        self._window = []

    def update(self, newValue):

        self._window.append(newValue)

        if len(self._window) >= self._windowSize:
            self._window.remove(self._window[0])
            xMean = sum(p[0] for p in self._window) / len(self._window)
            yMean = sum(p[1] for p in self._window) / len(self._window)

            return [xMean, yMean]
        else:
            return newValue



class WindowDetector:

    def __init__(self, paramsFile):

        with open(paramsFile) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            self._closenessAreaRatio = 0.01 if not data.has_key("closenessAreaRatio") else data["closenessAreaRatio"]

        self._lastEndTime = None
        self._FPSMAFilter = MovingAverageFilter(10)
        self._DTMAFilter = MovingAverageFilter(10)
        self._winCenterFilter = MovingAverageFilter2D(10)
        self._colorThresholdsSet = False
        self._colorBasedDetection = False
        self._it = 0
        # self.darkExtractionPermission = False

        # self._colorDetector = acd.AutoColorDetector(paramsFile)
        self.extractor = we.ColorBasedWindowsExtractor(paramsFile)
        self.tracker = wt.WindowTracker(paramsFile, self.extractor)
        # self._windowOptimizer = wo.WindowOptimizer(15, 20, 15, 1, refinementInterval=1)
        self.analyzer = wfa.FrameAnalyzer(paramsFile, "../params/controlParams.yaml", self.tracker)
        # self.controller = dc.DynamicControl("../params/controlParams.yaml", self.tracker, self.analyzer)


    def detect(self, image):

        startTime = time.time()
        self._it+=1
        print '------------------------------------------------------- it: ', self._it
        cleanImage = image.copy()
        self._closeEnough = False

        # print self._colorThresholdsSet

        if self.tracker.trackingMode=="manual" or self.tracker.trackingMode=="set":
            pass

        elif self.tracker.trackingMode=="auto":
            windows = self.extractor.extract(image)
            self.tracker.track(image, cleanImage, windows)#, self.extractor)
            # print 'e1'
        else:
            print "No object is detected yet."

        if self.tracker.trackingSucceed:
            # print 'det if'
            bbox = np.int32(self.tracker.trackedBR)
            cv.rectangle(image, (bbox[0], bbox[1]), (bbox[0]+bbox[2], bbox[1]+bbox[3]), (255,0,0), 2)
            if self.tracker.trackingMode=="2step" or not self.analyzer.darkExtractionPermission:
                self._closeEnough = False
            elif self.tracker.areaRatio > self._closenessAreaRatio:
                self._closeEnough = True

        #     self.analyzer.analyze(image, self._closeEnough, self.tracker.trackedContour, self.tracker.trackedSegment, self.tracker.trackedBR)
        #
        # else:
        #     self.analyzer.resetParams()


        # FPS and calculation time
        endTime = time.time()
        if self._lastEndTime != None:

            FPS = self._FPSMAFilter.update( 1. / (endTime - self._lastEndTime) )
            dt = self._DTMAFilter.update( endTime - startTime )
            cv.putText(image,"Calculation Time: {:.03f} ms   FPS: {:.0f}".format(dt, FPS) ,(20,20), cv.FONT_HERSHEY_PLAIN, 1, [255,255,255], 2)
            print "Calculation Time: {:.03f} ms   FPS: {:.0f}".format(dt, FPS)

        self._lastEndTime = endTime
