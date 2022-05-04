#!/usr/bin/env python

import cv2 as cv
import time
import numpy as np
import utility as utl
import math
import yaml

class GridAnalyzer:

    def __init__(self, paramsFile, extractor):

        with open(paramsFile) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            self._enteranceThresh = 0.4 if not data.has_key("enteranceThresh") else data["enteranceThresh"]
            self._verticalCenter = 0.23 if not data.has_key("verticalCenter") else data["verticalCenter"]
            self._segmentExtractionPixelOffset = 10 if not data.has_key("segmentExtractionPixelOffset") else data["segmentExtractionPixelOffset"]

        self.oldImage = []
        self.trackedBR = ()
        self.trackingMode = "manual"
        self.oldCentroid = None
        self.imageInfoSet = False
        self.imCenter = []
        self._windows = []
        self._contourSelected = False
        self.trackedContour = []
        self._trackedWindow = []
        self._centroidTrackOk = False
        self._optFlTrackOk = False
        self.trackingSucceed = False
        self.trackedContourIdx = 0
        self._trackerMF = None
        self._lk_params = dict( winSize  = (15,15),
                               maxLevel = 2,
                               criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
        self._matrixTrackOk = True
        self._frpl2 = False
        self._frpl1 = False
        self._fcpl2 = False
        self._fcpl1 = False
        self._index = []
        self._lims = []
        self._trackingInit = False
        self._lastTracked = []
        self._enteranceArea = None
        self._mode3FirstRect = None
        self.mode3SecondRect = None
        self.mode3SecondImage = None
        self.movePermission = False
        self.areaRatio = None
        self.trackedSegment = None
        self._extractor = extractor
        self.reset = False
        self._autoStep2 = False


    def selectTargetContour(self, x, y):

        print 'selectTargetContour-1'

        # if self.trackingMode == "2step" or self.trackingMode == "trmf" and not self.movePermission:
        #     self.movePermission = True
        #     return
        if not self.movePermission:
             self.movePermission = True

        if len(self._windows)==0:
            return
        print 'selectTargetContour-2'

        self.oldCentroid = [x,y]
        self._contourSelected = True
        self._trackingInit = False

        # if self.trackingmode=="tr":
        for idx, win in enumerate(self._windows):
            if cv.pointPolygonTest(win[0], (x,y), False)>=0:
                self.trackedContour = win[0]
                # self._trackedWindow = win
                self.trackedBR = win[1]
                print 'selectTargetContour-3'

        self._extractor.autoFilterAllow = True

        # if self.trackingMode=="2step" and
        print 'selectTargetContour-4', self.movePermission


    def setImageInfo(self, image):

        self.rows,self.cols,_ = image.shape
        xpc = self.cols/2 - 1
        ypc = self.rows*self._verticalCenter
        self.imCenter = [xpc, ypc]
        self.imArea = self.rows*self.cols
        self.imageInfoSet = True
        self._enteranceArea = self._enteranceThresh * self.imArea


    def setTrackingMode(self, mode):#, extractorLink):

        TRACKING_MODES = {
            1: "manual",
            2: "set",
            3: "auto",
        }
        if mode not in TRACKING_MODES:
            return

        self.trackingMode = TRACKING_MODES[mode]
        if mode!=0 or self._autoStep2:
            self.reset = True
            self.resetTracking()#self._extractor)
        # print mode==3 and not self._autoStep2
        if mode==2 or mode==3:
            self.movePermission = True


    def resetTracking(self):#, extractorLink):

        self._trackingInit = False
        self._trackerStarted = False
        self._mode3FirstRect = None
        self.mode3SecondRect = None
        self.mode3SecondImage = None
        self._centroidTrackOk = False
        self.movePermission = False
        self._optFlTrackOk = False
        self._matrixTrackOk = True
        self.trackingSucceed = False
        self._autoStep2 = False

        self._extractor.trackedAreaBefore = None
        self._extractor.trackedArea = None
        self._extractor.lastNormalArea = None
        self._extractor.init = None
        self._extractor.trackedAR = None
        self._extractor.trackedARBefore = None
        self._extractor.lastNormalAR = None
        self._extractor.autoFilterAllow = False


    def track(self, image, cleanImage=[], windows=[]):#, extractorLink=None):

        if not self.imageInfoSet:
            self.setImageInfo(image)
        self.trackingSucceed = False
        # if self.trackingmode==None: self.setTrackingMode(1)

        self._windows = windows

        if self.trackingMode=="manual":
            return

        elif self.trackingMode=="set":
            self.trTrack(image, 1)
            self.trackingSucceed = self._trTrackOk

        elif self.trackingMode=="auto":
            if self._trackingInit:
                self.centroidTrack(True)
            else:
                self.centroidTrack(False)
                self._trackingInit = True
            print self._centroidTrackOk, self.oldCentroid
            if self._centroidTrackOk:
                self.trackingSucceed = self._centroidTrackOk
                self._trackerStarted = False
            else:
                self.centroidTrack(False)
                if not (self._centroidTrackOk and self.checkTracking()):
                    self.oldCentroid = None
                    self.trTrack(image, 2)
                    self.trackingSucceed = self._trTrackOk
                else:
                    self.trackingSucceed = self._centroidTrackOk
                    self._trackerStarted = False

        else:
            print("Wrong tracking mode!")
            return

        self.oldImage = image.copy()
        if self.trackingSucceed:
            self._trackedArea = self.trackedBR[2]*self.trackedBR[3]
            self._extractor.setTrackedInfo(self._trackedArea, self.trackedBR[2], self.trackedBR[3])
            self.areaRatio = np.float32(self._trackedArea)/self.imArea
            self._trackingInit = True
            self._lastTracked = self.trackedBR
            br = np.int32(self.trackedBR)
            a = np.max([(br[0]-self._segmentExtractionPixelOffset),0])
            b = np.max([(br[1]-self._segmentExtractionPixelOffset),0])
            c = np.min([br[0]+br[2]+self._segmentExtractionPixelOffset,self.cols])
            d = np.min([br[1]+br[3]+self._segmentExtractionPixelOffset,self.rows])
            self.trackedSegment = [cleanImage[b:d, a:c, :], [a,b]]
            # cv.imshow('segment', self.trackedSegment[0])
        # else:
        #     self._lastTracked = []


    def checkTracking(self, image=[]):

        if not self._trackingInit:
            return True

        if self.trackingMode=="2step":
            if len(self._mode3FirstRect)!=0:
                print ((self.trackedBR[2]*self.trackedBR[3])/(self._mode3FirstRect[2]*self._mode3FirstRect[3]))
                if (self.trackedBR[2]*self.trackedBR[3])<(2*self._mode3FirstRect[2]*self._mode3FirstRect[3]):
                    self._autoStep2 = False
                    return True
                else:
                    self._mode3FirstRect = []
                    self.mode3SecondRect = np.int32(self.trackedBR)
                    self.mode3SecondImage = image.copy()
                    self._autoStep2 = True
                    return False

        elif self.trackingMode=="fullAuto":
            if len(self.trackedContour)==0 and self.oldCentroid==None:
                return False
            print self.trackedBR
            print self._lastTracked
            if (self.trackedBR[2]*self.trackedBR[3])>(1.8*self._lastTracked[2]*self._lastTracked[3]):# or cv.contourArea(self.trackedContour)<(0.4*self._lastTracked[2]*self._lastTracked[3]):
                return False
            motionVector = [abs(self.oldCentroid[0]-(self._lastTracked[0]+(self._lastTracked[2]/2))),
                            abs(self.oldCentroid[1]-(self._lastTracked[1]+(self._lastTracked[3]/2)))]
            # print 'vt report: ', motionVector, self._lastTracked
            return (motionVector[0]<(1.5*self._lastTracked[2]) and motionVector[1]<(1.5*self._lastTracked[3]))


    def trTrack(self, image, mode):

        # print 'trt in'
        self._trTrackOk = False

        if mode==1:

            # self.trackedBR = []

            success, newBR = self._trackerMF.update(image)
            if not success:
                print 'Tracker mf Failed!!!'
                return

            # self._trackedArea = self.trackedBR[2]*self.trackedBR[3]
            # self.areaRatio = self._trackedArea/self.imArea

        elif mode==2:
            # print 'trt 1'
            print len(self.oldImage)
            print self._lastTracked
            if len(self.oldImage)==0 or len(self._lastTracked)==0:
                self._trackerStarted = False
                return
            # print 'trt 2'

            if not self._trackerStarted:
                # print 'trt 3'
                self.initTrackerMF(self._lastTracked)

            success, newBR = self._trackerMF.update(image)
            if not success:
                # print 'trt 4'
                return

            self.trackedContour = utl.brToContour(newBR)

        self.oldCentroid = [newBR[0]+(newBR[2]/2), newBR[1]+(newBR[3]/2)]
        self.trackedBR = newBR
        self._trTrackOk = True
        # print 'trt out'
        return


    def initTrackerMF(self, target, image=[]):

        self._trackerMF = None
        self._trackerMF = cv.TrackerMedianFlow_create()

        # print '############  initTrackerMF  #############'

        if self.trackingMode=="2step" and not self._trackingInit:
            self._mode3FirstRect = target

        if len(image)!=0:
            img = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
            self._trackerMF.init(image, target)
            self._trackerStarted = True

        else:
            if len(self.oldImage)==0:
                return
            self._trackerMF.init(cv.cvtColor(self.oldImage, cv.COLOR_BGR2GRAY), target)
            self._trackerStarted = True


    def trFindAgain(self, image, init=False, target=None, initImage=None):

        if init:
            self.initTrackerMF(target, initImage)

        if self._trackerMF==None: return False, None

        success, newBR = self._trackerMF.update(image)
        if not success: return False, None

        self._trackerStarted = False

        return True, np.int32(newBR);


    def centroidTrack(self, findSurrounding):

        self._centroidTrackOk = False
        self.trackedContour = []
        self.trackedBR = ()
        if self.oldCentroid==None:
            return

        lwds = len(self._windows)
        if lwds==0:
            return

        if findSurrounding:
            if self.trackedContourIdx<lwds:
                idx = self.trackedContourIdx
            else: idx = 0
            self.trackedContourIdx = 0
            i=0
            while i<lwds:
                center = [self._windows[idx][1][0]+(self._windows[idx][1][2]/2), self._windows[idx][1][1]+(self._windows[idx][1][3]/2)]
                w = self._windows[idx][1][2]
                h = self._windows[idx][1][3]
                conX = abs(center[0]-self.oldCentroid[0])<=w/2
                conY = abs(center[1]-self.oldCentroid[1])<=h/2
                if conX and conY:
                    self._centroidTrackOk = True
                    self.oldCentroid = center
                    self.trackedContour = self._windows[idx][0]
                    self.trackedBR = self._windows[idx][1]
                    self.trackedContourIdx = idx
                    break
                idx += 1
                if idx==lwds: idx = 0
                i += 1
        else:
            minDist = 1e9
            for i, win in enumerate(self._windows):
                p = [win[1][0]+(win[1][2]/2), win[1][1]+(win[1][3]/2)]
                dist = utl.calcEuclideanDistance(p, self.oldCentroid)
                if dist < minDist:
                    minDist = dist
                    self.trackedContourIdx = i
                    newCentroid = p
            self._centroidTrackOk = True
            self.oldCentroid = newCentroid
            self.trackedContour = self._windows[self.trackedContourIdx][0]
            self.trackedBR = self._windows[self.trackedContourIdx][1]
            print self.trackedBR
            # print self.trackedContour
        # t2 = time.time()
        # print('coa time {:.6f}'.format(t2-t1))

    def optFlTrack(self, image):
        # print 'of in'
        # t1 = time.time()
        self._optFlTrackOk = False
        # if self._contourSelected: self._contourSelected = False
        # if len(self._contourSelected) > 0:
        #     self.oldCentroid = utl.findWindowCenter(self._contourSelected)
        #     self.trackedContour = self._contourSelected
        #     self._contourSelected = []
        #     return
        if self.oldCentroid==None or len(self.trackedContour)==0:
            return

        newImage = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        wp = utl.duplicateContour(utl.brToContour(self._lastTracked))

        wp = np.array(wp, dtype=np.float32)
        wp = [point[0] for point in wp]
        # print('in of: ', wp)
        wp_new, st, err = cv.calcOpticalFlowPyrLK(cv.cvtColor(self.oldImage, cv.COLOR_BGR2GRAY), newImage, np.float32(wp), None, **self._lk_params)
        # wp_new, st, err = cv.calcOpticalFlowPyrLK(cv.cvtColor(self.oldImage, cv.COLOR_BGR2GRAY), newImage, np.float32(wp), None,  winSize  = (15,15),
        #                        maxLevel = 2,
        #                        criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

        k = 0
        while k < len(wp_new):
            if err[k]>6:
                wp_new = np.delete(wp_new,k,axis=0)
                err = np.delete(err,k,axis=0)
                k-=1
            k+=1

        # wp_new = [[[int(el[0][0]),int(el[0][1])]] for el in wp_new]
        wp_new = [[[int(el[0]),int(el[1])]] for el in wp_new]

        if len(wp_new)==0: return
        self.oldCentroid = utl.findWindowCenter(wp_new)
        self.centroidTrack(True)
        if self._centroidTrackOk or len(wp_new)==1: return
        self.trackedContour = utl.reconstructRect(wp_new, False)
        self.trackedBR = cv.boundingRect(self.trackedContour)
        self._optFlTrackOk = True

        # t2 = time.time()
        # print('of time {:.6f}'.format(t2-t1))

    def findNearest(self, windowsInfo, win, xlim, ylim):
        # Each element of windowsInfo, consists of index, center x and y and the contour of the window
        minx = miny = 1e7
        nearestX_y = nearestY_x = tempX_y = tempY_x = 1e7
        # farest_x = farest_y = -1
        wins = windowsInfo

        row = []
        col = []

        _,_, a, b = cv.boundingRect(win[3])
        if xlim==0 or ylim==0:
            if xlim==0: xlim = (a*b)/math.sqrt((4*b*b)+(a*a))
            if ylim==0: ylim = (a*b)/math.sqrt((b*b)+(4*a*a))

        for k, wn in enumerate(wins):
            distX = abs(win[1]-wn[1])
            # distX = wn[1]-win[1]
            distY = abs(win[2]-wn[2])
            # distY = wn[2]-win[2]
            if distX==0 and distY==0:
                continue
            if distX<xlim and distY>ylim and win[2]<wn[2]:# and distY>0:
                if nearestX_y>distY:
                    nearestX_y = distY
                col.append([k, distY, distX])#, wn[2]])
            if distY<ylim and distX>xlim and win[1]<wn[1]:# and distX>0:
                if nearestY_x>distX:
                    nearestY_x = distX
                row.append([k, distX, distY])#, wn[1]])

        row.sort(key=lambda x: x[1])

        k = 0
        lr = len(row)
        while k<(lr-1):
            # if abs(row[k][3]-row[k+1][3])<(xlim/2):
            if abs(row[k][1]-row[k+1][1])<(xlim/2):
                if row[k][2]<=row[k+1][2]:
                    row.remove(row[k+1])
                else: row.remove(row[k])
            lr = len(row)
            k += 1

        col.sort(key=lambda x: x[1])

        k = 0
        lc = len(col)
        while k<(lc-1):
            # if abs(col[k][3]-col[k+1][3])<(ylim/2):
            if abs(col[k][1]-col[k+1][1])<(ylim/2):
                if col[k][2]<=col[k+1][2]:
                    col.remove(col[k+1])
                else: col.remove(col[k])
            lc = len(col)
            k += 1

        if nearestX_y<b or nearestY_x<a or nearestX_y==1e7 or nearestY_x==1e7:
            if nearestX_y<b or nearestX_y==1e7:
                nearestX_y = b
            if nearestY_x<a or nearestY_x==1e7:
                nearestY_x = a

        return nearestX_y, nearestY_x, row, col;

    def matrixTrack(self, image):

        t1 = time.time()

        # if len(self._contourSelected) > 0:
        #     print 'mat-1'
        #     self.oldCentroid = utl.findWindowCenter(self._contourSelected)
        #     input = self._contourSelected[:]
        #     self._contourSelected = []
        # else:
        #     print 'mat-2'
        #     input = self._index

        if self._contourSelected or not self._index:
            # print 'mat-1'
            # self.oldCentroid = utl.findWindowCenter(self._contourSelected)
            # input = self._contourSelected[:]
            input = [0,0,0,0]
            self._contourSelected = False
        else:
            # print 'mat-2'
            input = self._index

        # print 'mat-input   ', input
        # if len(input)==0: return
        index_in = len(input)==2
        window_in = len(input)==4
        # print 4

        if not (index_in or window_in):
            self.trackedContour = []
            self.trackedBR = ()
            self.oldCentroid = []
            print("Wrong input to windowIndex!")
            return [0]

        winTable = [[None]*9 for _ in range(len(self._windows))]
        for i, poly in enumerate(self._windows):
            # [x, y] = utl.findWindowCenter(poly)
            x = poly[1][0]+(poly[1][2]/2)
            y = poly[1][1]+(poly[1][3]/2)
            winTable[i]=[i, x, y, np.int32(poly[0]), None, None, None, None, poly[1]]
            #index, centroid, ..., contour, sameRows info, sameCols info, row number, col number

        b = a = 1e7
        dists = [None]*len(winTable)

        if len(self._lims)==0:
            for k, wn in enumerate(winTable):
                # bl, al,  wn[4], wn[5] = self.findNearest(winTable, wn, self.trackedBR[2], self.trackedBR[3])
                bl, al,  wn[4], wn[5] = self.findNearest(winTable, wn, wn[8][2], wn[8][3])
                if al<wn[8][2]: al=wn[8][2]
                if bl<wn[8][3]: bl=wn[8][3]
                if b>bl: b = bl
                if a>al: a = al
                # dists[k] = [al, bl, wn, k]
        else:
            for k, wn in enumerate(winTable):
                bl, al, wn[4], wn[5] = self.findNearest(winTable, wn, self._lims[0], self._lims[1])
                if al<wn[8][2]: al=wn[8][2]
                if bl<wn[8][3]: bl=wn[8][3]
                if b>bl: b = bl
                if a>al: a = al
                # dists[k] = [al, bl, wn, k]

        # for el in dists:
        #     if abs(el[0])>(2*a) or abs(el[1])>(2*b):
        #         if el[2] in winTable:
        #             winTable[dists[k][3]] = [None]

        # if len(self.trackedBR)!=0:
        #     if a<self.trackedBR[2]:
        #         a = self.trackedBR[2]
        #     if b<self.trackedBR[3]:
        #         b = self.trackedBR[3]
        # else:
        #     if a<15:
        #         a = 15
        #     if b<15:
        #         b = 15

        numWin = len(winTable)
        if numWin==0:
            print('Low number of contours to windowIndex!')
            self.trackedContour = []
            self.trackedBR = ()
            self.oldCentroid = []
            return [1]
        #
        ylim = 2*(a*b)/math.sqrt((b*b)+(4*a*a))
        xlim = 2*(a*b)/math.sqrt((4*b*b)+(a*a))
        self._lims = [xlim,ylim]

        # print 'lims', xlim,ylim,a,b

        winTsorted = []

        for k,el in enumerate(winTable):
            if len(el)<=1: continue
            winTsorted.append(el)

        winTsorted.sort(key=lambda x: x[1])

        i=0
        j=0
        k=1
        indexBox = []
        ies=[]
        tempWin = winTsorted[0]
        while k<=len(winTsorted):
            # print 'mat-3'#,tempWin[0], i
            if len(tempWin)!=0: print tempWin[0], tempWin[4], tempWin[6], i
            elif len(tempWin)==1:
                tempWin = winTsorted[k]
                k+=1
                continue
            # else: print 'zero'
            if len(indexBox)==numWin: break
            if len(tempWin)==0:
                # print 'e', k
                i+=1
                if k<len(winTsorted): tempWin = winTsorted[k]
                # tempWin = winTsorted[k]
                k+=1
                continue
            if tempWin[0] in indexBox:
                # print 'f', k
                if k<len(winTsorted): tempWin = winTsorted[k]
                # tempWin = winTsorted[k]
                k+=1
                continue
            else:
                indexBox.append(tempWin[0])
            if tempWin[6]==None:
                tempWin[6] = i
                winTable[tempWin[0]][6] = i
                # print 'tmp',tempWin
                # print 'wnt',winTable[tempWin[0]]
            # if len(tempWin)!=0: print tempWin[0],tempWin[6]
            # else: print 'zero'
            if len(tempWin[4])==0:
                ies.append([tempWin[6],tempWin[2]])
                tempWin = []
                continue
            ii=0
            while ii<len(tempWin[4]):
                if len(winTable[tempWin[4][ii][0]])<=1:
                    # print 'd'
                    ii+=1
                    continue
                if winTable[tempWin[4][ii][0]][6]!=None:
                    ii+=1
                    # print 'a'
                    if ii==len(tempWin[4]):
                        ies.append([tempWin[6],tempWin[2]])
                        tempWin = []
                        break
                    continue
                if (winTable[tempWin[4][ii][0]][1]-tempWin[1])>0:
                    winTable[tempWin[4][ii][0]][6] = tempWin[6]
                    tempWin = winTable[tempWin[4][ii][0]]
                    # print 'b'
                    break
                else:
                    ies.append([tempWin[6],tempWin[2]])
                    tempWin = []
                    # print 'c'
                    break

        k=0
        indexBox = []
        jes=[]
        winTsorted.sort(key=lambda x: x[2])
        tempWin = winTsorted[0]
        while k<=len(winTsorted):
            # print 'mat-4'#,tempWin[0], i
            if len(tempWin)!=0: print tempWin[0],tempWin[5], j
            elif len(tempWin)==1:
                k+=1
                continue
            if len(indexBox)==numWin: break
            if len(tempWin)==0:
                j += 1
                if k<len(winTsorted): tempWin = winTsorted[k]
                k+=1
                # print 'e'
                continue
            if tempWin[0] in indexBox:
                if k<len(winTsorted): tempWin = winTsorted[k]
                k+=1
                # print 'f'
                continue
            else:
                indexBox.append(tempWin[0])
            if tempWin[7]==None:
                tempWin[7] = j
                winTable[tempWin[0]][7] = j
            ii=0
            if len(tempWin[5])==0:
                jes.append([tempWin[7],tempWin[1]])
                tempWin = []
                continue
            while ii<len(tempWin[5]):
                if len(winTable[tempWin[5][ii][0]])<=1:
                    # print 'd'
                    ii+=1
                    continue
                if winTable[tempWin[5][ii][0]][7]!=None:
                    ii+=1
                    # print 'a'
                    if ii==len(tempWin[5]):
                        jes.append([tempWin[7],tempWin[1]])
                        tempWin = []
                        break
                    continue
                if (winTable[tempWin[5][ii][0]][2]-tempWin[2])>0:
                    # print 'b'
                    winTable[tempWin[5][ii][0]][7] = tempWin[7]
                    tempWin = winTable[tempWin[5][ii][0]]
                    break
                else:
                    # print 'c'
                    jes.append([tempWin[7],tempWin[1]])
                    tempWin = []
                    break


        ies.sort(key=lambda x: x[1])
        ies = ies[::-1]
        # print 'ie',ies
        iess = {k[0]:kk for kk,k in enumerate(ies)}
        jes.sort(key=lambda x: x[1])
        jess = {k[0]:kk for kk,k in enumerate(jes)}
        for wn in winTable:
            if len(wn)<=1: continue
            wn[6] = iess.get(wn[6])
            wn[7] = jess.get(wn[7])

        # t3 = time.time()
        # print('culstering time :', t3-t1)

        FRPL1 = False
        FRPL2 = False
        FCPL1 = False
        FCPL2 = False

        for wn in winTable:
            if len(wn)<=1: continue
            if wn[6]==0 and wn[2]<(ylim/2):
                FRPL2 = True
                self._matrixTrackOk = False
            if wn[6]==0 and wn[2]<(1.5*ylim):
                FRPL1 = True
                self._matrixTrackOk = False
            if wn[7]==0 and wn[1]<(xlim/2):
                FRPL2 = True
                self._matrixTrackOk = False
            if wn[7]==0 and wn[1]<(1.5*ylim):
                FRPL2 = True
                self._matrixTrackOk = False
        #
        #
        if FRPL2:
            for wn in winTable:
                if len(wn)<=1: continue
                if wn[6]==0: winTable.remove(wn)
            for wn in winTable:
                if len(wn)<=1: continue
                if wn[6]==None: continue
                wn[6] = wn[6]-1
            if self._frpl1 and not self._frpl2:
                # print(3)
                if len(self._index)!=0: self._index[0] = self._index[0]-1
        elif self._frpl2 and not FRPL2:
            # print(2)
            if FRPL1:
                # print(4)
                if len(self._index)!=0: self._index[0] = self._index[0]+1

        if FCPL2:
            for wn in winTable:
                if len(wn)<=1: continue
                if wn[7]==0: winTable.remove(wn)
            for wn in winTable:
                if len(wn)<=1: continue
                if wn[7]==None: continue
                wn[7] = wn[7]-1
            if self._fcpl1 and not self._fcpl2:
                # print(5)
                if len(self._index)!=0: self._index[1] = self._index[1]-1
        elif self._fcpl2 and not FCPL2:
            # print(6)
            if FCPL1:
                # print(7)
                if len(self._index)!=0: self._index[1] = self._index[1]+1
        #
        for k, wn in enumerate(winTable):
            if len(wn)<=1:
                # cv.putText(image, str(wn[0]),(wn[1]-10,wn[2]), cv.FONT_HERSHEY_SIMPLEX, 0.25, (0,255,255), lineType=cv.LINE_AA)
                continue
            cv.putText(image, str([wn[6], wn[7]]),(wn[1]-10,wn[2]), cv.FONT_HERSHEY_SIMPLEX, 0.25, (0,255,255), lineType=cv.LINE_AA)
            cv.putText(image, str([wn[0]]),(wn[1]+150,wn[2]), cv.FONT_HERSHEY_SIMPLEX, 0.25, (0,255,0), lineType=cv.LINE_AA)


        # cv.imwrite('lk.jpg',image)

        self._frpl2 = FRPL2
        self._frpl1 = FRPL1
        self._fcpl2 = FCPL2
        self._fcpl1 = FCPL1
        #
        t2 = time.time()
        # print('mat time {:.6f}'.format(t2-t1))

        if index_in:
            for wn in winTable:
                if len(wn)<=1: continue
                if wn[6]==self._index[0] and wn[7]==self._index[1]:
                    self.trackedContour = wn[3]
                    self.trackedBR = wn[8]
                    self.oldCentroid = [wn[1],wn[2]]
                    return
            print('Wrong index to windowIndex!')
            self.trackedContour = []
        elif window_in:
            minDist = 1e7
            # self.centroidTrack(False)
            for wn in winTable:
                if len(wn)<=1: continue
                dist = utl.calcEuclideanDistance([wn[1],wn[2]], self.oldCentroid)
                if minDist>dist:
                    minDist = dist
                    self._index = [wn[6], wn[7]]
                    cent = [wn[1],wn[2]]
                    self.trackedContour = wn[3]
                    self.trackedBR = wn[8]
            self.oldCentroid = cent
            return
                # if abs(self.oldCentroid[0]-wn[1])>(2*a) and abs(self.oldCentroid[1]-wn[2])>(3*b/2):
                # if abs(self.oldCentroid[0]-wn[1])<a/2 and abs(self.oldCentroid[1]-wn[2])<b/2:
                # if abs(self.oldCentroid[0]-wn[1])<a and abs(self.oldCentroid[1]-wn[2])<b:
                #     self.trackedContour = wn[3]
                #     self._index = [wn[6], wn[7]]
                #     return
            self.oldCentroid = []
            self.trackedContour = []
            self.trackedBR = ()
            print('Wrong contour to windowIndex!', a, b)
        return
