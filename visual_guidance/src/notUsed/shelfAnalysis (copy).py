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

        self._frpl2 = False
        self._frpl1 = False
        self._fcpl2 = False
        self._fcpl1 = False
        self.trackingMode = "manual"
        self.imageInfoSet = False
        self.imCenter = []
        self._lims = []
        self._extractor = extractor
        self._initLim = 5


    def setImageInfo(self, image):

        self.rows,self.cols,_ = image.shape
        xpc = self.cols/2 - 1
        ypc = self.rows*self._verticalCenter
        self.imCenter = [xpc, ypc]
        self.imArea = self.rows*self.cols
        self.imageInfoSet = True
        self._enteranceArea = self._enteranceThresh * self.imArea


    def analyze(self, image, nodes):

        self._nodes = nodes
        self.matrixTrack(image)



    def findNearest(self, windowsInfo, win, xlim, ylim):
        # Each element of windowsInfo, consists of index, center x and y and the contour of the window
        minx = miny = 1e7
        nearestX_y = nearestY_x = tempX_y = tempY_x = 1e7
        # farest_x = farest_y = -1
        wins = windowsInfo

        row = []
        col = []

        # _,_, a, b = cv.boundingRect(win[3])
        if xlim==0 or ylim==0:
            if xlim==0:
                xlim = self._initLim
            if ylim==0:
                ylim = self._initLim

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
                else:
                    row.remove(row[k])
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

        if nearestX_y<ylim or nearestY_x<xlim or nearestX_y==1e7 or nearestY_x==1e7:
            if nearestX_y<ylim or nearestX_y==1e7:
                nearestX_y = ylim
            if nearestY_x<xlim or nearestY_x==1e7:
                nearestY_x = xlim

        return nearestX_y, nearestY_x, row, col;


    def matrixTrack(self, image):

        t1 = time.time()

        winTable = [[None]*8 for _ in range(len(self._nodes))]
        for i, node in enumerate(self._nodes):
            # [x, y] = utl.findWindowCenter(poly)
            # x = poly[1][0]+(poly[1][2]/2)
            # y = poly[1][1]+(poly[1][3]/2)
            winTable[i] = [i, node[0], node[1], node, None, None, None, None]
            #index, centroid, ..., contour, sameRows info, sameCols info, row number, col number

        b = a = 1e7
        dists = [None]*len(winTable)

        if len(self._lims)==0:
            for k, wn in enumerate(winTable):
                # bl, al,  wn[4], wn[5] = self.findNearest(winTable, wn, self.trackedBR[2], self.trackedBR[3])
                print wn
                # print wn[8]
                bl, al,  wn[4], wn[5] = self.findNearest(winTable, wn, 0, 0)
                print al, bl
                if al<self._initLim:
                    al = self._initLim
                if bl<self._initLim:
                    bl = self._initLim
                if b>bl:
                    b = bl
                if a>al:
                    a = al
                # dists[k] = [al, bl, wn, k]
        else:
            for k, wn in enumerate(winTable):
                bl, al, wn[4], wn[5] = self.findNearest(winTable, wn, self._lims[0], self._lims[1])
                if al<(self._lims[0]-5):
                    al = (self._lims[0]-5)
                if bl<(self._lims[1]-5):
                    bl = (self._lims[1]-5)
                if b>bl:
                    b = bl
                if a>al:
                    a = al
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

        # print a, b

        numWin = len(winTable)
        if numWin==0:
            print('Low number of contours to windowIndex!')
            self.trackedContour = []
            self.trackedBR = ()
            self.oldCentroid = []
            return [1]
        #
        ylim = (a*b)/math.sqrt((b*b)+(4*a*a))
        xlim = (a*b)/math.sqrt((4*b*b)+(a*a))
        self._lims = [xlim,ylim]

        # print self._lims

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
            if len(tempWin)!=0:
                pass
                # print tempWin[0], tempWin[4], tempWin[6], i
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
            if len(tempWin)!=0:
                pass
                # print tempWin[0],tempWin[5], j
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

        # FRPL1 = False
        # FRPL2 = False
        # FCPL1 = False
        # FCPL2 = False
        #
        # for wn in winTable:
        #     if len(wn)<=1: continue
        #     if wn[6]==0 and wn[2]<(ylim/2):
        #         FRPL2 = True
        #         self._matrixTrackOk = False
        #     if wn[6]==0 and wn[2]<(1.5*ylim):
        #         FRPL1 = True
        #         self._matrixTrackOk = False
        #     if wn[7]==0 and wn[1]<(xlim/2):
        #         FRPL2 = True
        #         self._matrixTrackOk = False
        #     if wn[7]==0 and wn[1]<(1.5*ylim):
        #         FRPL2 = True
        #         self._matrixTrackOk = False
        # #
        # #
        # if FRPL2:
        #     for wn in winTable:
        #         if len(wn)<=1: continue
        #         if wn[6]==0: winTable.remove(wn)
        #     for wn in winTable:
        #         if len(wn)<=1: continue
        #         if wn[6]==None: continue
        #         wn[6] = wn[6]-1
        #     if self._frpl1 and not self._frpl2:
        #         # print(3)
        #         if len(self._index)!=0: self._index[0] = self._index[0]-1
        # elif self._frpl2 and not FRPL2:
        #     # print(2)
        #     if FRPL1:
        #         # print(4)
        #         if len(self._index)!=0: self._index[0] = self._index[0]+1
        #
        # if FCPL2:
        #     for wn in winTable:
        #         if len(wn)<=1: continue
        #         if wn[7]==0: winTable.remove(wn)
        #     for wn in winTable:
        #         if len(wn)<=1: continue
        #         if wn[7]==None: continue
        #         wn[7] = wn[7]-1
        #     if self._fcpl1 and not self._fcpl2:
        #         # print(5)
        #         if len(self._index)!=0: self._index[1] = self._index[1]-1
        # elif self._fcpl2 and not FCPL2:
        #     # print(6)
        #     if FCPL1:
        #         # print(7)
        #         if len(self._index)!=0: self._index[1] = self._index[1]+1
        #

        for k, wn in enumerate(winTable):
            if len(wn)<=1:
                # cv.putText(image, str(wn[0]),(wn[1]-10,wn[2]), cv.FONT_HERSHEY_SIMPLEX, 0.25, (0,255,255), lineType=cv.LINE_AA)
                continue
            cv.putText(image, str([wn[6], wn[7]]),(wn[1]-10,wn[2]), cv.FONT_HERSHEY_SIMPLEX, 0.25, (0,255,0), lineType=cv.LINE_AA)
            # cv.putText(image, str([wn[0]]),(wn[1]+150,wn[2]), cv.FONT_HERSHEY_SIMPLEX, 0.25, (0,255,0), lineType=cv.LINE_AA)


        # cv.imwrite('lk.jpg',image)

        # self._frpl2 = FRPL2
        # self._frpl1 = FRPL1
        # self._fcpl2 = FCPL2
        # self._fcpl1 = FCPL1
        #
        t2 = time.time()
        # print('mat time {:.6f}'.format(t2-t1))

        # if index_in:
        #     for wn in winTable:
        #         if len(wn)<=1: continue
        #         if wn[6]==self._index[0] and wn[7]==self._index[1]:
        #             self.trackedContour = wn[3]
        #             self.trackedBR = wn[8]
        #             self.oldCentroid = [wn[1],wn[2]]
        #             return
        #     print('Wrong index to windowIndex!')
        #     self.trackedContour = []
        # elif window_in:
        #     minDist = 1e7
        #     # self.centroidTrack(False)
        #     for wn in winTable:
        #         if len(wn)<=1: continue
        #         dist = utl.calcEuclideanDistance([wn[1],wn[2]], self.oldCentroid)
        #         if minDist>dist:
        #             minDist = dist
        #             self._index = [wn[6], wn[7]]
        #             cent = [wn[1],wn[2]]
        #             self.trackedContour = wn[3]
        #             self.trackedBR = wn[8]
        #     self.oldCentroid = cent
        #     return
        #         # if abs(self.oldCentroid[0]-wn[1])>(2*a) and abs(self.oldCentroid[1]-wn[2])>(3*b/2):
        #         # if abs(self.oldCentroid[0]-wn[1])<a/2 and abs(self.oldCentroid[1]-wn[2])<b/2:
        #         # if abs(self.oldCentroid[0]-wn[1])<a and abs(self.oldCentroid[1]-wn[2])<b:
        #         #     self.trackedContour = wn[3]
        #         #     self._index = [wn[6], wn[7]]
        #         #     return
        #     self.oldCentroid = []
        #     self.trackedContour = []
        #     self.trackedBR = ()
        #     print('Wrong contour to windowIndex!', a, b)
        return
