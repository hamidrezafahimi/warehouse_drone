#!/usr/bin/env python

import cv2 as cv
import time
import numpy as np
import utility as utl
import math
import yaml

class GridAnalyzer:

    def __init__(self, paramsFile, controllerParams, extractor):

        with open(paramsFile) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            self._enteranceThresh = 0.4 if not data.has_key("enteranceThresh") else data["enteranceThresh"]
            self._verticalCenter = 0.23 if not data.has_key("verticalCenter") else data["verticalCenter"]
            self._segmentExtractionPixelOffset = 10 if not data.has_key("segmentExtractionPixelOffset") else data["segmentExtractionPixelOffset"]
            self._minNodeNum = 68 if not data.has_key("minNodeNum") else data["minNodeNum"]
            self._gridImageHorLim = 20 if not data.has_key("gridImageHorLim") else data["gridImageHorLim"]
            self._gridImageVerLim = 70 if not data.has_key("gridImageVerLim") else data["gridImageVerLim"]
            self._vxConstCmd = 0.1 if not data.has_key("vxConstCmd") else data["vxConstCmd"]
            self._vyConstCmd = 0.2 if not data.has_key("vyConstCmd") else data["vyConstCmd"]
            self._vzConstCmd = 0.2 if not data.has_key("vzConstCmd") else data["vzConstCmd"]
            self._vzaConstCmd = 0.05 if not data.has_key("vzaConstCmd") else data["vzaConstCmd"]
            self._cellWidth = 0.572 if not data.has_key("cellWidth") else data["cellWidth"]
            self._cellHeight = 0.3 if not data.has_key("cellHeight") else data["cellHeight"]
            self._realNodeColsNum = 7 if not data.has_key("realNodeColsNum") else data["realNodeColsNum"]
            self._realNodeRowsNum = 10 if not data.has_key("realNodeRowsNum") else data["realNodeRowsNum"]
            self._nearNodeArea = 3500 if not data.has_key("nearNodeArea") else data["nearNodeArea"]

        with open(controllerParams) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            self._V = 0.5 if not data.has_key("V") else data["V"]
            self._Py_ins = -0.08 if not data.has_key("P_y_ins") else data["P_y_ins"]
            self._Pz_ins = -0.16 if not data.has_key("P_z_ins") else data["P_z_ins"]
            self._Py = -0.004 if not data.has_key("P_y") else data["P_y"]
            self._Pz = -0.008 if not data.has_key("P_z") else data["P_z"]
            self._Iy = -0.0004 if not data.has_key("I_y") else data["I_y"]
            self._Iz = -0.0 if not data.has_key("I_z") else data["I_z"]
            self._headingYLimCoef = 0.04 if not data.has_key("headingYLimCoef") else data["headingYLimCoef"]
            self._headingZLimCoef = 0.2 if not data.has_key("headingZLimCoef") else data["headingZLimCoef"]
            self._integralDySaturationCoef = 0.25 if not data.has_key("integralDySaturationCoef") else data["integralDySaturationCoef"]
            self._integralDzSaturationCoef = 0.1 if not data.has_key("integralDzSaturationCoef") else data["integralDzSaturationCoef"]
            self._straightCommandLimit = 0.4 if not data.has_key("straightCommandLimit") else data["straightCommandLimit"]
            self._yDriftCommandCoef = 1 if not data.has_key("yDriftCommandCoef") else data["yDriftCommandCoef"]
            self._zDriftCommandCoef = 1 if not data.has_key("zDriftCommandCoef") else data["zDriftCommandCoef"]
            self._dynamicDriftStraightCommandCoef = 0.5 if not data.has_key("dynamicDriftStraightCommandCoef") else data["dynamicDriftStraightCommandCoef"]
            self._sideDriftSpeed = 0.2 if not data.has_key("sideDriftSpeed") else data["sideDriftSpeed"]
            self._autoYawP = 0.008 if not data.has_key("autoYawP") else data["autoYawP"]
            self._entranceCrossSpeed = 0.3 if not data.has_key("entranceCrossSpeed") else data["entranceCrossSpeed"]

        self.imageInfoSet = False
        self.imCenter = []
        self._nodes = []
        self._frpl2 = False
        self._frpl1 = False
        self._fcpl2 = False
        self._fcpl1 = False
        self._lims = []
        self._extractor = extractor
        self._it = 0
        self.OK = False
        self._bestCell = [None, None]
        self.setProjectionParams()
        self._cellSet = False
        self.trackerStarted = False
        self._nearEnough = False
        self._mainRect = True
        self._psis = [[],[]]



    def setProjectionParams(self):

        self._realRectInfo = np.float32([ [0, 0, 0],
                                          [self._cellWidth, 0, 0],
                                          [self._cellWidth, self._cellHeight, 0],
                                          [0, self._cellHeight, 0]])
        self._axis3D = np.float32([ [0, 0, 0],
                                    [0.5, 0, 0],
                                    [0, 0.5, 0],
                                    [0, 0, 0.5]])



    def setCameraParams(self, cameraMatrix, distortionMatrix):

        self._cameraMatrix = cameraMatrix
        self._distortionMatrix = distortionMatrix



    def setImageInfo(self, image):

        self.rows,self.cols,_ = image.shape
        xpc = self.cols/2 - 1
        ypc = self.rows*self._verticalCenter
        self._setPoint = [xpc, ypc]
        self.imArea = self.rows*self.cols
        self.imageInfoSet = True
        self._enteranceArea = self._enteranceThresh * self.imArea



    def setCell(self, data):

        self._bestCell[0] = data.i
        self._bestCell[1] = data.j
        self._cellSet = True



    def analyze(self, image, windows):

        # self._it += 1
        if not self.imageInfoSet:
            self.setImageInfo(image)

        # self._bin = bin

        nodeGrid, cellGrid = [], []
        self._nodes = windows
        # nodeGrid, self.extrm, self._window, self._matrixDims, maxArea = self.generateMatrix(image)
        nodeGrid, self.extrm, self._window, self._matrixDims, nearEnough\
         = self.generateMatrix(image)

        # print maxArea, self._it

        # if maxArea>3500:
        #     self._nearEnough = True
        if not self._nearEnough and nearEnough:
            # utl.exportData(self._psis, 'psi')
            self._nearEnough = True

        # print len(nodeGrid) >= self._minNodeNum
        # print self.extrm[3] > 2*self._gridImageVerLim

        if self.trackerStarted:
            self._window = self.trTrack(image, nodeGrid)

        elif len(nodeGrid) >= self._minNodeNum and self.extrm[3] > 2*self._gridImageVerLim:
        # elif len(nodeGrid) >= self._minNodeNum:# and self.extrm[3] > 2*self._gridImageVerLim:
            cellGrid = self.makeInternalGrid(nodeGrid, image)
            self.OK = True

            if self._cellSet:
                roi = self.getRoi(nodeGrid)
                self.initTrackerMF(roi, image)

        if self._window == None:
            self._window = []

        return cellGrid, nodeGrid



    def trTrack(self, image, grid):

        # print 'trt in'
        self._trTrackOk = False

        success, newBR = self._trackerMF.update(image)
        if not success:
            # print 'trt 4'
            return

        center = [newBR[0]+(newBR[2]/2), newBR[1]+(newBR[3]/2)]

        dists = [[None]*2 for _ in range(len(grid))]
        for k, el in enumerate(grid):
            p = [el[2], el[3]]
            dists[k] = [utl.calcEuclideanDistance(center, p), k]

        dists.sort(key=lambda x: x[0])

        rect =  [[], [], [], []]
        for el in dists:

            if grid[el[1]][2]<center[0] and grid[el[1]][3]>center[1] and len(rect[0])==0:
                rect[0] = [[grid[el[1]][2], grid[el[1]][3]]]

            if grid[el[1]][2]>center[0] and grid[el[1]][3]>center[1] and len(rect[1])==0:
                rect[1] = [[grid[el[1]][2], grid[el[1]][3]]]

            if grid[el[1]][2]>center[0] and grid[el[1]][3]<center[1] and len(rect[2])==0:
                rect[2] = [[grid[el[1]][2], grid[el[1]][3]]]

            if grid[el[1]][2]<center[0] and grid[el[1]][3]<center[1] and len(rect[3])==0:
                rect[3] = [[grid[el[1]][2], grid[el[1]][3]]]

        if self._nearEnough or len(np.ravel(rect))<8:
            self._feedBack = center
            if self._mainRect:
                self.refineTracker(self._feedBack, center, newBR, image, True)
                self._mainRect = False
        else:
            self._feedBack = utl.findWindowCenter(rect)
        # self.trackedBR = newBR
        bbox = np.int32(newBR)
        cv.rectangle(image, (bbox[0], bbox[1]), (bbox[0]+bbox[2], bbox[1]+bbox[3]), (255,0,0), 2)
        # cv.rectangle(image, (newBR[0], newBR[1]), (newBR[0]+newBR[2], newBR[1]+newBR[3]), (255,0,0), 2)

        self.refineTracker(self._feedBack, center, newBR, image)

        self._trTrackOk = True
        # print 'trt out'
        return rect



    def refineTracker(self, cellCenter, roiCenter, roi, image, flag=False):

        if flag:
            self.initTrackerMF((roi[0]+(roi[2]/4), roi[1]+(roi[3]/4), roi[2]/2, roi[3]/2), image)


        if utl.calcEuclideanDistance(cellCenter, roiCenter)>15:

             vec = [cellCenter[0]-roiCenter[0], cellCenter[1]-roiCenter[1]]
             self.initTrackerMF((roi[0]+vec[0], roi[1]+vec[1], roi[2], roi[3]), image)





    def getRoi(self, grid):

        for el in grid:
            if self._bestCell[0] == el[0] and self._bestCell[1] == el[1]+1:
                x0, y0 = el[2], el[3]
                k1 = el[0]
            if self._bestCell[0] == el[0]+1 and self._bestCell[1] == el[1]:
                x1, y1 = el[2], el[3]
                k2 = el[1]

        # print 'getRoi', self._bestCell[0], self._bestCell[1], k1, k2


        x = np.max([0, x0-20])
        y = np.max([0, y0-20])
        w = np.min([x1-x0+40, self.cols])
        h = np.min([y1-y0+40, self.rows])
        # w = np.min
        return (x, y, w, h)




    def initTrackerMF(self, target, image):

        self._trackerMF = None
        self._trackerMF = cv.TrackerMedianFlow_create()

        # print '############  initTrackerMF  #############'
        img = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        self._trackerMF.init(image, target)
        self.trackerStarted = True






    def command(self, image, justAng = False, guidance = False):

        vx, vy, vz, vza = 0, 0, 0, 0

        if self._nearEnough:
                # print za, self._it

            # return self._entranceCrossSpeed, 0, 0, 0
            dy = (self._feedBack[0] - self._setPoint[0])
            vy = self._Py*dy
            return self._V, 0, 0, vy, True
        else:
            _, _, za = self.percept3D(image)
            # utl.saveData(self._psis, za, time.time())
            vza = 0.04*za

        if justAng:
            return 0, 0, 0, vza, False

        elif guidance:

            dy = (self._feedBack[0] - self._setPoint[0])
            dz = (self._feedBack[1] - self._setPoint[1])

            # self._integralDy += dy
            # self._integralDz += dz
            #
            # if self._integralDy<(-self._integralDySaturation):
            #     self._integralDy = -self._integralDySaturation
            # elif self._integralDy>self._integralDySaturation:
            #     self._integralDy = self._integralDySaturation
            #
            # if self._integralDz<(-self._integralDzSaturation):
            #     self._integralDz = -self._integralDzSaturation
            # elif self._integralDz>self._integralDzSaturation:
            #     self._integralDz = self._integralDzSaturation

            vx = self._V
            vy = self._Py*dy #+ self._Iy*self._integralDy
            vz = self._Pz*dz #+ self._Iz*self._integralDz
            # return vx, vy, vz, vza, False
            # print(za)

            if za > 2:
                vyl = -0.2
            elif za < -2:
                vyl = 0.2
            else:
                vyl = 0

            return vx, vyl, vz, vy, False

        else:

            rightLim = self.extrm[0] > (self.cols - self._gridImageHorLim)
            leftLim = self.extrm[2] < (self._gridImageHorLim)
            lowLim = self.extrm[3] > (self._gridImageVerLim)
            upLim = self.extrm[1] < (self.rows - self._gridImageVerLim)

            s = '0'
            if upLim and self._matrixDims[0] < self._realNodeRowsNum:
                vz = self._vzConstCmd
                s = '1'
            elif lowLim and self._matrixDims[0] < self._realNodeRowsNum:
                vz = -self._vzConstCmd
                s = '2'
            elif rightLim and leftLim:
                vx = -self._vxConstCmd
                s = '4'
            elif rightLim:
                vy = -self._vyConstCmd
                s = '5'
            elif leftLim:
                s = '6'
                vy = self._vyConstCmd
            elif lowLim and upLim and self._matrixDims[0] == self._realNodeRowsNum:
                vx = self._vxConstCmd
                s = '3'



            cv.putText(image,str(upLim)+' '+str(lowLim)+' '+str(rightLim)+' '+str(leftLim) ,(10,550), cv.FONT_HERSHEY_PLAIN,\
             1, [255,255,255], 2)
            cv.putText(image, s ,(10,570), cv.FONT_HERSHEY_PLAIN,\
             1, [255,255,255], 2)
            if za < 1:
                # cv.putText(image, 'a' ,(10,580), cv.FONT_HERSHEY_PLAIN,\
                #  1, [255,255,255], 2)
                return vx, vy, vz, vza, False
            else:
                # cv.putText(image, 'a' ,(10,580), cv.FONT_HERSHEY_PLAIN,\
                #  1, [255,255,255], 2)
                return 0, 0, 0, vza, False



    def percept3D(self, img):

        if len(self._window)==0 or len(np.ravel(self._window)) != 8:
            # print('ef', len(self._window)==0 , )
            return 0, 0, 0
        # print('dc1')
        # print(self._window)
        win = np.array(self._window)
        detectedRect = np.float32([win[0][0], win[1][0], win[2][0], win[3][0]])
        # print('dc2')
        #
        # print(self._realRectInfo)
        # print('-----------')
        # print(detectedRect)
        # print('-----------')
        # print(self._cameraMatrix)
        # print('-----------')
        # print(self._distortionMatrix)

        _, rotationVec, translationVec = cv.solvePnP(self._realRectInfo, detectedRect, self._cameraMatrix, self._distortionMatrix)

        # self.translationVector = utl.findRealTranslationNotThePnpOutput(translationVec, rotationVec)
        # print('dc2')

        axis2D, j = cv.projectPoints(self._axis3D, rotationVec, translationVec, self._cameraMatrix, self._distortionMatrix)
        # print('dc3')

        cv.drawContours(img, [win], -1, (0,0,255), 2)
        # print 'rotation Vector', rotationVec

        rvecs = utl.eulerFromRodrigues(rotationVec)
        # print 'roVec', rvecs

        cv.line(img, (axis2D[0][0][0], axis2D[0][0][1]), (axis2D[1][0][0],axis2D[1][0][1]), (255,255,0), 2)
        cv.line(img, (axis2D[0][0][0], axis2D[0][0][1]), (axis2D[2][0][0],axis2D[2][0][1]), (255,0,255), 2)
        cv.line(img, (axis2D[0][0][0], axis2D[0][0][1]), (axis2D[3][0][0],axis2D[3][0][1]), (0,255,255), 2)

        return rvecs[0], rvecs[1], rvecs[2]



    def makeInternalGrid(self, nodes, image):

        # print nodes
        nodes.sort(key=lambda x: x[0])
        nodes.sort(key=lambda x: x[1])
        # print nodes
        l = len(nodes)

        ipjs = [[None]*2 for _ in range(len(nodes))]
        imjs = [[None]*2 for _ in range(len(nodes))]

        for k, nd in enumerate(nodes):
            ipjs[k][0] = k
            imjs[k][0] = k
            ipjs[k][1] = nd[0] + nd[1]
            imjs[k][1] = nd[0] - nd[1]

        ipjs.sort(key=lambda x: x[1])
        imjs.sort(key=lambda x: x[1])

        grid = []
        checkList = []
        maxi = -1

        for k, ij in enumerate(ipjs):
            if k == l-1:
                break
            if ij[1] == ipjs[k+1][1]:
                center = utl.findWindowCenter([[[nodes[ij[0]][2], nodes[ij[0]][3]]], \
                [[nodes[ipjs[k+1][0]][2], nodes[ipjs[k+1][0]][3]]]])
                i = nodes[ij[0]][0]
                j = nodes[ipjs[k+1][0]][1]
                grid.append([i, j, center[0], center[1]])
                checkList.append('{:d},{:d}'.format(i,j))
            if maxi<nodes[ij[0]][0]:
                maxi = nodes[ij[0]][0]

        for k, ij in enumerate(imjs):
            if k == l-1:
                break
            i = nodes[imjs[k+1][0]][0]
            j = nodes[imjs[k+1][0]][1]
            if ij[1] == imjs[k+1][1] and ('{:d},{:d}'.format(i,j) not in checkList):
                center = utl.findWindowCenter([[[nodes[ij[0]][2], nodes[ij[0]][3]]], \
                [[nodes[imjs[k+1][0]][2], nodes[imjs[k+1][0]][3]]]])
                grid.append([i, j, center[0], center[1]])

        lg = len(grid)
        grid.sort(key=lambda x: -x[0])
        grid.sort(key=lambda x: x[1])
        for k, g in enumerate(grid):
            # print g
            cv.putText(image, str([g[0], g[1]]), (g[2]-10,g[3]), cv.FONT_HERSHEY_SIMPLEX,\
             0.25, (0,255,255), lineType=cv.LINE_AA)
            # cv.putText(self._bin, str([g[0], g[1]]), (g[2]-10,g[3]), cv.FONT_HERSHEY_SIMPLEX,\
             # 0.35, 0, lineType=cv.LINE_AA)
            if k == lg-1:
                break
            if g[0]==maxi:
                dx = grid[k+1][2]-g[2]
                dy = grid[k+1][3]-g[3]
                x = max(g[2]-dx, 0)
                y = max(g[3]-dy, 0)
                i = maxi+1
                j = g[1]
                grid.append([i, j, x, y])
                cv.putText(image, str([i, j]), (x-10,y), cv.FONT_HERSHEY_SIMPLEX, 0.25, (0,255,255), lineType=cv.LINE_AA)
                # cv.putText(self._bin, str([i, j]), (x-10,y), cv.FONT_HERSHEY_SIMPLEX, 0.25, 0, lineType=cv.LINE_AA)
        # print '------------------'

        return grid


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
                else:
                    col.remove(col[k])
            lc = len(col)
            k += 1

        if nearestX_y<b or nearestY_x<a or nearestX_y==1e7 or nearestY_x==1e7:
            if nearestX_y<b or nearestX_y==1e7:
                nearestX_y = b
            if nearestY_x<a or nearestY_x==1e7:
                nearestY_x = a

        return nearestX_y, nearestY_x, row, col;


    def generateMatrix(self, image):

        t1 = time.time()
        self._it += 1

        # if self._it >= 3:
        #     return

        winTable = [[None]*9 for _ in range(len(self._nodes))]
        for i, poly in enumerate(self._nodes):
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
                # print 2*wn[8][2], 2*wn[8][3]
                bl, al,  wn[4], wn[5] = self.findNearest(winTable, wn, 2*wn[8][2], 2*wn[8][3])
                # print wn[0], al, bl, wn[4], wn[5]
                if al<wn[8][2]:
                    al = wn[8][2]
                if bl<wn[8][3]:
                    bl = wn[8][3]
                if b>bl:
                    b = bl
                if a>al:
                    a = al
                # dists[k] = [al, bl, wn, k]
        else:
            for k, wn in enumerate(winTable):
                bl, al, wn[4], wn[5] = self.findNearest(winTable, wn, self._lims[0], self._lims[1])
                # print wn[0], al, bl, wn[4], wn[5]
                if al<wn[8][2]:
                    al = wn[8][2]
                if bl<wn[8][3]:
                    bl = wn[8][3]
                if b>bl:
                    b = bl
                if a>al:
                    a = al
                # dists[k] = [al, bl, wn, k]


        numWin = len(winTable)
        if numWin==0:
            # print('no mat')
            return [], [], [], [], 0
        #
        ylim = 8*(a*b)/math.sqrt((b*b)+(4*a*a))
        xlim = 8*(a*b)/math.sqrt((4*b*b)+(a*a))
        self._lims = [xlim,ylim]

        # print 'lims', xlim, ylim, a, b

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
            if len(indexBox)==numWin:
                break
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
            if len(indexBox)==numWin:
                break
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
        li, hi, lj, hj = -1, -1, -1, -1

        lies = len(ies)
        ljes = len(jes)

        li = lies/2
        lj = ljes/2
        hi = li + 1
        hj = lj + 1

        lens = [lies, ljes]
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
        #
        #
        #
        # self._frpl2 = FRPL2
        # self._frpl1 = FRPL1
        # self._fcpl2 = FCPL2
        # self._fcpl1 = FCPL1
        #
        t2 = time.time()
        # print('mat time {:.6f}'.format(t2-t1))
        minx, miny, maxx, maxy = 1e6, 1e6, -1, -1
        matrix = [[None]*4 for _ in range(len(winTable))]
        navNodes = [[], [], [], []]
        area = 0
        nnn = 0
        sum = 0
        upRowYMid = -1
        maxArea = -1
        for i, wn in enumerate(winTable):
            if len(wn)<=1:
                continue
            matrix[i][0] = wn[6]
            matrix[i][1] = wn[7]
            matrix[i][2] = wn[1]
            matrix[i][3] = wn[2]

            area = wn[8][2]*wn[8][3]
            if maxArea < area:
                maxArea = area

            if wn[6] == hi:
                nnn += 1
                sum += wn[2]
                upRowYMid = sum/nnn

            if maxx<wn[1]:
                maxx = wn[1]
            if maxy<wn[2]:
                maxy = wn[2]
            if minx>wn[1]:
                minx = wn[1]
            if miny>wn[2]:
                miny = wn[2]

            if wn[6] == li and wn[7] == lj:
                navNodes[0] = [[wn[1], wn[2]]]
            elif wn[6] == li and wn[7] == hj:
                navNodes[1] = [[wn[1], wn[2]]]
            elif wn[6] == hi and wn[7] == hj:
                navNodes[2] = [[wn[1], wn[2]]]
            elif wn[6] == hi and wn[7] == lj:
                navNodes[3] = [[wn[1], wn[2]]]
                # area = wn[8][2]*wn[8][3]

        # for i, wn in enumerate(matrix):
        #     matrix[]
        # print((upRowYMid>150),(nnn < 2),(minx < 40),(maxx > (self.cols-40)))
        f = (upRowYMid>150) or (nnn < 2) or (minx < 40) or (maxx > (self.cols-40))
        f = f and (maxArea > self._nearNodeArea)

        extrm = [maxx, maxy, minx, miny]
        # print('mat info:   ',maxx, minx, upRowYMid, f, maxArea)
        for k, wn in enumerate(winTable):
            cv.putText(image, str([wn[6], wn[7]]), (wn[1]-10,wn[2]), cv.FONT_HERSHEY_SIMPLEX, 0.25, (0,0,0), lineType=cv.LINE_AA)
            # cv.putText(self._bin, str([wn[6], wn[7]]), (wn[1]-10,wn[2]-10), cv.FONT_HERSHEY_SIMPLEX, 0.22, 0, lineType=cv.LINE_AA)

        if self._it==1:
            filename = "/home/hamidreza/thesis/workSpace/src/visual_guidance/src/file_%d.jpg"%self._it
            cv.imwrite(filename, image)
        # print matrix
        return matrix, extrm, navNodes, lens, f
