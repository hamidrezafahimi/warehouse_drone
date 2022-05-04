#!/usr/bin/env python

import math
import numpy as np
import cv2
from matplotlib import pyplot as plt
from matplotlib import mlab
import pandas as pd
import time
# from sklearn.mixture import GaussianMixture
# from scipy.signal import find_peaks
# from scipy.signal import medfilt
# from scipy.ndimage import gaussian_filter1d

def findWindowCenter(contour):

    sumx = 0
    sumy = 0

    for p in contour:
        sumx += p[0][0]
        sumy += p[0][1]

    cx = int(sumx / len(contour))
    cy = int(sumy / len(contour))

    return [cx, cy]

def calcEuclideanDistance(p1, p2):
    diff = [p1[0]-p2[0], p1[1]-p2[1]]
    return math.sqrt((diff[0]*diff[0])+(diff[1]*diff[1]))

def duplicateContour(contour):

    newContour = [None]*2*len(contour)
    i = 0
    for k, pt in enumerate(contour):
        newContour[i] = contour[k]
        i += 1
        if k==len(contour)-1:
            [a,b] = [int((pt[0][0] + contour[0][0][0])/2), int((pt[0][1] + contour[0][0][1])/2)]
        else:
            [a,b] = [int((pt[0][0] + contour[k+1][0][0])/2), int((pt[0][1] + contour[k+1][0][1])/2)]
        newContour[i] = [[a,b]]
        i += 1

    return newContour

def reconstructRect(contour, sort):
    lc = len(contour)
    if lc==1: return contour
    win = []
    if lc==2:
        win = [[[contour[0][0][0], contour[0][0][1]]],
              [[contour[0][0][0], contour[1][0][1]]],
              [[contour[1][0][0], contour[1][0][1]]],
              [[contour[1][0][0], contour[0][0][1]]]]
    elif lc==3:
        diff = -1
        for i in range(3):
            for j in range(3):
                if i==j: continue
                if diff<(abs(contour[i][0][0]-contour[j][0][0])+abs(contour[i][0][1]-contour[j][0][1])):
                    diff = abs(contour[i][0][0]-contour[j][0][0])+abs(contour[i][0][1]-contour[j][0][1])
                    diam1_1 = [[contour[i][0][0], contour[i][0][1]]]
                    diam1_2 = [[contour[j][0][0], contour[j][0][1]]]
        for k in range(3):
            di = [[contour[k][0][0], contour[k][0][1]]]
            if di!=diam1_1 and di!=diam1_2:
                diam2_1 = di
        diam2_2 = [[diam1_2[0][i] + diam1_1[0][i] - diam2_1[0][i] for i in range(2)]]
        win = [diam1_1,diam2_1,diam1_2,diam2_2]
    elif lc==4: win = contour
    elif not sort:
        p1, p2, p3, p4 = [], [],[],[]
        min_xpy=1e6
        max_xpy=-1
        min_xmy=1e6
        max_xmy=-1e6
        for cnt in contour:
            xpy = cnt[0][0] + cnt[0][1]
            xmy = cnt[0][0] - cnt[0][1]
            if min_xpy>=xpy:
                min_xpy = xpy
                p1 = cnt
            if min_xmy>=xmy:
                min_xmy = xmy
                p2 = cnt
            if max_xpy<=xpy:
                max_xpy = xpy
                p3 = cnt
            if max_xmy<=xmy:
                max_xmy = xmy
                p4 = cnt
        win = [p1,p2,p3,p4]
    else:
        win = contour

    if sort and len(win)!=0:
        p1, p2, p3, p4 = [], [],[],[]
        min_xpy=1e6
        max_xpy=-1
        min_xmy=1e6
        max_xmy=-1e6
        i = None
        j = None
        for k, pt in enumerate(win):
            xpy = pt[0][0] + pt[0][1]
            if min_xpy>=xpy:
                min_xpy = xpy
                p1 = pt
                i = k
            if max_xpy<=xpy:
                max_xpy = xpy
                p3 = pt
                j = k
        win = np.delete(win,[i,j],axis=0)
        for k, pt in enumerate(win):
            xmy = pt[0][0] - pt[0][1]
            if min_xmy>=xmy:
                min_xmy = xmy
                p4 = pt
            if max_xmy<=xmy:
                max_xmy = xmy
                p2 = pt
        win = [p4,p3,p2,p1]

    win = np.array(win)
    return win

def remove_noise(gray, num):
    Y, X = gray.shape
    nearest_neigbours = [[
        np.argmax(
            np.bincount(
                gray[max(i - num, 0):min(i + num, Y), max(j - num, 0):min(j + num, X)].ravel()))
        for j in range(X)] for i in range(Y)]
    result = np.array(nearest_neigbours, dtype=np.uint8)
    return result

def merge_intervals(intervals):
    starts = intervals[:,0]
    ends = np.maximum.accumulate(intervals[:,1])
    valid = np.zeros(len(intervals) + 1, dtype=np.bool)
    valid[0] = True
    valid[-1] = True
    valid[1:-1] = starts[1:] >= ends[:-1]
    return np.vstack((starts[:][valid[:-1]], ends[:][valid[1:]])).T

def findPeaks(input, mask, dev, idx):
    # fig, ax = plt.subplots()

    hist = cv2.calcHist([input], [idx], mask, [180], [0,180])
    hist = cv2.normalize(hist, None, 0, 255, cv2.NORM_MINMAX)

    peaks = find_peaks(hist.flatten(), prominence=2)
    # ax.plot(hist)
    # ax.plot(peaks[0], hist[peaks[0]], 'bo')

    intervals = np.zeros([len(peaks[0]), 2], dtype=np.int16)
    maxh = hist[peaks[0]].max()
    for idx, peak in enumerate(peaks[0]):
        h = hist[peak]
        ratio = float(h) / maxh
        d = int(math.ceil(dev * ratio))

        intervals[idx][0] = peak - d
        intervals[idx][1] = peak + d

        # ax.axvline(peak - d, 0, 1000, color="g")
        # ax.axvline(peak + d, 0, 1000, color="g")
        # ax.fill_between( range(peak - d, peak + d + 1), 0, 1,
        #         color='green', alpha=0.5, transform=ax.get_xaxis_transform())

    # plt.show()
    return intervals

def separateDist(input):
    plt.ion()
    plt.show()

    # blurred = cv2.GaussianBlur( input, ( 21, 21 ) , cv2.BORDER_DEFAULT);

    hist = cv2.calcHist([input], [1], None, [255], [0,255])
    hist = cv2.normalize(hist, None, 0, 255, cv2.NORM_MINMAX)
    hist = medfilt(hist.flatten(), 3)
    hist = gaussian_filter1d(hist.flatten(), 5)
    # print hist
    peaks = find_peaks(hist.flatten(), distance=50)
    plt.subplot(221)
    plt.cla()
    plt.plot(hist)
    plt.plot(peaks[0], hist[peaks[0]], 'bo')

    mixture = GaussianMixture(n_components=3).fit(hist.reshape(-1,1))
    means_hat = mixture.means_.flatten()
    sds_hat = np.sqrt(mixture.covariances_).flatten()

    print(mixture.converged_)
    print(means_hat)
    print(sds_hat)

    # for i in range(len(means_hat)):
    plt.subplot(222)
    plt.cla()
    plt.plot(range(256), mlab.normpdf(range(256), means_hat[0], sds_hat[0]))
    plt.ylim(0, 1)

    plt.subplot(223)
    plt.cla()
    plt.plot(range(256), mlab.normpdf(range(256), means_hat[1], sds_hat[1]))
    plt.ylim(0, 1)

    plt.subplot(224)
    plt.cla()
    plt.plot(range(256), mlab.normpdf(range(256), means_hat[2], sds_hat[2]))
    plt.ylim(0, 1)

    plt.draw()
    plt.pause(0.01)

def brToContour(br):
    return np.array([[[np.int32(br[0]), np.int32(br[1]+br[3])]],
            [[np.int32(br[0]+br[2]), np.int32(br[1]+br[3])]],
            [[np.int32(br[0]+br[2]), np.int32(br[1])]],
            [[np.int32(br[0]), np.int32(br[1])]]])

def iterateLine(P1, P2, img):

    imageH = img.shape[0]
    imageW = img.shape[1]
    P1X = P1[0]
    P1Y = P1[1]
    P2X = P2[0]
    P2Y = P2[1]

    dX = P2X - P1X
    dY = P2Y - P1Y
    dXa = np.abs(dX)
    dYa = np.abs(dY)

    itbuffer = np.empty(shape=(np.maximum(dYa,dXa),3),dtype=np.int16)

    negY = P1Y > P2Y
    negX = P1X > P2X
    if P1X == P2X:
        itbuffer[:,0] = P1X
        if negY:
            itbuffer[:,1] = np.arange(P1Y - 1,P1Y - dYa - 1,-1)
        else:
            itbuffer[:,1] = np.arange(P1Y+1,P1Y+dYa+1)
    elif P1Y == P2Y:
        itbuffer[:,1] = P1Y
        if negX:
            itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
        else:
            itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)
    else:
        steepSlope = dYa > dXa
        if steepSlope:
            slope = dX.astype(np.float32)/dY.astype(np.float32)
            if negY:
                itbuffer[:,1] = np.arange(P1Y-1,P1Y-dYa-1,-1)
            else:
                itbuffer[:,1] = np.arange(P1Y+1,P1Y+dYa+1)
            itbuffer[:,0] = (slope*(itbuffer[:,1]-P1Y)).astype(np.int16) + P1X
        else:
            slope = dY.astype(np.float32)/dX.astype(np.float32)
            if negX:
                itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
            else:
                itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)
            itbuffer[:,1] = (slope*(itbuffer[:,0]-P1X)).astype(np.int16) + P1Y

    itbuffer[:,2] = img[itbuffer[:,1].astype(np.uint),itbuffer[:,0].astype(np.uint)]

    return np.sum(itbuffer[:,2])

def iterateLine1(P1, P2):

    P1X = P1[0]
    P1Y = P1[1]
    P2X = P2[0]
    P2Y = P2[1]

    dX = P2X - P1X
    dY = P2Y - P1Y
    dXa = np.abs(dX)
    dYa = np.abs(dY)

    itbuffer = np.empty(shape=(np.maximum(dYa,dXa),2),dtype=np.int16)

    negY = P1Y > P2Y
    negX = P1X > P2X
    if P1X == P2X:
        itbuffer[:,0] = P1X
        if negY:
            itbuffer[:,1] = np.arange(P1Y - 1,P1Y - dYa - 1,-1)
        else:
            itbuffer[:,1] = np.arange(P1Y+1,P1Y+dYa+1)
    elif P1Y == P2Y:
        itbuffer[:,1] = P1Y
        if negX:
            itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
        else:
            itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)
    else:
        steepSlope = dYa > dXa
        if steepSlope:
            slope = float(dX)/float(dY)
            if negY:
                itbuffer[:,1] = np.arange(P1Y-1,P1Y-dYa-1,-1)
            else:
                itbuffer[:,1] = np.arange(P1Y+1,P1Y+dYa+1)
            itbuffer[:,0] = (slope*(itbuffer[:,1]-P1Y)).astype(np.int16) + P1X
        else:
            slope = float(dY)/float(dX)
            if negX:
                itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
            else:
                itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)
            itbuffer[:,1] = (slope*(itbuffer[:,0]-P1X)).astype(np.int16) + P1Y

    return itbuffer


def eulerFromRodrigues(rvecs):
    # from math import pi,atan2,asin
    R = cv2.Rodrigues(rvecs)[0]
    roll = 180*math.atan2(-R[2][1], R[2][2])/np.pi
    pitch = 180*math.asin(R[2][0])/np.pi
    yaw = 180*math.atan2(-R[1][0], R[0][0])/np.pi
    rot_params= [roll,pitch,yaw]
    return rot_params


def findRealTranslationNotThePnpOutput(tvecs, rvecs):

    R = cv2.Rodrigues(rvecs)[0]
    # R = np.array(R)
    # cameraRotationVector = cv2.Rodrigues(R.transpose(), )[0]
    # print R
    # print tvecs
    cameraTranslationVector = np.dot(-R.transpose(), tvecs)
    return cameraTranslationVector


def borderPointsCheck(contour, xlim, ylim, borderCheck=False, limVal=5):

    hor, ver = False, False
    datax = [False]*4
    datay = [False]*4
    if borderCheck:
        for k, pnt in enumerate(contour):
            if pnt[0][0]>=xlim or abs(pnt[0][0]-xlim)<limVal or pnt[0][0]<limVal:
                # print 'k1 ', k
                hor = True
                datax[k] = True
            if pnt[0][1]>=ylim or abs(pnt[0][1]-ylim)<limVal or pnt[0][1]<limVal:
                # print 'k2 ', k
                ver = True
                datay[k] = True
        return hor, ver, [datax, datay]
    else:
        for pnt in contour:
            if (abs(pnt[0][0]-xlim)<limVal or pnt[0][0]<limVal) and (abs(pnt[0][1]-ylim)<limVal or pnt[0][1]<limVal):
                return True
        return False


def cropRect(rect, img, minW=0, minH=0, wRatio=1, hRatio=1):

    ww = max(rect[2], minW)
    hh = max(rect[3], minH)
    a = int(max(0,rect[0]-(((wRatio-1)/2)*ww)))
    b = int(max(0,rect[1]-(((hRatio-1)/2)*hh)))
    c = int(wRatio*ww)
    d = int(hRatio*hh)
    croppedIm = img[b:b+d, a:a+c]

    return croppedIm, a, b


def exportData(data, name):

    print('exportData called')
    data[0][0] = 0
    dict = {'time': data[0], 'data':data[1], 'msgTime':data[2]}
    df = pd.DataFrame(dict)
    fileName = 'data_'+str(int(time.time()))+ name +'.csv'
    # df.to_csv('csv_files/'+fileName)
    df.to_csv('/home/hamidreza/thesis/data/'+fileName)



def saveData(dataSet, data, t):

    if len(dataSet[0]) == 0:
        dataSet[0].append(t)
    else:
        dataSet[0].append(t-dataSet[0][0])

    dataSet[1].append(data)
    dataSet[2].append(t)
