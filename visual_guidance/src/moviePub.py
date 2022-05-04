#!/usr/bin/env python

from __future__ import print_function
import rospy
import roslib
# roslib.load_manifest('visual_guidance')
import sys
import cv2 as cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty
import numpy as np

rospy.init_node('moviePub', anonymous=True)
pub = rospy.Publisher('/movie',Image,queue_size=10)

bridge = CvBridge()





# def pubVid(pub):
vid = "/home/hamidreza/thesis/videos/20200920_192308.mp4"
cap = cv2.VideoCapture(vid)
# total = int(cap.get(cv2.CV_CAP_PROP_FRAME_COUNT))
# print(total)
# print(type(cap))
i = 0
while True:
    i += 1
    res, frame = cap.read()
    # if frame==None:
    #     break
    kn = bridge.cv2_to_imgmsg(frame, "bgr8")
    cv2.imshow('jj',frame)
    print('sd', i)
    # pub.publish(kn)
    if cv2.waitKey(500) & 0xFF == ord('q'):
        break

rospy.spin()


# def main(args):
#   # # print("Node Started ...\n")
#   pub = rospy.Publisher('/movie',Empty,queue_size=10)
#   rospy.init_node('moviePub', anonymous=True)
#   # ic = window_detection()
#
#   try:
#     pubVid(pub)
#     rospy.spin()
#   except KeyboardInterrupt:
#     print("Shutting down")
#
# if __name__ == '__main__':
#     main(sys.argv)
