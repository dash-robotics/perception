#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image,PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.callback)

    def callback(self, pc):
        print(len(pc.data))
        print(pc.point_step)
        print(pc.row_step)
        print(pc.height)
        print(pc.width)
        print(pc.fields)
        print()


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)