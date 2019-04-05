#!/usr/bin/env python
from __future__ import print_function

import sys

import cv2
import numpy as np
import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from color_object_detection.msg import Rectangle

class ImageConverter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/object_detection/image_segmented",Image, queue_size=10)
    self.bbox_pub = rospy.Publisher("/object_detection/bbox", Rectangle, queue_size=100)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_red1 = np.array([0, 50, 100])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([175, 50, 100])
    upper_red2 = np.array([180, 255, 255])

    # Threshold the HSV image to get only blue colors
    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1,mask2)
    kernel = np.ones((5, 5), np.uint8)

    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)

    ret,thresh = cv2.threshold(mask, 200, 255, 0)
    im, contours,hierarchy= cv2.findContours(thresh, 1, 2)

    c = max(contours, key = cv2.contourArea)

    cnt = contours[0]
    x, y, w, h = cv2.boundingRect(c)

    # Bitwise-AND mask and original image
    # res1 = cv2.bitwise_and(cv_image,cv_image, mask= mask1)
    # res2 = cv2.bitwise_and(cv_image,cv_image, mask= mask2)
    # res = cv2.bitwise_or(res1, res2)

    d = 10
    rect = Rectangle(x - d, y - d, x + w + d, y + h + d)
    self.bbox_pub.publish(rect)

    res = cv2.bitwise_and(cv_image, cv_image, mask= mask)
    cv2.rectangle(res, (x - d, y - d), (x + w + d, y + h + d), (0, 255, 0), 2)

    # cv2.imshow("Image window", res)
    # cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('object_detector', anonymous=True)
  ic = ImageConverter()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
