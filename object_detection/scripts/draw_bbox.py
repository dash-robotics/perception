#!/usr/bin/env python

# Python 2/3 compatibility
from __future__ import print_function

# Built-in modules
import os
import sys

# External modules
import cv2
import numpy as np

# ROS modules
import rospy
import ros_numpy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from realsense2_camera.msg import Extrinsics

# Globals
CV_BRIDGE = CvBridge()
BBOX_POINTS = None
EXTRINSICS = None


def ext_callback(ext_msg):
    global EXTRINSICS
    if EXTRINSICS is None:
        extrinsic_matrix = np.eye(4)
        extrinsic_matrix[:3, :3] = np.asarray(ext_msg.rotation).reshape(3, 3)
        extrinsic_matrix[:3, -1] = np.asarray(ext_msg.translation)
        EXTRINSICS = extrinsic_matrix
        print('\nExtrinsics:\n', EXTRINSICS)


def bbox_callback(bbox_msg):
    global BBOX_POINTS
    BBOX_POINTS = ros_numpy.point_cloud2.pointcloud2_to_array(bbox_msg)
    BBOX_POINTS = np.asarray(BBOX_POINTS.tolist())


def callback(img_msg, info_msg, image_pub):
    # Read image using CV bridge
    try:
        img = CV_BRIDGE.imgmsg_to_cv2(img_msg, 'bgr8')
    except CvBridgeError as e: 
        rospy.logerr(e)
        return

    # Extract points from message
    if (BBOX_POINTS is None) or (EXTRINSICS is None): return
    bbox3D = BBOX_POINTS
    bbox3D = np.hstack((bbox3D, np.full((bbox3D.shape[0], 1), 1)))
    
    # Project 3D to 2D and filter bbox within image boundaries
    P = np.matrix(info_msg.P).reshape(3, 4)
    M = np.matmul(P, EXTRINSICS)
    bbox2D = np.matmul(M, bbox3D.T).T
    bbox2D = bbox2D / bbox2D[:, -1]
    bbox2D = bbox2D[:, :2].astype('int').tolist()

    # Draw the projected 3D bbox
    color = (0, 255, 0)
    cv2.line(img, tuple(bbox2D[0]), tuple(bbox2D[1]), color, 2)
    cv2.line(img, tuple(bbox2D[0]), tuple(bbox2D[2]), color, 2)
    cv2.line(img, tuple(bbox2D[0]), tuple(bbox2D[4]), color, 2)
    cv2.line(img, tuple(bbox2D[1]), tuple(bbox2D[3]), color, 2)
    cv2.line(img, tuple(bbox2D[1]), tuple(bbox2D[5]), color, 2)
    cv2.line(img, tuple(bbox2D[2]), tuple(bbox2D[3]), color, 2)
    cv2.line(img, tuple(bbox2D[2]), tuple(bbox2D[6]), color, 2)
    cv2.line(img, tuple(bbox2D[3]), tuple(bbox2D[7]), color, 2)
    cv2.line(img, tuple(bbox2D[4]), tuple(bbox2D[5]), color, 2)
    cv2.line(img, tuple(bbox2D[4]), tuple(bbox2D[6]), color, 2)
    cv2.line(img, tuple(bbox2D[5]), tuple(bbox2D[7]), color, 2)
    cv2.line(img, tuple(bbox2D[6]), tuple(bbox2D[7]), color, 2)

    # Publish the projected bbox image
    try:
        image_pub.publish(CV_BRIDGE.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError as e: 
        rospy.logerr(e)


def listener():
    # Start node
    rospy.init_node('draw_bbox', anonymous=True)
    
    # Handle params
    camera_info = rospy.get_param('~camera_info_topic')
    image_color = rospy.get_param('~image_color_topic')
    bbox_points = rospy.get_param('~bbox_points_topic')
    extrinsics = rospy.get_param('~extrinsics')
    output_topic = rospy.get_param('~output_topic')
    
    # Log info
    rospy.loginfo('Current PID: [%d]' % os.getpid())
    rospy.loginfo('CameraInfo topic: %s' % camera_info)
    rospy.loginfo('Image topic: %s' % image_color)
    rospy.loginfo('Bbox topic: %s' % bbox_points)
    rospy.loginfo('Extrinsics topic: %s' % extrinsics)
    rospy.loginfo('Output topic: %s' % output_topic)

    # Subscribe to topics
    info_sub = message_filters.Subscriber(camera_info, CameraInfo)
    image_sub = message_filters.Subscriber(image_color, Image)
    bbox_sub = rospy.Subscriber(bbox_points, PointCloud2, bbox_callback)
    ext_sub = rospy.Subscriber(extrinsics, Extrinsics, ext_callback)

    # Publish output topic
    image_pub = rospy.Publisher(output_topic, Image, queue_size=5)

    # Synchronize the topics by time
    ats = message_filters.ApproximateTimeSynchronizer(
        [image_sub, info_sub], queue_size=10, slop=0.1)
    ats.registerCallback(callback, image_pub)

    # Keep python from exiting until this node is stopped
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down')


if __name__ == '__main__':
    # Start node
    listener()
