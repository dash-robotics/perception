
import rospy
import cv2
import sys
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
import time


def image_publisher():
    pub = rospy.Publisher('camera_image', Image, queue_size=100)
    #if pub != None:
        #print "pub created"
    rospy.init_node('image_publisher', anonymous=True)
    rate = rospy.Rate(0.5) # not sure this is necessary
    bridge = CvBridge()

    # if len(sys.argv) < 2:
    #     print "You must give an argument to open a video stream."
    #     print "  It can be a number as video device, e.g.: 0 would be /dev/video0"
    #     print "  It can be a url of a stream,        e.g.: rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov"
    #     print "  It can be a video file,             e.g.: robotvideo.mkv"
    #     exit(0)

    # resource = sys.argv[1]
    # # If we are given just a number, interpret it as a video device
    # if len(resource) < 3:
    #     resource_name = "/dev/video" + resource
    #     resource = int(resource)
    #     vidfile = False
    # else:
    #     resource_name = resource
    #     vidfile = True
    # print "Trying to open resource: " + resource_name
    cap = cv2.VideoCapture(resource)
    rospy.Subscriber("camera/color/image_raw", ,)
    if not cap.isOpened():
        print "Error opening resource: " + str(resource)
        print "Maybe opencv VideoCapture can't open it"
        exit(0)


    print "Correctly opened resource, starting to show feed."
    rval, frame = cap.read()
    while rval:
        cv2.imshow("Stream: " + resource_name, frame)
        rval, frame = cap.read()

        # ROS image stuff
        if vidfile and frame is not None:
            frame = np.uint8(frame)
        image_message = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        #if image_message != None:
            #print "There's something here"
        #rospy.loginfo(image_message)

        pub.publish(image_message)

        key = cv2.waitKey(1000)   # was 20 but playing video fiels too fast
        # print "key pressed: " + str(key)
        # exit on ESC, you may want to uncomment the print to know which key is ESC for you
        if key == 27 or key == 1048603:
            break
    cv2.destroyWindow("preview")




if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass