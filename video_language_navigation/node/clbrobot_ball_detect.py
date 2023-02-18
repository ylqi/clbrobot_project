#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import rospy
import numpy as np
import os,cv2
import math
import imutils

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import BoundingBox2D


class ClbrobotBallDetect():
    def __init__(self):
        rospy.init_node('clbrobot_ball_detect_node', log_level=rospy.INFO)
        self.rate = rospy.get_param('~rate', 20)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw/')
        r = rospy.Rate(self.rate)
        self.name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)

        self.sub_image_type = "raw"
        self.pub_face_type = "raw"
        self.cv_image = None

        #green ball
        #self.ballLower = np.array((45, 100, 30))
        #self.ballUpper = np.array((65, 256, 256))
        #blue ball
        #self.ballLower = np.array((100, 80, 46))
        #self.ballUpper = np.array((124, 255, 255))
        #red ball
        self.ballLower = np.array((0, 162, 167))
        self.ballUpper = np.array((179, 255, 255))


        if self.sub_image_type == "compressed":
            self.sub_image_original = rospy.Subscriber(self.camera_topic+'compressed', CompressedImage, self.ImageCallback, queue_size = 1)
        elif self.sub_image_type == "raw":
            self.sub_image_original = rospy.Subscriber(self.camera_topic, Image, self.ImageCallback, queue_size = 1)


        if self.pub_face_type == "compressed":
            self.pub_face_image = rospy.Publisher('/object_image/compressed', CompressedImage, queue_size=1)
        else:
            self.pub_face_image = rospy.Publisher('/object_image', Image, queue_size=1)

        self.pub_bbox = rospy.Publisher("/detect_bbox", BoundingBox2D,  queue_size=1)

        self.cvBridge = CvBridge()
        center = None
       

        while not rospy.is_shutdown():

            # Resize frame of video to 1/4 size for faster face recognition processing
            if self.cv_image is not None:
                blurred = cv2.GaussianBlur(self.cv_image, (11, 11), 0)
                hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, self.ballLower, self.ballUpper)
                mask = cv2.erode(mask, None, iterations=2)
                mask = cv2.dilate(mask, None, iterations=2)
                cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cnts = imutils.grab_contours(cnts)
                if len(cnts) > 0:
                    # find the largest contour in the mask, then use
                    # it to compute the minimum enclosing circle and
                    # centroid
                    c = max(cnts, key=cv2.contourArea)
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                    # only proceed if the radius meets a minimum size
                    if radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                        cv2.circle(self.cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        cv2.circle(self.cv_image, center, 5, (0, 0, 255), -1)
                        bbox = BoundingBox2D()
                        bbox.center.x = center[0]
                        bbox.center.y = center[1]
                        bbox.size_x = radius
                        bbox.size_y = radius

                        self.pub_bbox.publish(bbox)
                
                if self.pub_face_type == "compressed":
                # publishes traffic sign image in compressed type
                    self.pub_face_image.publish(self.cvBridge.cv2_to_compressed_imgmsg(self.cv_image, "jpg"))

                elif self.pub_face_type == "raw":
                # publishes traffic sign image in raw type
                    self.pub_face_image.publish(self.cvBridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
            r.sleep()


    def ImageCallback(self, image_msg):
        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")


    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown face_location_node ...")

if __name__ == '__main__':
    try:
        detect = ClbrobotBallDetect()
        rospy.spin()
    except Exception as e:
        rospy.logerr("%s", str(e))
        os._exit(1)

