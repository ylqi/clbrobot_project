#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import rospy
import numpy as np
import os,cv2
import math
import imutils
from sklearn.metrics import pairwise

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import BoundingBox2D
from std_msgs.msg import UInt8


class ClbrobotBallDetect():
    def __init__(self):
        rospy.init_node('clbrobot_gesture_detect_node', log_level=rospy.INFO)
        self.rate = rospy.get_param('~rate', 20)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw/')
        r = rospy.Rate(self.rate)
        self.name = rospy.get_name()
        rospy.on_shutdown(self.shutdown)

        self.sub_image_type = "raw"
        self.pub_gesture_type = "raw"
        self.accumWeight = 0.5
        self.bg = None
        self.top, self.right, self.bottom, self.left = 10, 350, 225, 590    

        self.num_frames = 0

        if self.sub_image_type == "compressed":
            self.sub_image_original = rospy.Subscriber(self.camera_topic+'compressed', CompressedImage, self.ImageCallback, queue_size = 1)
        elif self.sub_image_type == "raw":
            self.sub_image_original = rospy.Subscriber(self.camera_topic, Image, self.ImageCallback, queue_size = 1)

        if self.pub_gesture_type == "compressed":
            self.pub_gesture_image = rospy.Publisher('/object_image/compressed', CompressedImage, queue_size=1)
        else:
            self.pub_gesture_image = rospy.Publisher('/object_image', Image, queue_size=1)

        self.pub_finger = rospy.Publisher('/finger', UInt8, queue_size=1)

        self.cvBridge = CvBridge()
       
        while not rospy.is_shutdown():
            r.sleep()

    def ImageCallback(self, image_msg):
        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        frame = imutils.resize(cv_image, width=700)
        frame = cv2.flip(frame, 1)
        clone = frame.copy()
        roi = frame[self.top:self.bottom, self.right:self.left]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)
        
        if self.num_frames < 30:
            self.run_avg(gray, self.accumWeight)
            if self.num_frames == 1:
                rospy.loginfo("[STATUS] please wait! calibrating...")
            elif self.num_frames == 29:
                rospy.loginfo("[STATUS] calibration successfull...")
        else:
 	    hand = self.segment(gray)
            if hand is not None:
                # if yes, unpack the thresholded image and
                # segmented region
                (thresholded, segmented) = hand

                # draw the segmented region and display the frame
                cv2.drawContours(clone, [segmented + (self.right, self.top)], -1, (0, 0, 255))

                # count the number of fingers
                fingers = UInt8()
                fingers.data = self.count(thresholded, segmented)

                cv2.putText(clone, str(fingers.data), (70, 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                self.pub_finger.publish(fingers)

                # show the thresholded image
                cv2.imshow("Thesholded", thresholded)
                cv2.waitKey(1)

        # draw the segmented hand
        cv2.rectangle(clone, (self.left, self.top), (self.right, self.bottom), (0,255,0), 2)
 	self.num_frames += 1
	
	if self.pub_gesture_type == "compressed":
            self.pub_gesture_image.publish(self.cvBridge.cv2_to_compressed_imgmsg(clone, "jpg"))

        elif self.pub_gesture_type == "raw":
            self.pub_gesture_image.publish(self.cvBridge.cv2_to_imgmsg(clone, "bgr8"))


    def count(self, thresholded, segmented):
        # find the convex hull of the segmented hand region
        chull = cv2.convexHull(segmented)

        # find the most extreme points in the convex hull
        extreme_top    = tuple(chull[chull[:, :, 1].argmin()][0])
        extreme_bottom = tuple(chull[chull[:, :, 1].argmax()][0])
        extreme_left   = tuple(chull[chull[:, :, 0].argmin()][0])
        extreme_right  = tuple(chull[chull[:, :, 0].argmax()][0])

        # find the center of the palm
        cX = int((extreme_left[0] + extreme_right[0]) / 2)
        cY = int((extreme_top[1] + extreme_bottom[1]) / 2)

        # find the maximum euclidean distance between the center of the palm
        # and the most extreme points of the convex hull
        distance = pairwise.euclidean_distances([(cX, cY)], Y=[extreme_left, extreme_right, extreme_top, extreme_bottom])[0]
        maximum_distance = distance[distance.argmax()]
        # calculate the radius of the circle with 80% of the max euclidean distance obtained
        radius = int(0.8 * maximum_distance)

        # find the circumference of the circle
        circumference = (2 * np.pi * radius)

        # take out the circular region of interest which has 
        # the palm and the fingers
        circular_roi = np.zeros(thresholded.shape[:2], dtype="uint8")

        # draw the circular ROI
        cv2.circle(circular_roi, (cX, cY), radius, 255, 1)

        # take bit-wise AND between thresholded hand using the circular ROI as the mask
        # which gives the cuts obtained using mask on the thresholded hand image
        circular_roi = cv2.bitwise_and(thresholded, thresholded, mask=circular_roi)

        # compute the contours in the circular ROI
        (_, cnts, _) = cv2.findContours(circular_roi.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # initalize the finger count
        count = 0

        # loop through the contours found
        for c in cnts:
            # compute the bounding box of the contour
            (x, y, w, h) = cv2.boundingRect(c)

            # increment the count of fingers only if -
            # 1. The contour region is not the wrist (bottom area)
            # 2. The number of points along the contour does not exceed
            #     25% of the circumference of the circular ROI
            if ((cY + (cY * 0.25)) > (y + h)) and ((circumference * 0.25) > c.shape[0]):
                count += 1

        return count

    def segment(self, image, threshold=25):
        # find the absolute difference between background and current frame
        diff = cv2.absdiff(self.bg.astype("uint8"), image)

        # threshold the diff image so that we get the foreground
        thresholded = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)[1]

        # get the contours in the thresholded image
        (_, cnts, _) = cv2.findContours(thresholded.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # return None, if no contours detected
        if len(cnts) == 0:
            return
        else:
            # based on contour area, get the maximum contour which is the hand
            segmented = max(cnts, key=cv2.contourArea)
            return (thresholded, segmented)
      
    def run_avg(self, image, accumWeight):
        # initialize the background
        if self.bg is None:
            self.bg = image.copy().astype("float")
            return

        # compute weighted average, accumulate it and update the background
        cv2.accumulateWeighted(image, self.bg, accumWeight)
        

    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown face_location_node ...")

if __name__ == '__main__':
    try:
        detect = ClbrobotBallDetect()
        rospy.spin()
    except Exception, e:
        rospy.logerr("%s", str(e))
        os._exit(1)

