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


class ClbrobotPeopleDetect():
    def __init__(self):
        rospy.init_node('clbrobot_people_detect_node', log_level=rospy.INFO)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')
        r = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown)

        self.sub_image_type = "raw"
        self.pub_obj_type = "raw"
        self.cv_image = None
        self.cvBridge = CvBridge()
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


        if self.sub_image_type == "compressed":
            self.sub_image_original = rospy.Subscriber(self.camera_topic+'compressed', CompressedImage, self.ImageCallback, queue_size = 1)
        elif self.sub_image_type == "raw":
            self.sub_image_original = rospy.Subscriber(self.camera_topic, Image, self.ImageCallback, queue_size = 1)


        if self.pub_obj_type == "compressed":
            self.pub_obj_image = rospy.Publisher('/object_image/compressed', CompressedImage, queue_size=1)
        else:
            self.pub_obj_image = rospy.Publisher('/object_image', Image, queue_size=1)

        self.pub_bbox = rospy.Publisher("/detect_bbox", BoundingBox2D,  queue_size=1)

	center = None

        while not rospy.is_shutdown():
            if self.cv_image is not None:
                gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
                boxes, weights = hog.detectMultiScale(self.cv_image, winStride=(8,8) )
                obj = self.object_filter(boxes)	
                if obj is not None:
                    # 当前画面有人脸
                    (x, y, w, h) = obj
                    # 在原彩图上绘制矩形
                    cv2.rectangle(self.cv_image, (x, y), (x+w, y+h), (0, 255, 0), 4)
                    #img_height, img_width,_ = img.shape
                    center = (x+w/2, y+h/2)
                    bbox = BoundingBox2D()
                    bbox.center.x = center[0]
                    bbox.center.y = center[1]
                    bbox.size_x = w
                    bbox.size_y = h 

                    self.pub_bbox.publish(bbox)

                if self.pub_obj_type == "compressed":
                    # publishes traffic sign image in compressed type
                    self.pub_obj_image.publish(self.cvBridge.cv2_to_compressed_imgmsg(self.cv_image, "jpg"))
                elif self.pub_obj_type == "raw":
                    # publishes traffic sign image in raw type
                    self.pub_obj_image.publish(self.cvBridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
            r.sleep()


    def ImageCallback(self, image_msg):
        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

    def object_filter(self, objects):
	if len(objects) == 0:
	    return None

	# 目前找的是画面中面积最大的人脸
	max_obj =  max(objects, key=lambda obj: obj[2]*obj[3])
	(x, y, w, h) = max_obj
	if w < 10 or h < 10:
	    return None
	return max_obj

    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown people node ...")

if __name__ == '__main__':
    try:
        detect = ClbrobotPeopleDetect()
        rospy.spin()
    except Exception, e:
        rospy.logerr("%s", str(e))
        os._exit(1)

