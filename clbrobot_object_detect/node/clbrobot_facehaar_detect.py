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


class ClbrobotFaceHaarDetect():
    def __init__(self):
        rospy.init_node('clbrobot_face_detect_node', log_level=rospy.INFO)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')
        r = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown)

        self.sub_image_type = "raw"
        self.pub_face_type = "raw"
        self.cv_image = None
        self.cvBridge = CvBridge()

        if self.sub_image_type == "compressed":
            self.sub_image_original = rospy.Subscriber(self.camera_topic+'compressed', CompressedImage, self.ImageCallback, queue_size = 1)
        elif self.sub_image_type == "raw":
            self.sub_image_original = rospy.Subscriber(self.camera_topic, Image, self.ImageCallback, queue_size = 1)


        if self.pub_face_type == "compressed":
            self.pub_face_image = rospy.Publisher('/object_image/compressed', CompressedImage, queue_size=1)
        else:
            self.pub_face_image = rospy.Publisher('/object_image', Image, queue_size=1)

        self.pub_bbox = rospy.Publisher("/detect_bbox", BoundingBox2D,  queue_size=1)

        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path = dir_path.replace('clbrobot_object_detect/node', 'clbrobot_object_detect/')
        file_path = dir_path + 'haar/haarcascade_frontalface_default.xml'

        #self.FaceCascade = cv2.CascadeClassifier('./haar/haarcascade_frontalface_default.xml')
        self.FaceCascade = cv2.CascadeClassifier(file_path)


	#center = (320, 240)
        center = None
        #w = h = 50

        while not rospy.is_shutdown():
            if self.cv_image is not None:
                gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
                faces = self.FaceCascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)   
                face = self.face_filter(faces)	
                if face is not None:
                    # 当前画面有人脸
                    (x, y, w, h) = face
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

    def face_filter(self, faces):
        if len(faces) == 0:
            return None
        # 目前找的是画面中面积最大的人脸
        max_face =  max(faces, key=lambda face: face[2]*face[3])
        (x, y, w, h) = max_face
        if w < 10 or h < 10:
            return None
        return max_face

    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown face_location_node ...")

if __name__ == '__main__':
    try:
        detect = ClbrobotFaceHaarDetect()
        rospy.spin()
    except Exception as e:
        rospy.logerr("%s", str(e))
        os._exit(1)

