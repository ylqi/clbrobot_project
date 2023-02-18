#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import rospy
import numpy as np
import os,cv2
import face_recognition
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import BoundingBox2D


class FaceLocation():
    def __init__(self):
        rospy.init_node('clb_face_tracker_node', log_level=rospy.INFO)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')
        r = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown)

        self.sub_image_type = "raw"
        self.pub_face_type = "raw"
        self.cv_image = None

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
       
        # Initialize some variables
        face_locations = []
        top = 0
        right = 0
        bottom = 0
        left = 0

        while not rospy.is_shutdown():
            # Resize frame of video to 1/4 size for faster face recognition processing
            if self.cv_image is not None:
                    small_frame = cv2.resize(self.cv_image, (0, 0), fx=0.25, fy=0.25)

                    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
                    rgb_small_frame = small_frame[:, :, ::-1]
                    face_locations = face_recognition.face_locations(rgb_small_frame)
                    # calculate face center ready to move servo
                    for (top, right, bottom, left) in face_locations:
                        # Scale back up face locations since the frame we detected in was scaled to 1/4 size
                        top    *= 4
                        right  *= 4
                        bottom *= 4
                        left   *= 4

                        # print the face center postion
                        cv2.rectangle(self.cv_image, (left, top), (right, bottom), (0, 0, 255), 2)
                        cv2.rectangle(self.cv_image, (left, bottom - 35), (right, bottom), (0, 0, 255), 2)
                        font = cv2.FONT_HERSHEY_DUPLEX
                        cv2.putText(self.cv_image, "clbrobot_face_detect", (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
                    
                        center_x = left + (right-left)/2
                        center_y = top + (bottom-top)/2
                        bbox = BoundingBox2D()
                        bbox.center.x = center_x
                        bbox.center.y = center_y
                        bbox.size_x = right - left
                        bbox.size_y = bottom - top

                        self.pub_bbox.publish(bbox)
                        break # at same time only recognize one face


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
        face_location = FaceLocation()
        rospy.spin()
    except Exception as e:
        rospy.logerr("%s", str(e))
        os._exit(1)

