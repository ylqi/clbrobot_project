#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import rospy
import numpy as np
import os,cv2
import math
import imutils
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import BoundingBox2D, Detection2DArray
from geometry_msgs.msg import Twist


class ClbrobotObjectFollower():
    def __init__(self):
        rospy.init_node('clbrobot_people_follower_node', log_level=rospy.INFO)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        self.id = rospy.get_param('~id', 1)
        r = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown)

        self.pub_object_type = "raw"
	self.sub_image_type = "raw"
        #self.cv_image = None
        self.cvBridge = CvBridge()


        self.image_center_x = 320
        self.image_center_y = 240
        self.offset_x = 80
        self.follow_rect = 200*500
        self.offset_rect = 20000
        self.linear_speed_max = 0.8
        self.angular_speed_max = 1.0


	#im_sub = message_filters.Subscriber(self.camera_topic, Image)
        #dep_sub = message_filters.Subscriber(self.depth_topic, Image)
        #self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub], 10, 0.5)
        #self.timeSynchronizer.registerCallback(self.ImageCallback)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.object_sub = rospy.Subscriber("/clbrobot_detect_node/detections", Detection2DArray, self.ObjectCallback)

        #if self.pub_object_type == "compressed":
        #    self.pub_object_image = rospy.Publisher('/object_image/compressed', CompressedImage, queue_size=1)
        #else:
        #    self.pub_object_image = rospy.Publisher('/object_image', Image, queue_size=1)
        

        while not rospy.is_shutdown():
            #if self.cv_image is not None:

            #    if self.pub_face_type == "compressed":
                    # publishes traffic sign image in compressed type
            #        self.pub_face_image.publish(self.cvBridge.cv2_to_compressed_imgmsg(self.cv_image, "jpg"))
            #    elif self.pub_face_type == "raw":
                    # publishes traffic sign image in raw type
            #        self.pub_face_image.publish(self.cvBridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
            r.sleep()

    #def ImageCallback(self, image_msg, depth_msg):
    #    if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
    #        np_arr = np.fromstring(image_msg.data, np.uint8)
    #        self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #    elif self.sub_image_type == "raw":
    #        self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

    #    self.depth_image = self.cvBridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

    def ObjectCallback(self, object_msg):
        obj_id = object_msg.detections[-1].results[-1].id;
        #rospy.loginfo("class id: %d", obj_id)  
        if obj_id == self.id:
            #rospy.loginfo(object_msg.detections[-1].bbox)  
            #rospy.loginfo("size: %f, center: %f", object_msg.detections[-1].bbox.size_x, object_msg.detections[-1].bbox.center.x)  
            if (object_msg.detections[-1].bbox.center.x < self.image_center_x - self.offset_x) or (object_msg.detections[-1].bbox.center.x > self.image_center_x + self.offset_x):
                angular_z = self.GetAngular(self.image_center_x - object_msg.detections[-1].bbox.center.x)
            else:
                angular_z = 0


            rect = object_msg.detections[-1].bbox.size_x * object_msg.detections[-1].bbox.size_y
            if(rect < self.follow_rect - self.offset_rect) or (rect > self.follow_rect + self.offset_rect):
                linear_x = self.GetLinear((self.follow_rect - rect)/100)
            else:
                linear_x = 0
            rospy.loginfo("rect: %f, linear_x:%f, angular_z: %f", rect, linear_x, angular_z)  
            twist = Twist()
            twist.linear.x = linear_x
            #twist.linear.x = 0
            twist.angular.z =angular_z
            self.vel_pub.publish(twist)

    def GetLinear(self, offset):
        Kp = 0.001
        Kd = 0.00001
        last_error = 0
        error = offset
        if abs(error) >= self.follow_rect/100:
            error = last_error

        speed = Kp*error + Kd*(error - last_error)
        last_error = error
        if speed > self.linear_speed_max:
            speed = self.linear_speed_max

        if speed < -self.linear_speed_max:
            speed = -self.linear_speed_max

        return speed

    def GetAngular(self, offset):
        Kp = 0.002
        Kd = 0.0001
        last_error = 0
        error = offset
        if abs(error) >= 320:
            error = last_error

        speed = Kp*error + Kd*(error - last_error)
        last_error = error
        if speed > self.angular_speed_max:
            speed = self.angular_speed_max

        if speed < -self.angular_speed_max:
            speed = -self.angular_speed_max

        return speed


    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown face_location_node ...")

if __name__ == '__main__':
    try:
        detect = ClbrobotObjectFollower()
        rospy.spin()
    except Exception, e:
        rospy.logerr("%s", str(e))
        os._exit(1)

