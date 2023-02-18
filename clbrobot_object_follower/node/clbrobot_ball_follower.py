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
        r = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown)

	self.sub_image_type = "raw"
        self.cvBridge = CvBridge()


        self.image_center_x = 320
        self.image_center_y = 240
        self.offset_x = 80
        self.follow_rect = 35*35
        self.offset_rect = 400
        self.linear_speed_max = 0.4
        self.angular_speed_max = 1

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.object_sub = rospy.Subscriber("/detect_bbox", BoundingBox2D, self.ObjectCallback)


    def ObjectCallback(self, object_msg):
        #rospy.loginfo(object_msg.detections[-1].bbox)  
        #rospy.loginfo("size: %f, center: %f", object_msg.bbox.size_x, object_msg.bbox.center.x)  
        if (object_msg.center.x < self.image_center_x - self.offset_x) or (object_msg.center.x > self.image_center_x + self.offset_x):
            angular_z = self.GetAngular(self.image_center_x - object_msg.center.x)
        else:
            angular_z = 0


        rect = object_msg.size_x * object_msg.size_y
        if(rect < self.follow_rect - self.offset_rect) or (rect > self.follow_rect + self.offset_rect):
            linear_x = self.GetLinear((self.follow_rect - rect))
        else:
            linear_x = 0
        rospy.loginfo("rect: %f, linear_x:%f, angular_z: %f", rect, linear_x, angular_z)  
        twist = Twist()
        twist.linear.x = linear_x
        #twist.linear.x = 0
        twist.angular.z =angular_z
        self.vel_pub.publish(twist)

    def GetLinear(self, offset):
        Kp = 0.0004
        Kd = 0.0001
        last_error = 0
        error = offset
        if abs(error) >= self.follow_rect:
            error = last_error

        speed = Kp*error + Kd*(error - last_error)
        last_error = error
        if speed > self.linear_speed_max:
            speed = self.linear_speed_max

        if speed < -self.linear_speed_max:
            speed = -self.linear_speed_max

        return speed

    def GetAngular(self, offset):
        Kp = 0.003
        Kd = 0.0002
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

