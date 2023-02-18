#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import rospy
import os
import math
import numpy as np
from clb_msgs.msg import Servo
from vision_msgs.msg import BoundingBox2D
import pid as pidlib
from clb_msgs.srv import *


class ClbrobotShoot():
    def __init__(self):
        rospy.init_node('clbrobot_shoot_node', log_level=rospy.INFO)
        r = rospy.Rate(20)
        rospy.on_shutdown(self.shutdown)


        self.fast_detect = rospy.get_param('~speed', True)
        self.offset = rospy.get_param('~offset', 20)

        self.sub_bbox = rospy.Subscriber("/detect_bbox", BoundingBox2D, self.BboxCallback, queue_size = 1)
        self.pub_servo = rospy.Publisher('/servo', Servo, queue_size=1)

        self.image_width = 640 
        self.image_height = 480 
        self.center_x = self.center_y = self.detect_square = 0
        self.center_y_offset = -10
        self.center_x_offset = 50
        self.x_pose = self.y_pose = 90
        self.x_shoot = self.y_shoot = False

        self.shoot_timeout = rospy.get_param('~shoot_timeout', 4)
        self.shoot_time = 0.5
        self.shoot_lock = False
        self.begin = self.end = rospy.get_time()

        #self.pid = pidlib.PID(0.03, 0, 0.001)
        #detect slow pid
        if self.fast_detect is True:
            self.x_pid = pidlib.PID(0.016, 0.001, 0.004)
            self.y_pid = pidlib.PID(0.014, 0.001, 0.002)
        else:
            self.x_pid = pidlib.PID(0.08, 0.002, 0.002)
            self.y_pid = pidlib.PID(0.06, 0.001, 0.002)
        self.x_pid.setPoint(0)
        self.y_pid.setPoint(0)
        
        self.servo = Servo() 

        while not rospy.is_shutdown():
            self.x_shoot = self.y_shoot = False
            if(self.detect_square > 0):
                self.detect_square = 0
                offset_x = self.image_width / 2 - self.center_x_offset - self.center_x
                if(abs(offset_x) > self.offset):
                    x = self.x_pid.update(offset_x)
                    self.update_xservo(x, 0, 180)
                else:
                    self.x_shoot = True
                offset_y = self.image_height / 2 - self.center_y_offset - self.center_y
                if(abs(offset_y) > self.offset):
                    y = self.y_pid.update(offset_y)
                    self.update_yservo(y, 0, 180)
                else:
                    self.y_shoot = True
            if self.x_shoot is True  and self.y_shoot is True :
                if self.shoot_lock is not True:
                    self.shoot_lock = True
                    self.end = self.begin = rospy.get_time()
                    rospy.loginfo("shooting..................")
                    self.ShootCtrlClient(self.shoot_time)
                else:
                    self.end  = rospy.get_time()
                if self.end - self.begin > self.shoot_timeout:
                    self.shoot_lock = False
                    self.begin = self.end = rospy.get_time()
                
            self.servo.Servo1 =  self.y_pose
            self.servo.Servo2 =  self.x_pose
            self.pub_servo.publish(self.servo)
            rospy.loginfo("(x_pose:%d, y_pose:%d)" %(self.x_pose, self.y_pose))
            r.sleep()

    def ShootCtrlClient(self, time):
        rospy.wait_for_service('shootctrl') 
        try:
            shootctrl = rospy.ServiceProxy('shootctrl', Shoot)
            shootctrl(time*10)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def update_xservo(self, x, min_value, max_value):
        self.x_pose = self.x_pose - int(x) 
        if self.x_pose > max_value:
            self.x_pose = max_value
        if self.x_pose < min_value:
            self.x_pose = min_value

    def update_yservo(self, y, min_value, max_value):
        self.y_pose = self.y_pose - int(y) 
        if self.y_pose > max_value:
            self.y_pose = max_value
        if self.y_pose < min_value:
            self.y_pose = min_value

    def BboxCallback(self, bbox_msg):
        self.center_x = bbox_msg.center.x
        self.center_y = bbox_msg.center.y
        self.detect_square = math.sqrt(bbox_msg.size_x*bbox_msg.size_x + bbox_msg.size_y*bbox_msg.size_y)
        #rospy.loginfo("(%d, %d)" %(self.center_x, self.center_y))


    def shutdown(self):
        # Release handle to the webcam
        rospy.logwarn("now will shutdown clbrobot tracker servo ...")

if __name__ == '__main__':
    try:
        Shoot = ClbrobotShoot()
        rospy.spin()
    except Exception, e:
        rospy.logerr("%s", str(e))
        os._exit(1)

