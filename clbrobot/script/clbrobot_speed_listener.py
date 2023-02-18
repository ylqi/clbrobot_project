#!/usr/bin/env python
#coding=utf-8

import rospy

import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist


class MultiRobotVelFollower():
    def __init__(self):

        listen_robot = rospy.get_param('~listen_robot', 'clb1')
        follow_robot = rospy.get_param('~follow_robot', 'clb2')

        #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
        self.clbrobot_vel_pub = rospy.Publisher(follow_robot + '/cmd_vel', Twist, queue_size=1)
        self.clbrobot_vel_sub = rospy.Subscriber(listen_robot + '/cmd_vel', Twist, self.cbGetVel, queue_size=1)
        #self.linear_x = 0.0
        #self.angular_z = 0.0

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            
            rate.sleep()


    def cbGetVel(self, vel_msg):
        vel = Twist()
        vel.linear.x = vel_msg.linear.x
        vel.angular.z = vel_msg.angular.z
        self.clbrobot_vel_pub.publish(vel)




if __name__ == '__main__':
    rospy.init_node('clbrobot_multi')
    node = MultiRobotVelFollower()
    rospy.spin()

        
