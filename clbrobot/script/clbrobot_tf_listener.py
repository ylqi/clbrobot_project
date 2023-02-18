#!/usr/bin/env python
#coding=utf-8

import rospy

import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('clbrobot_multi')

    listener = tf.TransformListener() #TransformListener创建后就开始接受tf广播信息，最多可以缓存10s
    listen_robot = rospy.get_param('~listen_robot', 'clb1')
    follow_robot = rospy.get_param('~follow_robot', 'clb2')

    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    clbrobot_vel = rospy.Publisher(follow_robot + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            #得到以robot2为坐标原点的robot1的姿态信息(平移和旋转)
            (trans, rot) = listener.lookupTransform(follow_robot, listen_robot, rospy.Time(0)) #查看相对的tf,返回平移和旋转  turtle2跟着turtle1变换
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = 4*math.atan2(trans[1], trans[0]) #角度变换 计算出前往robot1的角速度
        linear = 0.5*math.sqrt(trans[0] ** 2 + trans[1] ** 2) #平移变换 计算出前往robot1的线速度
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = linear   #平移变换
        msg.angular.z = angular #角度变换
        rospy.loginfo('linear=%f, angular=%f', linear, angular)
        
        clbrobot_vel.publish(msg) #向/robot2/cmd_vel话题发布新坐标  (即robot2根据/robot2/cmd_vel的数据来控制robot2移动)
        rate.sleep()
        
