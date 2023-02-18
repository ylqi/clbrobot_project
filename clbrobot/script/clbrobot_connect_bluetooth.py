#!/usr/bin/env python

import rospy
import os
from geometry_msgs.msg import Twist, Vector3
from clb_msgs.msg import Blue_connect
from clb_msgs.msg import Bluetooth

class Getbluetoothdata:
	def __init__(self):
		# the topic for the tracker that gives us the current position of the object we are following
		self.BluetoothSubscriber = rospy.Subscriber('/bluetooth', Bluetooth, self.BlueCallback)
		self.Blue_contorl_pub = rospy.Publisher("blue_connect", Blue_connect, queue_size=1)
		os.system("rostopic pub -1 blue_connect clb_msgs/Blue_connect -- '1'")
                #self.blue = Blue_connect()
		#self.blue.connect_stats = 1
		#for i in range(2):
			#self.Blue_contorl_pub.publish(self.blue.connect_stats)
			#rospy.sleep(0.5)
		rospy.on_shutdown(self.controllerLoss)
	
	def BlueCallback(self, bluetooth):
		angle_x = bluetooth.angle_x
		angle_y = bluetooth.angle_y
		if(bluetooth.connect_stats==3):
			rospy.loginfo('Angle_x: {}, '.format(angle_x))
			rospy.loginfo('Angle_Y: {}, '.format(angle_y))
		#rate = rospy.Rate(1)
		#rate.sleep()
	def controllerLoss(self):
                os.system("rostopic pub -1 blue_connect clb_msgs/Blue_connect -- '0'")
		#self.blue.connect_stats = 0
		#for i in range(1):
			#self.Blue_contorl_pub.publish(self.blue.connect_stats)
			#rospy.sleep(1)
		#rospy.loginfo('CLBROBOT Bluetooth is lost connected')

if __name__ == '__main__':
        print('                                                              ')
	print('Try connecting to CLBROBOT Bluetooth,waiting for Bluetooth data')
        rospy.init_node('clbrobot_connect_bluetooth')
	bluetooth = Getbluetoothdata()
	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		print('exception')



