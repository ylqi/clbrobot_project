#!/usr/bin/env python
# _*_ coding:utf-8 _*_

from __future__ import print_function

import rospy
import numpy as np
import os,cv2
import face_recognition
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import BoundingBox2D

import socket
import sys

import select, termios, tty
from geometry_msgs.msg import Twist
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import threading

import struct
 
HOST, PORT = "172.16.38.29", 19984
mode = 'test'
if mode == 'train':
    print("######### Start Training ##########")
else:
    print("######### Start Testing ##########")


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }
settings = termios.tcgetattr(sys.stdin)

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)


class NavigationClient():
    def __init__(self):
        rospy.init_node('video_language_navigation_node', log_level=rospy.INFO)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')
        r = rospy.Rate(1) # 10
        rospy.on_shutdown(self.shutdown)

        self.sub_image_type = "raw"
        self.pub_face_type = "raw"
        self.cv_image = None

        if self.sub_image_type == "compressed":
            self.sub_image_original = rospy.Subscriber(self.camera_topic+'compressed', CompressedImage, self.ImageCallback, queue_size = 1)
        elif self.sub_image_type == "raw":
            self.sub_image_original = rospy.Subscriber(self.camera_topic, Image, self.ImageCallback, queue_size = 1)

        self.cvBridge = CvBridge()

        # ----------- Keybord teaching -----------
        speed = 1.0 # rospy.get_param("~speed", 0.5)
        turn = 1.0 # rospy.get_param("~turn", 1.0)
        repeat = rospy.get_param("~repeat_rate", 0.0)
        key_timeout = rospy.get_param("~key_timeout", 0.0)
        if key_timeout == 0.0:
            key_timeout = None

        self.pub_thread = PublishThread(repeat)

        x = 0
        y = 0
        z = 0
        th = 0
        status = 0

        self.pub_thread.wait_for_subscribers()
        self.pub_thread.update(x, y, z, th, speed, turn)

        prompt = input("Please input the language prompt: ")

        print(msg)
        print(vels(speed,turn))

        while not rospy.is_shutdown():
            # Resize frame of video to 1/4 size for faster face recognition processing
            if self.cv_image is not None:

                if mode == "train":
                    key = getKey(key_timeout)
                    if key in moveBindings.keys():
                        x = moveBindings[key][0]
                        y = moveBindings[key][1]
                        z = moveBindings[key][2]
                        th = moveBindings[key][3]
                    elif key in speedBindings.keys():
                        speed = speed * speedBindings[key][0]
                        turn = turn * speedBindings[key][1]

                        print(vels(speed,turn))
                        if (status == 14):
                            print(msg)
                        status = (status + 1) % 15
                    else:
                        # Skip updating cmd_vel if key timeout and robot already
                        # stopped.
                        if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                            continue
                        x = 0
                        y = 0
                        z = 0
                        th = 0
                        if (key == '\x03'):
                            break
         
                    self.pub_thread.update(x, y, z, th, speed, turn)

                small_frame = cv2.resize(self.cv_image, (0, 0), fx=0.25, fy=0.25)

                # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
                rgb_small_frame = small_frame[:, :, ::-1]
                face_locations = face_recognition.face_locations(rgb_small_frame)
                
                # -------------------- Send to server --------------------
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((HOST, PORT))
                arrBuf = bytearray(b'\xff\xaa\xff\xaa')
                
                picBytes = cv2.imencode(".jpg", self.cv_image)[1].tobytes()
                
                picSize = len(picBytes)
               
                datalen = 64 + 1 + 128 + 4 + 4 + 4 + 4 + 4 + 4 + picSize
                
                arrBuf += bytearray(datalen.to_bytes(4, byteorder='little'))
                guid = 23458283482894382928948
                arrBuf += bytearray(guid.to_bytes(64, byteorder='little'))
                arrBuf += b'\x00' if mode == "train" else b'\x01'
                arrBuf += prompt.ljust(128, "*").encode('utf-8')
                arrBuf += bytearray(struct.pack('<f', x))
                arrBuf += bytearray(struct.pack('<f', y))
                arrBuf += bytearray(struct.pack('<f', z))
                arrBuf += bytearray(struct.pack('<f', th))
                arrBuf += bytearray(struct.pack('<f', speed))
                arrBuf += bytearray(struct.pack('<f', turn))
                arrBuf += picBytes
                
                sock.sendall(arrBuf)
                # -------------------- Send to server --------------------
                
                # ------------------ Receive from server -----------------
                if mode == 'test':
                    recv_str = sock.recv(8)
                    data = bytearray(recv_str)
                    headIndex = data.find(b'\xff\xaa\xff\xaa')
                    if headIndex == 0:
                        allLen = int.from_bytes(data[headIndex+4:headIndex+8], byteorder='little')
                        curSize = 0
                        allData = b''
                        while curSize < allLen:
                            data = sock.recv(1024)
                            allData += data
                            curSize += len(data)
                        
                        arrGuid = allData[0:64]
                        tail = arrGuid.find(b'\x00')
                        arrGuid = arrGuid[0:tail]
                    
                        xData = allData[64:68]
                        yData = allData[68:72]
                        zData = allData[72:76]
                        thData = allData[76:80]
                    
                        x = struct.unpack('<f', xData)[0]
                        y = struct.unpack('<f', yData)[0]
                        z = struct.unpack('<f', zData)[0]
                        th = struct.unpack('<f', thData)[0]

                        print("Receive action: x: %d, y: %d, z: %d, th: %d" % (x, y, z, th))
                        if x == -2 and y == -2:
                            print("Done!")
                            self.shutdown()
                    
                        self.pub_thread.update(x, y, z, th, speed, turn)
                # ------------------ Receive from server -----------------
                sock.close()
                
           
                r.sleep()


    def ImageCallback(self, image_msg):
        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            self.cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")


    def shutdown(self):
        self.pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        # Release handle to the webcam
        rospy.logwarn("now will shutdown client_node ...")

if __name__ == '__main__':
    try:
        navigation_client = NavigationClient()
        rospy.spin()
    except Exception as e:
        rospy.logerr("%s", str(e))
        os._exit(1)

