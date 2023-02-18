#!/usr/bin/env python
# _*_ coding:utf-8 _*_

import rospy
import numpy as np
import os,cv2
import face_recognition
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

class ClbrobotFaceDetect():
    def __init__(self):
        rospy.init_node('clbrobot_facename_detect_node', log_level=rospy.INFO)
        self.rate = rospy.get_param('~rate', 10)
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw/')
        r = rospy.Rate(self.rate)
        self.name = rospy.get_name()
        rospy.loginfo("start  face name detect")
        rospy.on_shutdown(self.shutdown)

        self.sub_image_type = "raw"
        self.pub_face_type = "raw"
        self.cv_image = None
        self.counter = 1
        self.time_flag = True
        
        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path = dir_path.replace('clbrobot_object_detect/node', 'clbrobot_object_detect/')
        dir_path += 'image/'


        id1_image = face_recognition.load_image_file(dir_path + "zhulin1.jpg")
        id1_face_encoding = face_recognition.face_encodings(id1_image)[0]

        # Load a second sample picture and learn how to recognize it.
        id2_image = face_recognition.load_image_file(dir_path + "zhulin2.jpg")
        id2_face_encoding = face_recognition.face_encodings(id2_image)[0]

	    # Create arrays of known face encodings and their names
        known_face_encodings = [
                id1_face_encoding,
                id2_face_encoding
        ]
        known_face_names = [
                "zhulin",
                "zhulin"
            ]

        face_locations = []
        face_encodings = []
        face_names = []
        process_this_frame = True

        if self.sub_image_type == "compressed":
            self.sub_image_original = rospy.Subscriber(self.camera_topic+'compressed', CompressedImage, self.ImageCallback, queue_size = 1)
        elif self.sub_image_type == "raw":
            self.sub_image_original = rospy.Subscriber(self.camera_topic, Image, self.ImageCallback, queue_size = 1)


        if self.pub_face_type == "compressed":
            self.pub_face_image = rospy.Publisher('/face_id_image/compressed', CompressedImage, queue_size=1)
        else:
            self.pub_face_image = rospy.Publisher('/face_id_image', Image, queue_size=1)

        self.cvBridge = CvBridge()

          
        self.begin = rospy.get_time()
        while not rospy.is_shutdown():

            if self.cv_image is not None:

                    small_frame = cv2.resize(self.cv_image, (0, 0), fx=0.25, fy=0.25)

                    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
                    rgb_small_frame = small_frame[:, :, ::-1]
                    if process_this_frame:
                        # Find all the faces and face encodings in the current frame of video
                            face_locations = face_recognition.face_locations(rgb_small_frame)
                            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

                            face_names = []
                            for face_encoding in face_encodings:
                                # See if the face is a match for the known face(s)
                                matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                                name = "Unknown"

                                # # If a match was found in known_face_encodings, just use the first one.
                                # if True in matches:
                                #     first_match_index = matches.index(True)
                                #     name = known_face_names[first_match_index]

                                # Or instead, use the known face with the smallest distance to the new face
                                face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                                best_match_index = np.argmin(face_distances)
                                if matches[best_match_index]:
                                    name = known_face_names[best_match_index]

                                face_names.append(name)
                    process_this_frame = not process_this_frame



                    # Display the results
                    for (top, right, bottom, left), name in zip(face_locations, face_names):
                    # Scale back up face locations since the frame we detected in was scaled to 1/4 size
                        top *= 4
                        right *= 4
                        bottom *= 4
                        left *= 4

                        # Draw a box around the face
                        cv2.rectangle(self.cv_image, (left, top), (right, bottom), (0, 0, 255), 2)

                        # Draw a label with a name below the face
                        cv2.rectangle(self.cv_image, (left, bottom - 34), (right, bottom), (0, 0, 255), cv2.FILLED)
                        font = cv2.FONT_HERSHEY_DUPLEX
                        cv2.putText(self.cv_image, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

                    if known_face_names[0] in face_names :
                        self.end = rospy.get_time()
                        if self.end-self.begin > 10:
                            rospy.loginfo("have face, end time: %d", self.end)
                            #self.time_flag = True
                        self.begin = self.end
                        self.time_flag = True

                    elif self.time_flag == True:
                        rospy.loginfo("no face, begin :%d", self.begin)
                        self.begin = rospy.get_time()
                        self.time_flag = False
                    
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
        Detect = ClbrobotFaceDetect()
        rospy.spin()
    except Exception as e:
        rospy.logerr("%s", str(e))
        os._exit(1)

