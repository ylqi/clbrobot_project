/** MIT License
Copyright (c) 2017 Sudarshan Raghunathan
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 *@copyright Copyright 2017 Sudarshan Raghunathan
 *@file   detect.cpp
 *@author Sudarshan Raghunathan
 *@brief  Ros Nod to subscribe to turtlebot images and perform image processing to detect line
 */
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "std_msgs/Float32.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "clbrobot_line_detect.hpp"
#include <geometry_msgs/Twist.h>
#include <unistd.h>

float GetAngular(float offset)
{
    float Kp = 0.006;
    float Kd = 0.0001;
    static float last_error = 0;
    float error = offset;
    if (abs(error) >= 320)
        error = last_error;
    
    float angular_z = Kp*error + Kd*(error - last_error);
    last_error = error;

    if (angular_z > 1.0)
        angular_z = 1.0;
    if (angular_z < -1.0)
        angular_z = -1.0;
    //ROS_INFO("offset:%f, angular_z： %f", error, angular_z);
    return angular_z;
}

int main(int argc, char **argv) {
    // Initializing node and object
    ros::init(argc, argv, "ClbrobotLineFollower");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    float offset, max_speed_ = 0.2;
    int color_ ;
    bool detect_mode_, servo_ctrl_;
    std::string image_topic;
    ClbrobotLineDetect clb_det;
    geometry_msgs::Twist vel;
    //                             yellow         red          green       blue
    cv::Scalar line_lower[] = {{20, 91, 129}, {0, 70, 50}, {35, 43, 46},{100, 80, 46}};
    cv::Scalar line_upper[] = {{34, 255, 255}, {10, 255, 255}, {90, 255, 255}, {124, 255, 255}};
    // Creating Publisher and subscriber
    nh_private.getParam("max_speed", max_speed_);
    nh_private.getParam("color", color_);
    nh_private.getParam("detect_mode", detect_mode_);
    nh_private.getParam("servo_ctrl", servo_ctrl_);
    nh_private.getParam("image_topic", image_topic);
    ros::Subscriber sub = nh.subscribe(image_topic, 1, &ClbrobotLineDetect::imageCallback, &clb_det);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher detect_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    clb_det.SetLineHsvScalar(line_lower[color_], line_upper[color_]);

    if (servo_ctrl_)
        system("rostopic pub -1 servo clb_msgs/Servo -- '105' '85'");

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        if (!clb_det.img.empty()) {
            // Perform image processing
            clb_det.img_filt = clb_det.Gauss(clb_det.img);
            offset = clb_det.colorthresh(clb_det.img_filt);
            if(!detect_mode_){
                vel.angular.z = GetAngular(offset);
                vel.linear.x = max_speed_ *(1.0 - (1.2*fabs(vel.angular.z)));
                ROS_INFO("k:%f, linear_x:%f, angular_z： %f", (1.0 - fabs(vel.angular.z)), vel.linear.x, vel.angular.z);
                vel_pub.publish(vel);
            }
        }
        rate.sleep();
    }
    // Closing image viewer
    cv::destroyWindow("Clbrobot  View");
}
