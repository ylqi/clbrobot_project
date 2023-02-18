#ifndef CLB_BASE_H
#define CLB_BASE_H

#include <ros/ros.h>
#include <clb_msgs/Velocities.h>
#include <clb_msgs/Battery.h>
#include <tf/transform_broadcaster.h>

class ClbBase
{
public:
    ClbBase();
    void velCallback(const clb_msgs::Velocities& vel);
    void batteryCallback(const clb_msgs::Battery& bat);


private:
    ros::NodeHandle nh_;
    ros::Publisher odom_publisher_;
    ros::Subscriber velocity_subscriber_;
    ros::Subscriber battery_sub_;
    tf::TransformBroadcaster odom_broadcaster_;

    float linear_scale_;
    float low_battery_;
    float steering_angle_;
    float linear_velocity_x_;
    float linear_velocity_y_;
    float angular_velocity_z_;
    ros::Time last_vel_time_;
    float vel_dt_;
    float x_pos_;
    float y_pos_;
    float heading_;
};

#endif
