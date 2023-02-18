#include "clbrobot_pid/clbrobot_pid_core.h"

ClbrobotPID::ClbrobotPID()
{
}

ClbrobotPID::~ClbrobotPID()
{
}

void ClbrobotPID::publishMessage(ros::Publisher *pub_message)
{
  clb_msgs::PID msg;
  msg.p = p_;
  msg.d = d_;
  msg.i = i_;
  pub_message->publish(msg);
}

void ClbrobotPID::messageCallback(const clb_msgs::PID::ConstPtr &msg)
{
  p_ = msg->p;
  d_ = msg->d;
  i_ = msg->i;

  //echo P,I,D
  ROS_INFO("P: %f", p_);
  ROS_INFO("D: %f", d_);
  ROS_INFO("I: %f", i_);
}

void ClbrobotPID::configCallback(clbrobot_pid::clbrobotPIDConfig &config, double level)
{
  //for PID GUI
  p_ = config.p;
  d_ = config.d;
  i_ = config.i;

}
