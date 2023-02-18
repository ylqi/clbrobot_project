#ifndef SR_CLB_PID_CORE_H
#define SR_CLB_PID_CORE_H

#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
#include "clb_msgs/PID.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <clbrobot_pid/clbrobotPIDConfig.h>

class ClbrobotPID
{
public:
  ClbrobotPID();
  ~ClbrobotPID();
  void configCallback(clbrobot_pid::clbrobotPIDConfig &config, double level);
  void publishMessage(ros::Publisher *pub_message);
  void messageCallback(const clb_msgs::PID::ConstPtr &msg);

  double p_;
  double d_;
  double i_;

};
#endif
