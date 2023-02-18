#include "clbrobot_pid/clbrobot_pid_core.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pid_listener");
  ros::NodeHandle nh;

  int rate;

  ros::NodeHandle pnh("~");
  pnh.param("rate", rate, int(40));

  ClbrobotPID *clbrobot_pid = new ClbrobotPID();

  ros::Subscriber sub_message = nh.subscribe("pid", 1000, &ClbrobotPID::messageCallback, clbrobot_pid);

  ros::Rate r(rate);

  // Main loop.
  while (nh.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
