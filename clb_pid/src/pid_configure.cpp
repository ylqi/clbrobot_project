#include "clbrobot_pid/clbrobot_pid_core.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pid_configure");
  ros::NodeHandle nh;

  ClbrobotPID *clbrobot_pid = new ClbrobotPID();

  dynamic_reconfigure::Server<clbrobot_pid::clbrobotPIDConfig> dr_srv;
  dynamic_reconfigure::Server<clbrobot_pid::clbrobotPIDConfig>::CallbackType cb;
  cb = boost::bind(&ClbrobotPID::configCallback, clbrobot_pid, _1, _2);
  dr_srv.setCallback(cb);

  double p;
  double d;
  double i;
  int rate;

  ros::NodeHandle pnh("~");
  pnh.param("p", p, 0.1);
  pnh.param("d", d, 0.10);
  pnh.param("i", i, 0.10);
  pnh.param("rate", rate, 1);

  ros::Publisher pub_message = nh.advertise<clb_msgs::PID>("pid", 10);

  ros::Rate r(rate);

  while (nh.ok())
  {
    clbrobot_pid->publishMessage(&pub_message);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
