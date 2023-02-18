#include <ros/ros.h>
#include "clb_base.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "clb_base_node");
    ClbBase clb;
    ros::spin();
    return 0;
}
