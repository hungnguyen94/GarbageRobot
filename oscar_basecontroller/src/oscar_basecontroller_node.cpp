#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include "oscar_basecontroller/threemxl_base.h"


int main(int argc, char** argv)
{
    ThreeMxlBase *base = new ThreeMxlBase();
    std::cout << "Init" << std::endl;
    ros::init(argc, argv, "threemxl");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("cmd_vel", 100, &ThreeMxlBase::moveCallback, base);
    std::cout << "sub" << std::endl;

    ros::spin();
    return 0;
}
