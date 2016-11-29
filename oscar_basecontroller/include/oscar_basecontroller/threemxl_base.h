#ifndef OSCAR_BASECONTROLLER_THREEMXL_BASE_H
#define OSCAR_BASECONTROLLER_THREEMXL_BASE_H

#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <threemxl/C3mxl.h>
#include <geometry_msgs/Twist.h>
#include <string>

class ThreeMxlBase
{
protected:
    C3mxl *leftMotor, *rightMotor;
    double wheelDiameter, wheelBase;
protected:
public:
    ThreeMxlBase();
    ~ThreeMxlBase()
    {
        delete leftMotor;
        delete rightMotor;
    }

    void init(std::string usbAddress);

    void move(double linearX, double angularZ);

    void moveCallback(const geometry_msgs::Twist::ConstPtr &msg);
};

#endif //OSCAR_BASECONTROLLER_THREEMXL_BASE_H
