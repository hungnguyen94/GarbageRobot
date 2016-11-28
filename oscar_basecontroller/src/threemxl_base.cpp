#include "oscar_basecontroller/threemxl_base.h"

#define DXLC_SAFE_CALL(call) \
  do { \
    int ret = call; \
    if (ret != DXL_SUCCESS) { \
      std::cout << "Error:" << std::endl << "  " << C3mxl::translateErrorCode(ret) << " (0x" << std::hex << ret << std::dec << ")" << std::endl; \
    } \
  } while (0)

ThreeMxlBase::ThreeMxlBase()
{
    wheelBase = 3.0;
    wheelDiameter = 1.5;
    std::string usbAddress = "/dev/ttyUSB0";
    ThreeMxlBase::init(usbAddress);
}

/**
 * Init the 3mxl motors.
 * @param usbAddress USB address.
 */
void ThreeMxlBase::init(std::string usbAddress)
{
    LxSerial *serialPort = new LxSerial();
    CDxlConfig *leftMotorConfig = new CDxlConfig();
    CDxlConfig *rightMotorConfig = new CDxlConfig();

    serialPort->port_open(usbAddress, LxSerial::RS485_FTDI);
    serialPort->set_speed(LxSerial::S921600);

    wheelBase = 3.0;
    wheelDiameter = 5;

    leftMotor = new C3mxl();
    rightMotor = new C3mxl();

    leftMotor->setSerialPort(serialPort);
    rightMotor->setSerialPort(serialPort);

    leftMotor->setConfig(leftMotorConfig->setID(106));
    rightMotor->setConfig(rightMotorConfig->setID(107));

    while(leftMotor->init(false) != DXL_SUCCESS)
    {
        int result = leftMotor->init(false);
        ROS_ERROR("Failed to init left motor with result %d. Retrying.", result);
        sleep(1);
//        ros::Duration(1.0).sleep();
    }

    while(rightMotor->init(false) != DXL_SUCCESS)
    {
        int result = rightMotor->init(false);
        ROS_ERROR("Failed to init right motor with result %d. Retrying.", result);
        sleep(1);
//        ros::Duration(1.0).sleep();
    }
    ROS_DEBUG("Init motors");

    DXLC_SAFE_CALL(leftMotor->set3MxlMode(SPEED_MODE));
    DXLC_SAFE_CALL(rightMotor->set3MxlMode(SPEED_MODE));

    ROS_DEBUG("Set 3mxl mode to \"SPEED\"");
}

/**
 * Move the base.
 * @param linearX Linear X movement
 * @param angularZ Angular Z movement
 */
void ThreeMxlBase::move(double linearX, double angularZ)
{
    wheelBase = 0.6d;
    wheelDiameter = 0.3d;
    double linearVelocity  = linearX / (wheelDiameter / 2.0);
    double angularVelocity = angularZ * (wheelBase / wheelDiameter);

    double leftVelocity = linearVelocity - angularVelocity;
    double rightVelocity = linearVelocity + angularVelocity;

    ROS_INFO("ThreeMxl base -> leftVelocity: %f, rightVelocity: %f", leftVelocity, rightVelocity);

    DXLC_SAFE_CALL(leftMotor->setSpeed(leftVelocity));
    DXLC_SAFE_CALL(rightMotor->setSpeed(rightVelocity));
}


/**
 * Callback function when a msg is published.
 * @param msg Twist message
 */
void ThreeMxlBase::moveCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    double linearX = msg->linear.x;
    double angularZ = msg->angular.z;
    ROS_INFO("ThreeMxl base -> linear x: %f, angular.z: %f", linearX, angularZ);
    move(linearX, angularZ);
}
