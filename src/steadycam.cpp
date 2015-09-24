#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "steadycam/Control.h"
#include "steadycam/setRunning.h"
#include "steadycam/getRunning.h"
#include "steadycam/setControlMode.h"
#include "steadycam/getControlMode.h"
#include "steadycam/setEulerAngles.h"
#include "steadycam/getEulerAngles.h"
#include "steadycam/setPoint.h"
#include "steadycam/getPoint.h"

using namespace std;
using namespace ros;

void imuCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    //on do callback from IMU data
}
bool running = false;
string control_mode = "";

bool setRunningCallBack(steadycam::setRunning::Request &req,
                        steadycam::setRunning::Response &res)
{
    ROS_INFO("I got %i",req.running);
    running = req.running;
    return true;
}

bool setPointCallBack(steadycam::setPoint::Request &req,
                      steadycam::setPoint::Response &res)
{
    return true;
}
bool setEulerAnglesCallBack(steadycam::setEulerAngles::Request &req,
                            steadycam::setEulerAngles::Response &res)
{
    return true;
}
bool setControlModeCallBack(steadycam::setControlMode::Request &req,
                            steadycam::setControlMode::Response &res)
{
    control_mode = req.controlMode;
    return true;
}

bool getControlModeCallBack(steadycam::getControlMode::Request &req,
                            steadycam::getControlMode::Response &res)
{
    res.controlMode = control_mode;
    return true;
}

bool getEulerAnglesCallBack(steadycam::getEulerAngles::Request &req,
                            steadycam::getEulerAngles::Response &res)
{
    return true;
}

bool getPointCallBack(steadycam::getPoint::Request &req,
                      steadycam::getPoint::Response &res)
{
    return true;
}

bool getRunningCallBack(steadycam::getRunning::Request &req,
                        steadycam::getRunning::Response &res)
{
    res.running = running;
    return true;
}


int main(int argc, char **argv)
{
    init(argc,argv,"steadycam");
    NodeHandle n;

    Publisher pub = n.advertise<geometry_msgs::Twist>("steadycam_servo_control",1000);
    Subscriber sub = n.subscribe("steadycam_imu_data",1000,imuCallback);


    ServiceServer service1 = n.advertiseService("setRunning", setRunningCallBack);
    ServiceServer service2_ = n.advertiseService("setPoint",setPointCallBack);
    ServiceServer service3 = n.advertiseService("setEulerAngles",setEulerAnglesCallBack);
    ServiceServer service4 = n.advertiseService("setControlMode",setControlModeCallBack);
    ServiceServer service5 = n.advertiseService("getRunning",getRunningCallBack);
    ServiceServer service6 = n.advertiseService("getPoint",getPointCallBack);
    ServiceServer service7 = n.advertiseService("getEulerAngles",getEulerAnglesCallBack);
    ServiceServer service8 = n.advertiseService("getControlMode",getControlModeCallBack);

    Rate loop_rate(60);

    float count = 0.0;
    geometry_msgs::Vector3 vector;
    geometry_msgs::Twist msg;
    while(ros::ok())
    {
        vector.x = 6.1*sin(count*(2*3.1415/480));
        vector.y = 2.0*sin(count*(2*3.1415/480));
        msg.angular = vector;
        pub.publish(msg);

        spinOnce();
        loop_rate.sleep();
        count = count + 0.1;
        if(count > 480)
        {
            count = 0.0;
        }
    }
    return 0;
}






