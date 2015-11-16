#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "steadycam/Control.h"
#include "steadycam/setRunning.h"
#include "steadycam/getRunning.h"
#include "steadycam/setControlMode.h"
#include "steadycam/getControlMode.h"
#include "steadycam/setEulerAngles.h"
#include "steadycam/getEulerAngles.h"
#include "steadycam/setPoint.h"
#include "steadycam/getPoint.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "math.h"


using namespace Eigen;

using namespace std;
using namespace ros;



string control_mode = "";
bool running = false;
double refRoll = 0.0;
double refPitch = 0.0;
double refYaw = 0.0;
double actRoll = 0.0;
double actPitch = 0.0;
double actYaw = 0.0;
Matrix4f T00;
Matrix4f T01;
Matrix4f T12;
Matrix4f T23;
Matrix4f T34;
Matrix4f T45;
Matrix4f T05;
Matrix4f T56;
Matrix4f T67;
Matrix4f T78;
Matrix4f T06;
Matrix4f T07;
Matrix4f T08;
Matrix3f J;

Matrix4f Rot(double x, double y, double z)
{
    Matrix4f a;
    a <<  cos(y)*cos(z), -cos(y)*sin(z), sin(y), 0.0,
         (cos(x)*sin(z))+(cos(z)*sin(x)*sin(y)), (cos(x)*cos(z))-(sin(x)*sin(y)*sin(z)), -cos(y)*sin(x), 0.0,
         (sin(x)*sin(z))-(cos(x)*cos(z)*sin(y)), (cos(z)*sin(x))+(cos(x)*sin(y)*sin(z)), cos(x)*cos(y), 0.0,
         0.0, 0.0, 0.0, 1.0;
    return a;
}

Matrix3f forwardkin(double qh[4], double qhg[3])
{
    // Fill initial matrices
    T00 <<  1.0, 0.0, 0.0, 0.0,
            0.0, -1.0, 0.0, 0.0,
            0.0, 0.0, -1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
    T01 <<  cos(qh[0]), 0.0, sin(qh[0]), cos(qh[0])/40.0,
            sin(qh[0]), 0.0, -cos(qh[0]), sin(qh[0])/40.0,
            0.0, 1.0, 0.0, -2.0/5.0,
            0.0, 0.0, 0.0, 1.0;
    T12 <<  cos(qh[1]), -sin(qh[1]), 0.0, (91.0*cos(qh[1]))/200.0,
            sin(qh[1]), cos(qh[1]), 0.0, (91.0*sin(qh[1]))/200.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
    T23 <<  sin(qh[2]), 0.0, -cos(qh[2]), (7.0*sin(qh[2]))/200,
            -cos(qh[2]), 0.0, -sin(qh[2]), -(7.0*cos(qh[2]))/200,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
    T34 <<  cos(qh[3]), 0.0, -sin(qh[3]), 0.0,
            sin(qh[3]), 0.0, cos(qh[3]), 0.0,
            0.0, -1.0, 0.0, -21.0/50.0,
            0.0, 0.0, 0.0, 1.0;
    T45 <<  0.0, 0.0, 1.0, 0.0,
            1.0, 0.0, 0.0, -59.0/20000.0,
            0.0, 1.0, 0.0, 16939.0/100000.0,
            0.0, 0.0, 0.0, 1.0;
    T05 = T00*T01*T12*T23*T34*T45;
    T56 = Rot(0.0,M_PI/2,qhg[0]);
    T67 = Rot(0.0,-M_PI/2,qhg[1]);
    T78 = Rot(-M_PI/2,0.0,qhg[2]);
    T06 = T05*T56;
    T07 = T06*T67;
    T08 = T07*T78;

    J << T06(0,2),T07(0,2),T08(0,2),
         T06(1,2),T07(1,2),T08(1,2),
         T06(2,2),T07(2,2),T08(2,2);
    return J;
}

void imuCallback(const geometry_msgs::Vector3::ConstPtr& rpy)
{
    //on do callback from IMU data
    actRoll = rpy->x;
    actPitch = rpy->y;
    actYaw = rpy->z;
}

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
    ServiceServer service2_= n.advertiseService("setPoint",setPointCallBack);
    ServiceServer service3 = n.advertiseService("setEulerAngles",setEulerAnglesCallBack);
    ServiceServer service4 = n.advertiseService("setControlMode",setControlModeCallBack);
    ServiceServer service5 = n.advertiseService("getRunning",getRunningCallBack);
    ServiceServer service6 = n.advertiseService("getPoint",getPointCallBack);
    ServiceServer service7 = n.advertiseService("getEulerAngles",getEulerAnglesCallBack);
    ServiceServer service8 = n.advertiseService("getControlMode",getControlModeCallBack);

    Rate loop_rate(1);

    float count = 0.0;
    geometry_msgs::Twist msg;
    while(ros::ok())
    {

        double qh[4] = {0.0, -M_PI/2, M_PI/2, 0.0};
        double qhg[3] = {M_PI/4, -M_PI/8, -M_PI/10};
        Matrix3f a = forwardkin(qh,qhg);
        std::cout << a;
        std::cout << "\n\r\n\r";
        msg.angular.x = actRoll;
        msg.angular.y = actPitch;
        msg.angular.z = actYaw;
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






