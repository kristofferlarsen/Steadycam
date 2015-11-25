//
// Created by Kristoffer Larsen & Asgeir Bjørkedal
// for a semester project in Robotics at NTNU Trondheim
//

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
#include "steadycam.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "math.h"


using namespace std;
using namespace ros;
using namespace Eigen;

// Create local variables
bool control_mode = true;
bool running = false;
//Setpoint for "angle-lock" mode
Vector3f angle;
//Setpoint for "position-lock" mode
Vector3f point;
//Matrices for forward kinematics
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
//Geometric Jacobian
Matrix3f J;
//Joint vector for the KUKA KR6 R900 Sixx robot manipulator
Vector4f qh;


/*
 * Callback for topic updates on the robots joint vector. Maps the topic data to a
 * local variable
 */
void robotcallback(const control_msgs::JointTrajectoryControllerStateConstPtr &qhmsg) {
    qh << qhmsg->actual.positions[0],
          qhmsg->actual.positions[1],
          qhmsg->actual.positions[2],
          qhmsg->actual.positions[3];
}

/*
 * Main program entry point
 */
int main(int argc, char **argv) {
    //Initial values
    qh << 0.0, -M_PI/2, M_PI/2, 0.0;
    angle << 0.0, 0.0, 0.0;
    point << 0.0, 0.0, 0.0;

    //Initialize the node with name "steadycam"
    init(argc, argv, "steadycam");
    NodeHandle n;

    // Advertise the steadycam_servo_control topic
    Publisher pub = n.advertise<geometry_msgs::Vector3>("steadycam_servo_control", 1000);

    // Subscribe to the robot joint vector topic
    Subscriber sub = n.subscribe("/ag1/position_trajectory_controller/state", 1, robotcallback);


    //Setting up the necessary service servers
    ServiceServer service1 = n.advertiseService("setRunning", setRunningCallBack);
    ServiceServer service2_ = n.advertiseService("setPoint", setPointCallBack);
    ServiceServer service3 = n.advertiseService("setEulerAngles", setEulerAnglesCallBack);
    ServiceServer service4 = n.advertiseService("setControlMode", setControlModeCallBack);
    ServiceServer service5 = n.advertiseService("getRunning", getRunningCallBack);
    ServiceServer service6 = n.advertiseService("getPoint", getPointCallBack);
    ServiceServer service7 = n.advertiseService("getEulerAngles", getEulerAnglesCallBack);
    ServiceServer service8 = n.advertiseService("getControlMode", getControlModeCallBack);

    //Set the clock speed of the node
    Rate loop_rate(60);
    // Create the message object that is to be sendt to the microcontroller.
    geometry_msgs::Vector3 msg;

    while (ros::ok()) {

        if (running) {
            // Node is in "running" mode
            // Variable to hold the calculated gimbal angles
            Vector3f gimbal_joint;
            // Control mode == True => position mode
            if (control_mode) {
                double robot_angles[4] = {qh(0), qh(1), qh(2), qh(3)};
                double qhg[3] = {0.0, 0.0, 0.0};
                Forward f = forwardkin(robot_angles, qhg);
                Vector3f diff;
                diff << point(0) - f.Ti(0, 3),
                        point(1) - f.Ti(1, 3),
                        point(2) - f.Ti(2, 3);
                double hyp = sqrt(diff(0) * diff(0) + diff(1) * diff(1));
                double yaw;
                double pitch = atan2(diff(2), hyp);
                double roll = 0.0;

                if (diff(1) < 0) {
                    yaw = atan2(-diff(1), diff(0));
                }
                else {
                    yaw = -atan2(diff(1), diff(0));
                }

                gimbal_joint = invkin(Rot(roll, yaw, -pitch), robot_angles);

                msg.x = gimbal_joint(0);
                msg.y = gimbal_joint(1);
                msg.z = gimbal_joint(2);
            }

                // Control mode == False => Angle mode
            else {
                double robot_angles[4] = {qh(0), qh(1), qh(2), qh(3)};
                //calculate gimbal joint vector
                gimbal_joint = invkin(Rot(angle(0), angle(1), angle(2)), robot_angles);
                msg.x = gimbal_joint(0);
                msg.y = gimbal_joint(1);
                msg.z = gimbal_joint(2);
            }
        }
        else {
            //not running, go to 0.0.0
            msg.x = 0.0;
            msg.y = 0.0;
            msg.z = 0.0;
        }
        pub.publish(msg);
        spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

/*
 * Calculates the desired qh vector for the gimbal to reach a desired pose using inverse jacobian
 */
Vector3f invkin(Matrix4f Td, double qh[4]) {
    double qcurrent[3] = {0.0, 0.0, 0.0};
    Matrix4f desired = Rot(-M_PI / 2, 0.0, 0.0) * Td;
    Matrix3f Rd;
    Rd << desired(0, 0), desired(0, 1), desired(0, 2),
          desired(1, 0), desired(1, 1), desired(1, 2),
          desired(2, 0), desired(2, 1), desired(2, 2);

    Forward f;
    for (int i = 0; i < 5; i++) {
        f = forwardkin(qh, qcurrent);
        Matrix3f Rcurrent;
        Rcurrent << f.Ti(0, 0), f.Ti(0, 1), f.Ti(0, 2),
                f.Ti(1, 0), f.Ti(1, 1), f.Ti(1, 2),
                f.Ti(2, 0), f.Ti(2, 1), f.Ti(2, 2);

        Matrix3f Rerror;
        Rerror = Rd * Rcurrent.inverse();
        Vector3f error;
        error << (1.0 / 2.0) * (Rerror(2, 1) - Rerror(1, 2)),
                 (1.0 / 2.0) * (Rerror(0, 2) - Rerror(2, 0)),
                 (1.0 / 2.0) * (Rerror(1, 0) - Rerror(0, 1));

        double k = 1.0;
        Vector3f desiredQ = k * f.Ji.inverse() * error;
        qcurrent[0] = qcurrent[0] + desiredQ(0);
        qcurrent[1] = qcurrent[1] + desiredQ(1);
        qcurrent[2] = qcurrent[2] + desiredQ(2);
    }
    Vector3f out;
    out << qcurrent[0],
           qcurrent[1],
           qcurrent[2];
    return out;
}

/*
 * Calculates the pose of the end effector (gimbal) and the Geometric Jacobian for this pose.
 */
Forward forwardkin(double qh[4], double qhg[3]) {
    // Fill initial matrices
    T00 << 1.0, 0.0, 0.0, 0.0,
            0.0, -1.0, 0.0, 0.0,
            0.0, 0.0, -1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

    T01 << cos(qh[0]), 0.0, sin(qh[0]), cos(qh[0]) / 40.0,
            sin(qh[0]), 0.0, -cos(qh[0]), sin(qh[0]) / 40.0,
            0.0, 1.0, 0.0, -2.0 / 5.0,
            0.0, 0.0, 0.0, 1.0;

    T12 << cos(qh[1]), -sin(qh[1]), 0.0, (91.0 * cos(qh[1])) / 200.0,
            sin(qh[1]), cos(qh[1]), 0.0, (91.0 * sin(qh[1])) / 200.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

    T23 << sin(qh[2]), 0.0, -cos(qh[2]), (7.0 * sin(qh[2])) / 200,
            -cos(qh[2]), 0.0, -sin(qh[2]), -(7.0 * cos(qh[2])) / 200,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

    T34 << cos(qh[3]), 0.0, -sin(qh[3]), 0.0,
            sin(qh[3]), 0.0, cos(qh[3]), 0.0,
            0.0, -1.0, 0.0, -21.0 / 50.0,
            0.0, 0.0, 0.0, 1.0;

    T45 << 0.0, 0.0, 1.0, 0.0,
            1.0, 0.0, 0.0, -59.0 / 20000.0,
            0.0, 1.0, 0.0, 16939.0 / 100000.0,
            0.0, 0.0, 0.0, 1.0;

    T05 = T00 * T01 * T12 * T23 * T34 * T45;
    T56 = Rot(0.0, M_PI / 2, qhg[0]);
    T67 = Rot(0.0, -M_PI / 2, qhg[1]);
    T78 = Rot(-M_PI / 2, 0.0, qhg[2]);
    T06 = T05 * T56;
    T07 = T06 * T67;
    T08 = T07 * T78;

    //Create geometric Jacobian for the gimbal pose
    J << T06(0, 2), T07(0, 2), T08(0, 2),
         T06(1, 2), T07(1, 2), T08(1, 2),
         T06(2, 2), T07(2, 2), T08(2, 2);

    Forward f;
    f.Ti = T08;
    f.Ji = J;
    return f;
}

/*
 * Creates a rotation matrix based on the given input (used for kinematics)
 */
Matrix4f Rot(double x, double y, double z) {
    Matrix4f a;
    a << cos(y) * cos(z), -cos(y) * sin(z), sin(y), 0.0,
            (cos(x) * sin(z)) + (cos(z) * sin(x) * sin(y)), (cos(x) * cos(z)) - (sin(x) * sin(y) * sin(z)), -cos(y) *
                                                                                                            sin(x), 0.0,
            (sin(x) * sin(z)) - (cos(x) * cos(z) * sin(y)), (cos(z) * sin(x)) + (cos(x) * sin(y) * sin(z)), cos(x) *
                                                                                                            cos(y), 0.0,
            0.0, 0.0, 0.0, 1.0;
    return a;
}

/*
 * Callback for service "setRunning"
 * Used to set the running state of the node
 */
bool setRunningCallBack(steadycam::setRunning::Request &req,
                        steadycam::setRunning::Response &res) {
    running = req.running;
    return true;
}

/*
 * Callback for service "setPoint"
 * Used to set the reference point in space for the gimbal to track
 */
bool setPointCallBack(steadycam::setPoint::Request &req,
                      steadycam::setPoint::Response &res) {
    point << req.position_lock.x,
            req.position_lock.y,
            req.position_lock.z;
    return true;
}

/*
 * Callback for service "setEulerAngles"
 * Used to set the reference angles for the gimbal to hold
 */
bool setEulerAnglesCallBack(steadycam::setEulerAngles::Request &req,
                            steadycam::setEulerAngles::Response &res) {
    angle << req.angle_lock.x,
            req.angle_lock.y,
            req.angle_lock.z;
    return true;
}

/*
 * Callback for service "setControlMode"
 * Used to set the control mode (fixed angle or tracking point)
 */
bool setControlModeCallBack(steadycam::setControlMode::Request &req,
                            steadycam::setControlMode::Response &res) {
    control_mode = req.controlMode;
    return true;
}

/*
 * Callback for service "getControlMode"
 * Used to report current control mode
 */
bool getControlModeCallBack(steadycam::getControlMode::Request &req,
                            steadycam::getControlMode::Response &res) {
    res.controlMode = control_mode;
    return true;
}

/*
 * Callback for service "getEulerAngles"
 * Used to report current Euler angles
 */
bool getEulerAnglesCallBack(steadycam::getEulerAngles::Request &req,
                            steadycam::getEulerAngles::Response &res) {
    res.angle_lock.x = angle(0);
    res.angle_lock.y = angle(1);
    res.angle_lock.z = angle(2);
    return true;
}

/*
 * Callback for service "getPoint"
 * Used to report current tracking point in space
 */
bool getPointCallBack(steadycam::getPoint::Request &req,
                      steadycam::getPoint::Response &res) {
    res.position_lock.x = point(0);
    res.position_lock.y = point(1);
    res.position_lock.z = point(2);
    return true;
}

/*
 * Callback for service "getRunning"
 * Used to report current running status
 */
bool getRunningCallBack(steadycam::getRunning::Request &req,
                        steadycam::getRunning::Response &res) {
    res.running = running;
    return true;
}




