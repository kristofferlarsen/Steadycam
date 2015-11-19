//
// Created by minions on 17.11.15.
//

#ifndef STEADYCAM_STEADYCAM_H
#define STEADYCAM_STEADYCAM_H

#include <eigen3/Eigen/Dense>
#include "control_msgs/JointTrajectoryControllerState.h"

using namespace Eigen;

struct Forward{
    Matrix4f Ti;
    Matrix3f Ji;
};

Matrix4f Rot(double x, double y, double z);

Forward forwardkin(double qh[4], double qhg[3]);

Vector3f invkin(Matrix4f desired, double qh[4]);

void robotcallback(const control_msgs::JointTrajectoryControllerStateConstPtr& qhmsg);

bool setRunningCallBack(steadycam::setRunning::Request &req,
                        steadycam::setRunning::Response &res);

bool setPointCallBack(steadycam::setPoint::Request &req,
                      steadycam::setPoint::Response &res);

bool setEulerAnglesCallBack(steadycam::setEulerAngles::Request &req,
                            steadycam::setEulerAngles::Response &res);

bool setControlModeCallBack(steadycam::setControlMode::Request &req,
                            steadycam::setControlMode::Response &res);

bool getControlModeCallBack(steadycam::getControlMode::Request &req,
                            steadycam::getControlMode::Response &res);

bool getEulerAnglesCallBack(steadycam::getEulerAngles::Request &req,
                            steadycam::getEulerAngles::Response &res);

bool getPointCallBack(steadycam::getPoint::Request &req,
                      steadycam::getPoint::Response &res);

bool getRunningCallBack(steadycam::getRunning::Request &req,
                        steadycam::getRunning::Response &res);

#endif //STEADYCAM_STEADYCAM_H
