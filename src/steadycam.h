//
// Created by minions on 17.11.15.
//

#ifndef STEADYCAM_STEADYCAM_H
#define STEADYCAM_STEADYCAM_H

#include <eigen3/Eigen/Dense>

struct Forward{
    Eigen::Matrix4f Ti;
    Eigen::Matrix3f Ji;
};

Eigen::Matrix4f Rot(double x, double y, double z);

Forward forwardkin(double qh[4], double qhg[3]);

//double[] invkin(Matrix4f desired, double qh[4], double qhg[3]);

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
