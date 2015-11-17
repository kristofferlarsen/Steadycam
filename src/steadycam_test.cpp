#include <steadycam/getRunning.h>
#include <steadycam/setRunning.h>
#include <steadycam/setEulerAngles.h>
#include <steadycam/setControlMode.h>
#include <steadycam/setEulerAngles.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "steadycam/Control.h"
#include "math.h"

using namespace std;
using namespace ros;
using namespace geometry_msgs;


// mode-false = angle lock, true = position lock
bool sender = false;
int seq = 0;


//builds the message that is to be sendt to the arduino servo controller.
steadycam::Control buildPackage(bool on_off,string control_type,double angle_lock[], double position_lock[])
{
    steadycam::Control msg;
    msg.on_off.data = on_off;
    msg.control_type = control_type;
    msg.angle_lock.header.stamp = ros::Time::now();
    msg.angle_lock.header.seq = seq;
    msg.angle_lock.quaternion.x = angle_lock[0];
    msg.angle_lock.quaternion.y = angle_lock[1];
    msg.angle_lock.quaternion.z = angle_lock[2];
    msg.position_lock.header.stamp = ros::Time::now();
    msg.position_lock.header.seq = seq;
    msg.position_lock.point.x = position_lock[0];
    msg.position_lock.point.y = position_lock[1];
    msg.position_lock.point.z = position_lock[2];
    seq += 1;
    return msg;
}


int main(int argc, char **argv)
{
    cout << "Starting Node \n\r";
    bool start = true;
    double counter = 0.0;
    init(argc,argv,"steadycam_test");
    NodeHandle n;
    Rate loop_rate(60);
    cout << "Setting up service clients \n\r";
    ServiceClient setRunning_client = n.serviceClient<steadycam::setRunning>("setRunning");
    ServiceClient setControlMode_client = n.serviceClient<steadycam::setControlMode>("setControlMode");
    ServiceClient setEulerAngles_client = n.serviceClient<steadycam::setEulerAngles>("setEulerAngles");
    steadycam::setEulerAngles s_eulerAngles;
    steadycam::setControlMode s_controlMode;
    steadycam::setRunning s_running;

    double scale = M_PI/16.0;
    while(ros::ok())
    {
        if(start){
            cout << "Setting initial values \n\r";
            s_running.request.running = true;
            s_controlMode.request.controlMode = false;
            s_eulerAngles.request.angle_lock.x = 0.0;
            s_eulerAngles.request.angle_lock.y = 0.0;
            s_eulerAngles.request.angle_lock.z = 0.0;
            cout << "Sending values \n\r";
            setRunning_client.call(s_running);
            setControlMode_client.call(s_controlMode);
            setEulerAngles_client.call(s_eulerAngles);
            start = false;
        }
        else{
            //s_eulerAngles.request.angle_lock.x = scale*sin(counter/(2.0*M_PI));
            //s_eulerAngles.request.angle_lock.y = scale*sin(counter/(2.0*M_PI));
            s_eulerAngles.request.angle_lock.z = scale*sin(counter/(2.0*M_PI));
            setEulerAngles_client.call(s_eulerAngles);
        }

        spinOnce();
        loop_rate.sleep();
        counter = counter +0.8;
    }
    return 0;
}




