#include <steadycam/getRunning.h>
#include <steadycam/setRunning.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "steadycam/Control.h"

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
    init(argc,argv,"steadycam_test");
    NodeHandle n;
    Rate loop_rate(1);
    ServiceClient setRunning_client = n.serviceClient<steadycam::setRunning>("setRunning");
    steadycam::setRunning srv;
    while(ros::ok())
    {
        srv.request.running = sender;
        sender = !sender;
        setRunning_client.call(srv);
        spinOnce();
        loop_rate.sleep();
    }
    return 0;
}




