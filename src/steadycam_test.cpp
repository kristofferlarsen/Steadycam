
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "steadycam/Control.h"

using namespace std;
using namespace ros;


// mode-false = angle lock, true = position lock
bool sender = false;

int main(int argc, char **argv)
{
    init(argc,argv,"steadycam_test");
    NodeHandle n;
    Rate loop_rate(1);
    Publisher pub = n.advertise<steadycam::Control>("steadycam_control_input",1000);
    steadycam::Control msg;
    std_msgs::Bool test;
    while(ros::ok())
    {
        test.data = sender;
        sender = !sender;
        msg.on_off = test;
        pub.publish(msg);
        spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



