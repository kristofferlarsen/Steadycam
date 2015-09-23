#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "steadycam/Control.h"
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace ros;
void imuCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    //on do callback from IMU data
}
bool running = false;

void comfigCallBack(const steadycam::Control::ConstPtr&  msg)
{
    bool running = msg->on_off.data;
    ROS_INFO("%i",running);
}
int main(int argc, char **argv)
{
    init(argc,argv,"steadycam");
    NodeHandle n;
    Publisher pub = n.advertise<geometry_msgs::Twist>("steadycam_servo_control",1000);
    Subscriber sub = n.subscribe("steadycam_imu_data",1000,imuCallback);
    Subscriber comfig_sub = n.subscribe("steadycam_control",1000,comfigCallBack);
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



