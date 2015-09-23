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

// mode-false = angle lock, true = position lock


void comfigCallBack(const steadycam::Control::ConstPtr&  msg)
{
    bool running = msg->on_off.data;
    if(running)
    {
        ROS_INFO("%i",running);
    }
    /*
    //read the config string, type of control and
    string input = msg->data.c_str();
    vector<std::string> strs;
    boost::split(strs, input, boost::is_any_of("@"));
    if(strs.at(0) == "off")
    {
        ROS_INFO("The controller is off");
        running = false;
    }
    else if(strs.at(0) == "on")
    {
        ROS_INFO("The controller is on");
        running = true;
    }
    if(strs.at(1) == "angle_lock")
    {
        ROS_INFO("Running in angle_lock mode");
    }
    else if(strs.at(1) == "position_lock")
    {
        ROS_INFO("Running in position_lock mode");
    }

    vector<std::string> pos;
    boost::split(pos,strs.at(2),boost::is_any_of(","));
    ostringstream os;
    os << "Control paramter, x:" << pos.at(0) << ", y: " << pos.at(1)<< ", z: "<<pos.at(2);
    string control = os.str();
    ROS_INFO("%s",control.c_str());
     */
}
int main(int argc, char **argv)
{
    init(argc,argv,"steadycam");
    NodeHandle n;
    Publisher pub = n.advertise<geometry_msgs::Twist>("steadycam_servo_control",1000);
    Subscriber sub = n.subscribe("steadycam_imu_data",1000,imuCallback);
    Subscriber comfig_sub = n.subscribe("steadycam_control_input",1000,comfigCallBack);
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



