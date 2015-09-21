
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

void imuCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    //on do callback from IMU data
    ROS_INFO("%f",msg->x);
}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"steadycam");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cam_servo_control",1000);
    ros::Subscriber sub = n.subscribe("steadycam_IMU",1000,imuCallback);
    ros::Rate loop_rate(60);

    float count = 0.0;
    geometry_msgs::Vector3 vector;
    geometry_msgs::Twist msg;
    while(ros::ok())
    {
        vector.x = 6.1*sin(count*(2*3.1415/480));
        vector.y = 2.0*sin(count*(2*3.1415/480));
        msg.angular = vector;
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        count = count + 0.1;
        if(count > 480)
        {
            count = 0.0;
        }
    }
    return 0;
}

