
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <DynamixelSerial1.h>

#define maxRpm 59.0
#define pi 3.1415
#define timeout_sec 0.5

ros::NodeHandle nh;
float angular_x = 0.0;
float angular_y = 0.0;
float angular_z = 0.0;
int cycle_freq = 40; //Hz
int time = 0;
int time_sh = 0;
int last_message = 0;

void messageCb( const geometry_msgs::Twist& msg)
{
  angular_x = msg.angular.x;
  angular_y = msg.angular.y;
  angular_z = msg.angular.z;
  last_message = millis();
}

ros::Subscriber<geometry_msgs::Twist> sub("steadycam_servo_control",messageCb);
geometry_msgs::Quaternion quaternion;
ros::Publisher chatter("steadycam_imu_data",&quaternion);


void setup()
{
  Dynamixel.begin(1000000,2);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  Dynamixel.setEndless(1,ON);
  Dynamixel.setEndless(2,ON);
  Dynamixel.setEndless(3,ON);
}

void loop()
{
  time = millis();
  quaternion.x = angular_to_rpm(angular_x);
  quaternion.y = angular_y;
  if(time-last_message > timeout_sec*1000)
  {
    timeout();
  }
  
  if(time-time_sh > (1000/cycle_freq) )
  {
    //har vi fått ny IMU data?
    //hvis ja, send ny imu data
    //do-actions like "send data"
    chatter.publish(&quaternion);
    time_sh = time;
    Dynamixel.turn(1,sign(angular_x),angular_to_rpm(angular_x));
    Dynamixel.turn(2,sign(angular_y),angular_to_rpm(angular_y));
    Dynamixel.turn(3,sign(angular_z),angular_to_rpm(angular_z));
  }  
  nh.spinOnce();
}

//returns a value from 0-1023 representing the motor speed mapped to the maximum speed of the motor
int angular_to_rpm(double angular)
{
  return (int)abs((((angular/(2.0*pi))*60.0)/(maxRpm/1024.0)));
}

//reutrn the sign of a double (positive = true, negative = false)
boolean sign(double a_vel)
{
  if(a_vel >= 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

//on call, set angle velocities to 0 and write to the motors.
void timeout()
{
  angular_x = 0.0;
  angular_y = 0.0;
  angular_z = 0.0;
  Dynamixel.turn(1,true,0);
  Dynamixel.turn(1,true,0);
  Dynamixel.turn(1,true,0);
}
