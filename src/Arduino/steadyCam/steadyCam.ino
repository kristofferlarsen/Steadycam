
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <DynamixelSerial1.h>

#define timeout_sec 0.5

ros::NodeHandle nh;
double anglex = 0.0;
double angley = 0.0;
double anglez = 0.0;
int cycle_freq = 120; //Hz
int time = 0;
int time_sh = 0;
int last_message = 0;
int movespeed = 200;

void messageCb( const geometry_msgs::Vector3& msg)
{
  anglex = msg.x;
  angley = msg.y;
  anglez = msg.z;
  last_message = millis();
  movespeed = 1023;
}

ros::Subscriber<geometry_msgs::Vector3> sub("steadycam_servo_control",messageCb);
void setup()
{
  Dynamixel.begin(1000000,2);
  nh.initNode();
  nh.subscribe(sub);
  Dynamixel.moveSpeed(1,512,movespeed);
  Dynamixel.moveSpeed(2,512,movespeed);
  Dynamixel.moveSpeed(3,512,movespeed);
}

void loop()
{   
  time = millis();

  if(time-last_message > timeout_sec*1000)
  {
    timeout();
  }
  
  if(time-time_sh > (1000/cycle_freq))
  {
    time_sh = time;
    Dynamixel.moveSpeed(1,modifiedMap(anglex,-2.617993878,2.617993878,0.0,1023.0),movespeed);
    Dynamixel.moveSpeed(2,modifiedMap(angley,2.617993878,-2.617993878,0.0,1023.0),movespeed);
    Dynamixel.moveSpeed(3,modifiedMap(anglez,-2.617993878,2.617993878,0.0,1023.0),movespeed);
  }
  nh.spinOnce();
}

double modifiedMap(double x, double in_min, double in_max, double out_min, double out_max)
{
 double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 temp = (int) (4*temp + .5);
 return (double) temp/4;
}

//on call, set angle velocities to 0 and write to the motors.
void timeout()
{
  //move slowly to zero
  movespeed = 200;
  Dynamixel.moveSpeed(1,512,movespeed);
  Dynamixel.moveSpeed(2,512,movespeed);
  Dynamixel.moveSpeed(3,512,movespeed);
  anglex = 0.0;
  angley = 0.0;
  anglez = 0.0;
}

