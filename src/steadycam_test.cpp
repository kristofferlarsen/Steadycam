//
// Created by Kristoffer Larsen & Asgeir Bjørkedal
// for a semester project in Robotics at NTNU Trondheim
//

#include <steadycam/getRunning.h>
#include <steadycam/setRunning.h>
#include <steadycam/setEulerAngles.h>
#include <steadycam/setControlMode.h>
#include <steadycam/setPoint.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "steadycam/Control.h"

using namespace std;
using namespace ros;
using namespace geometry_msgs;

/*
 * Entry point of the program.
 */
int main(int argc, char **argv) {
    cout << "Starting Node \n\r";

    init(argc, argv, "steadycam_test");
    NodeHandle n;
    Rate loop_rate(60);
    cout << "Setting up service clients \n\r";

    // Setting up service servers to the necessary services
    ServiceClient setRunning_client = n.serviceClient<steadycam::setRunning>("setRunning");
    ServiceClient setControlMode_client = n.serviceClient<steadycam::setControlMode>("setControlMode");
    ServiceClient setEulerAngles_client = n.serviceClient<steadycam::setEulerAngles>("setEulerAngles");
    ServiceClient setPoint_clinet = n.serviceClient<steadycam::setPoint>("setPoint");

    // create service objects
    steadycam::setPoint s_point;
    steadycam::setEulerAngles s_eulerAngles;
    steadycam::setControlMode s_controlMode;
    steadycam::setRunning s_running;
    bool start = true;

    while (ros::ok()) {
        if (start) {
            // Initialize
            cout << "Setting initial values \n\r";
            s_running.request.running = true;
            s_controlMode.request.controlMode = true;
            s_eulerAngles.request.angle_lock.x = 0.0;
            s_eulerAngles.request.angle_lock.y = 0.0;
            s_eulerAngles.request.angle_lock.z = 0.0;
            s_point.request.position_lock.x = 2.0;
            s_point.request.position_lock.y = -1.5;
            s_point.request.position_lock.z = 1.7;

            cout << "Sending values \n\r";
            setRunning_client.call(s_running);
            setControlMode_client.call(s_controlMode);
            setEulerAngles_client.call(s_eulerAngles);
            setPoint_clinet.call(s_point);
            start = false;
        }
        else {
            // Main loop, send commands to the Steadycam gimbal controller node.

            //At this point, this test node is used to start and set initial values for
            //the steadycam node. Nothing is done in the continous loop.
        }
        spinOnce();
        loop_rate.sleep();
    }
    return 0;
}




