# Steadycam

The goal for this project was to construct and control a 3 DOF (Degrees of freedom) gimbal that was to be mounted to a robot manipulator.
The control of the gimbal should be integrated in ROS (Robot operating system).

Our implementation gives the user two modes of operation,
Angle lock - The orientation of the gimbal is locked in operational space according to the users input (angles)
Position lock - The orientation of the gimbal is tracking a given point in operational space according to the users input (point in space)

Our prototype uses a Arduino MEGA 2560 to communicate with three Dynamixel AX18-A servo motors. The microcontroller uses the ros.h library, giving us the ability
start the arduino as a node in ros and communicate with other ros nodes using topics.

The orientation of the gimbal is calculated using forward kinematics for the robot manipulator (the actual joint angles for the manipulator
is read by the node and used to calculate the position and orientation in space of the gimbal). The gimbals angles is then solved using
inverse kinematics (the inverse Jacobian approach). For our project, this produced a good result for a wide range of motions.

This project depends on:

    -Eigen3 (matrix calculation)

    -Rosserial (ros-arduino communication)

   -DynamixelSerial1 (arduino-dynamixel communication)
