# ReysenSeniorProject

This is the final product of my senior project for Fall 2025! The MVP of my project is a simulation of an unmanned aerial vehicle (UAV) and/or an unmanned vehicle (UxV) that completes several challenges. This includes the UAV being able to take off, move autonomously to designated areas, communicate with the UxV to complete particular tasks, and land upon challenge completion. 

This repository includes the ROS 2 Foxy workspace that interacts with Gazebo Classic and the ArduPilot flight controller firmware to simulate challenges from the Raytheon Autonomous Vehicle Competition.

There are world files, launch files, and mission logic scripts for the below three challenges:

Challenge 1:
A UAV autonomously searches the field for the target ArUco marker. 

Challenge 2:
A UAV autonomously searches the field for the target ArUco marker and sends a UxV to the target marker's location. 

Challenge 3: 
A UAV preforms a precision landing on an autonomous, moving ground vehicle.

Future Iterations:

A goal that I attempted to accomplish before the course deadline, but unfortunately did not succeed in completing, was to achieve non-GPS based navigation for my challenges. I worked on using VI-SLAM, specifically ORB_SLAM3’s Monocular-Inertial SLAM algorithm, to estimate the position of the drone without requiring the use of GPS. 

SLAM stands for Simultaneous Localization and Mapping, and it is frequently used in autonomous vehicles for navigation. It involves retrieving sensor data from the vehicle–for example, VI-SLAM retrieves data from the inertial measurement unit (IMU) of the vehicle as well as from its visual sensors such as its camera–to build a map of the vehicle’s environment and estimate its position within the map. I was able to get Monocular SLAM working, but without IMU data, I cannot estimate the drone’s true position within the environment. The synchronization of collecting IMU data and camera sensor data was a more difficult task than expected, and thus I did not accomplish this goal before the deadline. Thus, a future iteration for this project would be to successfully utilize a VI-SLAM algorithm  for navigation instead of GPS.
