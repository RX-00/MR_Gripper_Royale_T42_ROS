# MR_Gripper_Royale_T42_ROS

ROS enabled custom gripper from MassRobotics

NOTE: make sure you got maestro from pololu installed in the proper directories in order to utilize the 6 channel servo controller

TODO: make sure the latency is super low


NOTE: When first compiling, comment out these lines from the CMakelists.txt

find_package(Boost REQUIRED COMPONENTS system) 
find_package(MR_Gripper_Royale_T42_ROS REQUIRED) 

then after the catkin_make fails but has found/linked other packages, then uncomment those lines and recompile
