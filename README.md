# inertial_alignment
Code to compute the initial attitude from static IMU measurements. This repository will eventually be merged into a full dead-reckoning pipeline to be used as an inertial-aiding thread on a visual-inertial odometry algorithm.



# 静基座状态下初始对准
根据加速度计所测到的重力矢量在b系下的投影计算b系相对于导航系的roll和pitch,顺便根据这两个得到旋转矩阵
