# LIV-SLAM
A slam framework with LiDAR, IMU, and Vision(camera).

## front-end design
Imu is used to provide gravity direction for every and each point cloud frame.
Also, imu gives the initial delta orientation between two point cloud frames.
NDT is used to match  consecutive scans.
Feature-based match is also provided to match scans, that's optional.

## back-end disign
coming soon ...
