/**
 * 
 */
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Imu.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_broadcaster.h>


#include<sstream>

#include<Eigen/Core>

#include"ImuTracker.h"

ImuTracker imu_tracker(5.0);

ros::Subscriber subImu;
ros::Publisher pubPoseRot;
ros::Publisher pubPoseRotNoG;
nav_msgs::Odometry trackedOrient;
nav_msgs::Odometry trackedOrientNoG;


std::size_t num_msg = 0;

void imuCallback(const sensor_msgs::ImuConstPtr& ros_imu)
{
    num_msg++;
    ImuData imu_data {common::Time( common::FromSeconds(double(ros_imu->header.stamp.sec))+
                                    common::FromNanoseconds(ros_imu->header.stamp.nsec) ),
                    Eigen::Vector3d(ros_imu->linear_acceleration.x,
                                    ros_imu->linear_acceleration.y,
                                    ros_imu->linear_acceleration.z),
                    Eigen::Vector3d(ros_imu->angular_velocity.x,
                                    ros_imu->angular_velocity.y,
                                    ros_imu->angular_velocity.z) };
    imu_tracker.addImu(imu_data);
    Eigen::Quaterniond quat_orient = imu_tracker.orientation();
    Eigen::Quaterniond quat_orient_NoG = imu_tracker.orientationWithoutG();

    // TODO, publish pose from ImuTracker.
    static tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform tf_g;
    tf_g.stamp_ = ros_imu->header.stamp;
    tf_g.frame_id_ = "map";
    tf_g.child_frame_id_ = "imu_G";
    tf_g.setRotation(tf::Quaternion(
        quat_orient.x(),quat_orient.y(),quat_orient.z(),quat_orient.w() ));
    tf_g.setOrigin(tf::Vector3(0, 0, 0));
    tf_broadcaster.sendTransform(tf_g);

    tf::StampedTransform tf_Nog;
    tf_Nog.stamp_ = ros_imu->header.stamp;
    tf_Nog.frame_id_ = "map";
    tf_Nog.child_frame_id_ = "imu_NoG";
    tf_Nog.setRotation(tf::Quaternion(
        quat_orient_NoG.x(),quat_orient_NoG.y(),
        quat_orient_NoG.z(),quat_orient_NoG.w() ));
    tf_Nog.setOrigin(tf::Vector3(0, -2, 0));
    tf_broadcaster.sendTransform(tf_Nog);

    return;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ImuTracker_Test");
    ros::NodeHandle nh;

    subImu = nh.subscribe("/imu/data", 100, imuCallback);
    pubPoseRot = nh.advertise<nav_msgs::Odometry>("/orientation", 100);
    pubPoseRotNoG = nh.advertise<nav_msgs::Odometry>("/orientationNoG", 100);

    ros::spin(); 

    return 0;
}