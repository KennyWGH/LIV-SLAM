#include"MotionFilter.h"

MotionFilter::MotionFilter(){}

MotionFilter::MotionFilter(const double& thres_DIST, const double thres_ANG,
                    const double& thres_TIME)
{
    THRES_DIST = thres_DIST;
    THRES_ANG = thres_ANG;
    THRES_TIME = common::Time(common::FromSeconds(thres_TIME));
}

MotionFilter::MotionFilter(const double& thres_DIST, const double thres_ANG)
{
    THRES_DIST = thres_DIST;
    THRES_ANG = thres_ANG;
}

MotionFilter::~MotionFilter(){}

bool MotionFilter::isSimilar(const Rigid3d& input_pose)
{
    // 监控和打印过滤比例
    if (num_total_>0 && (num_total_%500==0) ) {
        std::cout << "Motion filter reduced the number of LiDAR frames to "
            << 100.0*num_saved_/num_total_ << "%" << std::endl;
    }

    // 执行运动过滤
    ++num_total_;
    if (num_total_>1 
        && (input_pose.translation()-last_pose_.translation()).norm()<THRES_DIST
        && getAngle(input_pose*last_pose_.inverse())<THRES_ANG ) 
    {
        return true;
    }
    
    last_pose_ = input_pose;
    ++num_saved_;
    return false;
}