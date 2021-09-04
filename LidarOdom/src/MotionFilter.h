/**
 * TODO: 文件/代码说明
 */

#ifndef MOTION_FILTER_H_
#define MOTION_FILTER_H_

#include<string>
#include<math.h>

#include"RigidTransform.h"
#include"liv_time.h"
#include"liv_utils.h"

class MotionFilter {
  public:
    MotionFilter();
    MotionFilter(const double& thres_DIST, const double thres_ANG,
                    const double& thres_TIME);
    MotionFilter(const double& thres_DIST, const double thres_ANG);
    ~MotionFilter();

    // void isSimilar(common::Time& input_time, Rigid3d& input_pose);
    bool isSimilar(const Rigid3d& input_pose);

  private:
    common::Time THRES_TIME = common::Time(common::FromSeconds(100.0));
    double THRES_DIST = 0.2;
    double THRES_ANG = 2.0/180.0*M_PI;
    std::size_t num_total_ = 0;
    std::size_t num_saved_ = 0;
    common::Time last_time_;
    Rigid3d last_pose_;


}



#endif // MOTION_FILTER_H_