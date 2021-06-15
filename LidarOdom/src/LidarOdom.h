/**
 * 
 * 
 * 
 * 
 */

#ifndef LIDAR_ODOM_H_
#define LIDAR_ODOM_H_

// C++, third party
#include<iostream>
// 自定义工程
#include"Options.h"
#include"time.h"

class LidarOdom{
  public:
    explicit LidarOdom(LidarOdomOptions& lo_options);
    ~LidarOdom();

    LidarOdom(const LidarOdom&) = delete;
    LidarOdom& operator=(const LidarOdom&) = delete;

    void addImu();
    void addPointcloud();

  private:
    enum WhichMethod {
        NDT = 0,
        FEATURE = 1,
        OTHER = 2
    };

    LidarOdomOptions lo_options_;
    WhichMethod which_method = NDT;
};

#endif // LIDAR_ODOM_H_