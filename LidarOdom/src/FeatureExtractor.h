/**
 * @author: 王冠华
 * @description: 纯函数类，用于将点云投影为深度图像，并提取所需特征
 * @data: July 2nd, 2021
 */

#ifndef FEATURE_EXTRACTOR_H_
#define FEATURE_EXTRACTOR_H_

// C++
// opencv
#include<opencv/cv.hpp>
// PCL
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

class FeatureExtractor{
  public:
    explicit FeatureExtractor();
    ~FeatureExtractor();

    bool denoiseCloud(); //TODO, 点云预处理：去噪声，移除离群点
    bool getRangeImage(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, cv::Mat& output_img);


  private:
    // 类初始化的时候需要确定的参数
    int N_Scans = 16;
    double angResVert = 2;
    double angOffsetVert = 15.0;
    bool useCloudRing = false;
    //
};




#endif // FEATURE_EXTRACTOR_H_