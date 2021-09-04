
// C++
#include<cmath>
// 
#include"FeatureExtractor.h"

using namespace std;

FeatureExtractor::FeatureExtractor()
{
    //
}

FeatureExtractor::~FeatureExtractor() {}


// 形参output_img必须在输入前指定其行数和列数；
// 函数将依据行列数确定角度分辨率，并按角度将深度值填入image相应位置；
// 思考一个问题：填入Mat的应该是range值还是depth值呢？特征点需要哪种表达？
// 如果需要range值，请在可视化的时候转换为深度值
bool FeatureExtractor::getRangeImage(
            const pcl::PointCloud<PointType>& input_cloud, cv::Mat& output_img)
{
    if (output_img.type()!=CV_32FC1) {
        cout << "ERROR! range image type must be CV_32FC1, please re-check." << endl;
        return false;
    }

    double angResHori = 360.0/output_img.cols;
    size_t rowIdx, colIdx;
    double angHori, angVert;
    double thisX, thisY, thisZ;
    size_t cloudSize = input_cloud.size();
    size_t count = 0;
    double temp_depth_sum = 0;
    for (size_t i=0; i<cloudSize; i++) 
    {
        thisX = input_cloud.points[i].x;
        thisY = input_cloud.points[i].y;
        thisZ = input_cloud.points[i].z;
        double depth = sqrt(thisX*thisX + thisY*thisY);
        double range = sqrt(thisX*thisX + thisY*thisY + thisZ*thisZ);
        angVert = atan2(thisZ, depth)*180/M_PI;
        angHori = (-thisY>0 ? atan2(-thisY, thisX) : (atan2(-thisY, thisX)+2*M_PI))*180/M_PI;
        rowIdx = round((angVert+angOffsetVert)/2); // index starts from 0 to 15
        colIdx = round(angHori/angResHori)-1;

        // if (thisY>0 ){
        //     cout << "angResHori: " << angResHori << endl;
        //     cout << "X,Y,Z: " << thisX << ", " << thisY << ", " << thisZ << endl;
        //     cout << "atan2(-thisY, thisX): " << atan2(-thisY, thisX) << endl;
        //     cout << "angVert & angHori: " << angVert << ", " << angHori << endl;
        //     cout << "rowIds & colIdx: " << rowIdx << ", " << colIdx << endl;
        // }

        temp_depth_sum+=depth;

        if (rowIdx<0 || rowIdx>N_Scans-1 || colIdx<0 || colIdx>(output_img.cols-1))
        {
            continue;
        }
        // output_img.at<float>(rowIdx,colIdx) = range>100?100:range;
        output_img.at<float>(rowIdx,colIdx) = depth>100?100:depth;
        count++;
    }
    // cout << "FeatureExtractor::getRangeImage() -- Projected points/All: " << count << "/" << cloudSize
    //         << "  |  Average depth: " << temp_depth_sum/cloudSize << endl;
    return true;
}