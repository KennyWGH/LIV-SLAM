/**
 * 
 * 
 * 
 */

// C++
#include<iostream>
// 自定义
#include"liv_utils.h"




FixedRatioSampler::FixedRatioSampler(double ratio)
    :ratio_(ratio) 
{
    // C++断言检查ratio必须>=0且<=1！
    if( !(0.<=ratio_ && ratio_ <=1.) )
        std::cout << "ERROR! ratio passed to FixedRatioSampler must be within [0,1]!" 
                << std::endl;
}

FixedRatioSampler::FixedRatioSampler(){}

FixedRatioSampler::~FixedRatioSampler(){}

bool FixedRatioSampler::pulse()
{
    num_pulses_++;
    if( static_cast<double>(num_hits_)/num_pulses_ < ratio_ )
    {
        num_hits_++;
        return true;
    }
    return false;
}

double FixedRatioSampler::currentRatio()
{
    return static_cast<double>(num_hits_)/num_pulses_;
}

void FixedRatioSampler::setRatio(double ratio)
{
    ratio_ = ratio;
}
