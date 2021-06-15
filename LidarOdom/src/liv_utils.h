/**
 * 
 * 
 */

#ifndef LIV_UTILS_H_
#define LIV_UTILS_H_


#include<cmath>
#include<ratio>

// fixed ratio sampler!
class FixedRatioSampler{
  public:
    explicit FixedRatioSampler(double ratio);
    ~FixedRatioSampler();

    FixedRatioSampler(const FixedRatioSampler&) = delete;
    FixedRatioSampler& operator=(const FixedRatioSampler&) = delete;

    bool pulse();
    double currentRatio();

  private:
    double ratio_;
    std::uint64_t num_hits_ = 0;
    std::uint64_t num_pulses_ = 0;
};

#endif // LIV_UTILS_H_
