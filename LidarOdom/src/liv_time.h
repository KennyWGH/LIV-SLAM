/*
 * 基础设施：时间
 * 定义所有时间、时间维护、转化相关的设施，算法层中统一使用本基础设施；
 * 基础设施完全基于c++ chrono模块；
 * 基础设施以1970年1月1日0点作为时间纪元epoch，同unix系统；
 * ROS时间戳以"int32 header.stamp.sec, int32 header.stamp.nsec"计时；
 * LIVSLAM与ROS对接，同样以秒和纳秒作为计时单位；
 */

#ifndef LIV_TIME_H_
#define LIV_TIME_H_

#include <chrono>
#include <ratio>
#include <ostream>

namespace common {

// 本来，c++没有int8/32/63这样的类型，只有int和long等修饰的int
// 但是，不同平台上，long对应的长度是不一样的；
// 为保证跨平台统一性，定义如下。
using int64 = std::int64_t;

// custom clock
struct UnixTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 1000000000>; /* 纳秒 */
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UnixTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Unix Time Scale durations and timestamps which are 64-bit
// integers representing nanosecond ticks since the Unix Epoch.
using Duration = UnixTimeScaleClock::duration;
using Time = UnixTimeScaleClock::time_point;

// Convenience functions to create common::Durations.
Duration FromSeconds(double seconds);
Duration FromMilliseconds(int64 milliseconds);
Duration FromNanoseconds(int64 nanoseconds);
/* PCL pointcloud header time stamp is in microsenond. */
Duration FromMicroseconds(int64 microseconds); 

// Creates a time from a Unix Time Scale.
Time FromUnixTicks(int64 ticks);

// Outputs the Unix Time Scale timestamp for a given Time.
int64 ToUnixTicks(const Time& time);

// For logging and unit tests, outputs the timestamp integer as nanosecond.
std::ostream& operator<<(std::ostream& os, Time time);

// Returns the given duration in seconds.
inline double ToSeconds(const Duration& duration){
    return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
        .count();}

inline double ToSeconds(const Time& time){
    return std::chrono::duration_cast<std::chrono::duration<double>>(time.time_since_epoch())
        .count();}

inline int64 GetSecFromMicro(std::uint64_t micro_sec){
    return int64(micro_sec/1000000.0);}

// 本函数从微妙数#########转化出的纳秒数不等于######000，转换误差为+-1微妙
inline int64 GetNanoFromMicro(std::uint64_t micro_sec){
    return int64
    ((micro_sec/1000000.0 - int64_t(micro_sec/1000000.0))*1000000000);}

}  // namespace common

#endif  // LIV_TIME_H_
