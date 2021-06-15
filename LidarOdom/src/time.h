/*
 * 基础设施之：时间；
 * 定义所有时间、时间维护、转化相关的设施，算法层中统一使用本基础设施；
 * 基础设施完全基于c++ chrono模块开发；
 * 基础设施以1970年1月1日0点作为时间纪元epoch，同unix系统；
 * 由于ROS时间戳精确到纳秒，LIVSLAM与ROS对接，同样以纳秒作为最小时间单位；
 */

#ifndef TIME_H_
#define TIME_H_



#include <chrono>
#include <ratio>
#include <ostream>

// 不同平台上，long类型的长度是不一样的；
// 为保证跨平台统一性，定义如下。
using int64 = std::int64_t;

// 公元计时法与Unix计时法的秒数差异。
// Cartographer中采用公元计时法。
// LIVSLAM中采用UnixTimeStamp(UTC)计时法。
constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

using Time_N = std::chrono::nanoseconds;
// using Time_M = std::chrono::milliseconds;

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;

// Convenience functions to create common::Durations.
Duration FromSeconds(double seconds);
Duration FromMilliseconds(int64 milliseconds);

// Returns the given duration in seconds.
double ToSeconds(Duration duration);
double ToSeconds(std::chrono::steady_clock::duration duration);

// Creates a time from a Universal Time Scale.
Time FromUniversal(int64 ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
int64 ToUniversal(Time time);

// For logging and unit tests, outputs the timestamp integer.
std::ostream& operator<<(std::ostream& os, Time time);

// CPU time consumed by the thread so far, in seconds.
// double GetThreadCpuTimeSeconds();



#endif  // TIME_H_
