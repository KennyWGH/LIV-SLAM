/*
 * Copyright 2021 The LIV-SLAM Authors
 */

// ROS
#include <time.h>
// C++
#include <cerrno>
#include <cstring>
#include <string>
//
#include "liv_time.h"

namespace common {

Duration FromSeconds(const double seconds) {
    return std::chrono::duration_cast<Duration>(
        std::chrono::duration<double>(seconds));
}

Duration FromMilliseconds(const int64 milliseconds) {
    return std::chrono::duration_cast<Duration>(
        std::chrono::milliseconds(milliseconds));
}

Duration FromNanoseconds(int64 nanoseconds) {
    return std::chrono::duration_cast<Duration>(
        std::chrono::nanoseconds(nanoseconds));
}

Duration FromMicroseconds(const int64 microseconds) {
    return std::chrono::duration_cast<Duration>(
        std::chrono::microseconds(microseconds));
}

Time FromUnixTicks(const int64 ticks) { return Time(Duration(ticks)); }

int64 ToUnixTicks(const Time& time) { return time.time_since_epoch().count(); }

std::ostream& operator<<(std::ostream& os, const Time time) {
    os << std::to_string(ToUnixTicks(time));
    return os;
}

}  // namespace common
