#ifndef __UTILITY_HPP__
#define __UTILITY_HPP__

#include "rclcpp/rclcpp.hpp"

class Utility {
public:
  static double StampToSec(builtin_interfaces::msg::Time stamp) {
    double seconds = stamp.sec + (stamp.nanosec * pow(10, -9));
    return seconds;
  }

  static rclcpp::Time SecToStamp(double seconds) {
    int32_t sec, nanosec;
    sec = (int32_t)seconds;
    nanosec = (int32_t)((seconds - sec) * pow(10, 9));
    return rclcpp::Time(sec, nanosec);
  }
};

#endif
