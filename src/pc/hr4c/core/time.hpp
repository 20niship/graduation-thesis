#pragma once

#include <chrono>
#include <iostream>

namespace hr4c {

class Timestamp {
public:
  static Timestamp now() {
    auto now       = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    return Timestamp(timestamp);
  }

  explicit Timestamp(long long timestamp) : timestamp_(timestamp) {}
  Timestamp() { *this = Timestamp::now(); }
  // print YYYY-MM-DD HH:MM:SS
  std::string str() const { 
    auto time = std::chrono::system_clock::time_point(std::chrono::milliseconds(timestamp_));
    auto t    = std::chrono::system_clock::to_time_t(time);
    char buf[100];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    return std::string(buf);
  }

private:
  long long timestamp_;
};


} // namespace hr4c
