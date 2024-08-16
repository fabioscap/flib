#pragma once

#include <chrono>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace fodom {
template <typename TimeType>
double time_to_sec(const TimeType& time) {
  static_assert(sizeof(TimeType) == 0,
                "Specialization for this type is not provided");
}

template <>
double time_to_sec(
    const std::chrono::duration<long int, std::ratio<1, 1000000000>>&);

template <typename TimePoint_, typename Duration_>
struct SimpleStopWatch_ {
  static constexpr size_t stack_size = 16;
  SimpleStopWatch_(std::function<TimePoint_()> f) : now_f(f) {
    tics.reserve(stack_size);
  };
  void tic() { tics.push_back(now_f()); }
  std::function<TimePoint_()> now_f;
  Duration_ toc() {
    /*     if (tics.empty()) {
          return -1;
        } */
    TimePoint_ start_time = tics.back();
    tics.pop_back();
    TimePoint_ end_time = now_f();
    return end_time - start_time;
  }

  std::vector<TimePoint_> tics;
};

struct ChronoStopWatch
    : public SimpleStopWatch_<
          std::chrono::high_resolution_clock::time_point,
          std::chrono::duration<long int, std::ratio<1, 1000000000>>> {
  ChronoStopWatch()
      : SimpleStopWatch_(std::chrono::high_resolution_clock::now) {}
};
}  // namespace fodom