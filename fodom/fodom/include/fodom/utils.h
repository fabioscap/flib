#pragma once

#include <chrono>
#include <functional>
#include <vector>

namespace fodom {
template <typename TimeType>
double time_to_sec(const TimeType& time) {
  static_assert(sizeof(TimeType) == 0,
                "Specialization for this type is not provided");
  return 0.0;
}

template <>
double time_to_sec(
    const std::chrono::duration<long int, std::ratio<1, 1000000000>>&);

struct StopWatchBase {
  virtual void tic() = 0;
  virtual double toc() = 0;
};

template <typename TimePoint_>
struct StopWatch_ : public StopWatchBase {
  static constexpr size_t stack_size = 16;
  StopWatch_(std::function<TimePoint_()> f) : now_f(f) {
    tics.reserve(stack_size);
  };
  void tic() override { tics.push_back(now_f()); }
  std::function<TimePoint_()> now_f;
  double toc() override {
    /* if (tics.empty()) {
      throw std::runtime_error("");
    } */
    TimePoint_ start_time = tics.back();
    tics.pop_back();
    TimePoint_ end_time = now_f();
    return time_to_sec(end_time - start_time);
  }

  std::vector<TimePoint_> tics;
};

struct ChronoStopWatch
    : public StopWatch_<std::chrono::steady_clock::time_point> {
  ChronoStopWatch() : StopWatch_(std::chrono::steady_clock::now) {}
};
}  // namespace fodom