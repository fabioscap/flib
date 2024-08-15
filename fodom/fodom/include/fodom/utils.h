#pragma once

#include <chrono>
#include <functional>
#include <stack>
#include <string>

namespace fodom {
template <typename Scalar_>
struct SimpleStopWatch {
  SimpleStopWatch() : now_f([]() { return Scalar_{0.0}; }) {};
  SimpleStopWatch(std::function<double()> f) : now_f(f) {};
  void tic() { tics.push(now_f()); }
  Scalar_ toc() {
    if (tics.empty()) {
      return -1;
    }
    Scalar_ start_time = tics.top();
    tics.pop();
    Scalar_ end_time = now_f();
    return end_time - start_time;
  }
  std::function<Scalar_()> now_f;
  std::stack<Scalar_> tics;
};
}  // namespace fodom