#pragma once
// A transport delay in time: values pushed at t become current at t + delay; the latest due value holds.
#include <deque>
#include <optional>
#include <utility>

namespace mobile_manipulator_plant
{
template <class T>
class DelayQueue
{
public:
  explicit DelayQueue(double delay_s) : delay_(delay_s) {}
  void push(double t, T value) { pending_.emplace_back(t + delay_, std::move(value)); }
  // The latest value due by `t` (nullopt before the first is due); `due_at` its arrival time.
  std::optional<T> at(double t, double * due_at = nullptr)
  {
    while (!pending_.empty() && pending_.front().first <= t + 1e-12) {
      current_due_ = pending_.front().first;
      current_ = std::move(pending_.front().second);
      pending_.pop_front();
    }
    if (due_at) *due_at = current_due_;
    return current_;
  }
  void clear()
  {
    pending_.clear();
    current_.reset();
  }
  double delay() const { return delay_; }

private:
  double delay_;
  std::deque<std::pair<double, T>> pending_;
  std::optional<T> current_;
  double current_due_ = 0.0;
};
}  // namespace mobile_manipulator_plant
