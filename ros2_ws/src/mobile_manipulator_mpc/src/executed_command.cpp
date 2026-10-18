#include "mobile_manipulator_mpc/executed_command.hpp"

namespace mobile_manipulator_mpc
{
void ExecutedCommand::on_sent(double, const Eigen::Vector2d & vw)
{
  sent_[sent_next_] = vw;
  sent_next_ = (sent_next_ + 1) % kSentKept;
  if (sent_count_ < kSentKept) ++sent_count_;
  latest_sent_ = vw;
}

void ExecutedCommand::on_executed(double t, const Eigen::Vector2d & vw)
{
  executed_ = vw;
  executed_at_ = t;
}

Eigen::Vector2d ExecutedCommand::executed(double now, double max_age_s) const
{
  return now - executed_at_ <= max_age_s ? executed_ : Eigen::Vector2d::Zero();
}

Eigen::Vector2d ExecutedCommand::previous(double now, double max_age_s) const
{
  const Eigen::Vector2d e = executed(now, max_age_s);
  if (now - executed_at_ > max_age_s) return e;
  for (std::size_t i = 0; i < sent_count_; ++i) {
    if ((sent_[i] - e).cwiseAbs().maxCoeff() <= kPassThroughTol) return latest_sent_;
  }
  return e;
}
}  // namespace mobile_manipulator_mpc
