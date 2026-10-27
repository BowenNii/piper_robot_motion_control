#include "piper_control/common/error_statistics.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace piper_control {
void ErrorAccumulator::add(double value) {
  if (!std::isfinite(value) || value < 0.0)
    throw std::invalid_argument(
        "ErrorAccumulator requires finite nonnegative error");
  if (count_ == std::numeric_limits<std::size_t>::max())
    throw std::overflow_error("ErrorAccumulator count overflow");
  const long double next_sum = sum_ + static_cast<long double>(value);
  if (!std::isfinite(next_sum))
    throw std::overflow_error("ErrorAccumulator sum overflow");
  if (value > 0) {
    // 累计 (value/scale)^2，不直接计算 value^2，避免大数平方溢出。
    // 新最大值出现时，将已累计平方和重新缩放到新的 scale。
    if (value > scale_) {
      const long double ratio = scale_ / value;
      scaled_square_sum_ = 1 + scaled_square_sum_ * ratio * ratio;
      scale_ = value;
    } else {
      const long double ratio = value / scale_;
      scaled_square_sum_ += ratio * ratio;
    }
  }
  sum_ = next_sum;
  max_ = std::max(max_, value);
  ++count_;
}
void ErrorAccumulator::reset() { *this = ErrorAccumulator{}; }
ErrorStatistics ErrorAccumulator::statistics() const {
  if (count_ == 0)
    return {};
  return {count_, static_cast<double>(sum_ / count_),
          static_cast<double>(scale_ * std::sqrt(scaled_square_sum_ / count_)),
          max_};
}
} // namespace piper_control
