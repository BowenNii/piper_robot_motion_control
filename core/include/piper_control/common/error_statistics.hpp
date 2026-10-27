#pragma once
#include <cstddef>
#include <limits>

namespace piper_control {
struct ErrorStatistics {
  std::size_t count = 0;
  double mean = std::numeric_limits<double>::quiet_NaN();
  double rms = std::numeric_limits<double>::quiet_NaN();
  double max = std::numeric_limits<double>::quiet_NaN();
};
// 【STAT-01】非负有限误差逐条add -> count/mean/RMS/max，单位与输入一致。
// 空统计返回NaN而不是0；reset清空。add失败不修改已有统计。
class ErrorAccumulator {
public:
  void add(double error);
  void reset();
  [[nodiscard]] ErrorStatistics statistics() const;

private:
  std::size_t count_ = 0;
  long double sum_ = 0.0L;
  // 缩放平方和，避免直接error*error造成double溢出。
  double scale_ = 0.0;
  long double scaled_square_sum_ = 0.0L;
  double max_ = 0.0;
};
} // namespace piper_control
