#include "piper_control/safety/joint_limits.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace piper_control {
JointLimitReport check_joint_limits(const MatXd &q, const VecXd &lower,
                                    const VecXd &upper, double tolerance) {
  if (q.rows() < 1 || q.cols() < 1 || lower.size() != q.cols() ||
      upper.size() != q.cols() || !q.allFinite() || !lower.allFinite() ||
      !upper.allFinite() || (lower.array() > upper.array()).any() ||
      !std::isfinite(tolerance) || tolerance < 0)
    throw std::invalid_argument("Invalid joint limits input/bounds");
  JointLimitReport r;
  r.valid_rows.assign(static_cast<std::size_t>(q.rows()), true);
  r.excess_rad = MatXd::Zero(q.rows(), q.cols());
  // 超下限取 lower-q；超上限取 q-upper；范围内为0。不修改输入角度。
  for (Eigen::Index row = 0; row < q.rows(); ++row)
    for (Eigen::Index joint = 0; joint < q.cols(); ++joint) {
      const double excess = std::max(
          {0.0, lower(joint) - q(row, joint), q(row, joint) - upper(joint)});
      if (!std::isfinite(excess))
        throw std::invalid_argument("Limit excess overflow");
      r.excess_rad(row, joint) = excess;
      if (excess > tolerance) {
        r.ok = false;
        r.valid_rows[static_cast<std::size_t>(row)] = false;
        r.violations.push_back({static_cast<std::size_t>(row),
                                static_cast<std::size_t>(joint), excess});
      }
    }
  return r;
}
} // namespace piper_control
