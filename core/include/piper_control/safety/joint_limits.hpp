#pragma once
#include "piper_control/common/types.hpp"
#include <cstddef>
#include <vector>

namespace piper_control {
struct JointLimitViolation {
  std::size_t row = 0;     // 0起始样本索引
  std::size_t joint = 0;   // 0起始关节索引
  double excess_rad = 0.0; // 相对原始边界的超限量，不扣除容差
};
struct JointLimitReport {
  bool ok = true;
  std::vector<bool> valid_rows;
  MatXd excess_rad; // N×n，边界内为0，超边界为正数
  std::vector<JointLimitViolation> violations;
};
// 【SAFE-02】N×n角度rad、n维上下限、非负容差rad -> 超限行/关节/超限量。
// 不交换反置边界、不裁剪角度、不发送指令。非法维度/NaN/Inf/边界抛invalid_argument。
[[nodiscard]] JointLimitReport check_joint_limits(const MatXd &q_rad,
                                                  const VecXd &lower_rad,
                                                  const VecXd &upper_rad,
                                                  double tolerance_rad = 0.0);
} // namespace piper_control
