#pragma once
#include "piper_control/common/types.hpp"
#include "piper_control/safety/joint_limits.hpp"
#include "piper_control/validation/pose_error.hpp"
#include <cstddef>
#include <vector>

namespace piper_control {
struct IkValidationConfig {
  double position_tolerance_m = 1e-4;
  double rotation_tolerance_rad = 1e-3;
  double limit_tolerance_rad = 0.0;
};
struct IkValidationReport {
  bool ok = false;
  Mat4 T_fk = Mat4::Identity();
  PoseError error;
  JointLimitReport limits;
};
// 【VAL-03】单个IK解 ->
// FK回代残差与限位报告。不求解IK；调用者还须检查求解器success。 S_list
// 6×n，M/target 4×4，q/上下限 n维。格式/参数错误抛invalid_argument。
[[nodiscard]] IkValidationReport
check_ik_solution(const MatXd &S_list, const Mat4 &M, const VecXd &q_rad,
                  const Mat4 &target, const VecXd &lower_rad,
                  const VecXd &upper_rad,
                  const IkValidationConfig &config = {});

struct JointJump {
  std::size_t row = 0;   // 0起始索引：发生跳变的后一行
  std::size_t joint = 0; // 0起始关节索引
  double delta_rad = 0.0;
};
struct JointContinuityReport {
  bool ok = true;
  MatXd delta_rad; // (N-1)×n，后一行减前一行，不做2pi绕回
  std::vector<JointJump> jumps;
};
// 【VAL-04】N×n关节序列与每关节相邻变化上限rad -> 跳变报告。
// 阈值不是速度；必须结合采样周期/目标间距设置。至少一行，单行没有相邻跳变。
[[nodiscard]] JointContinuityReport
check_joint_continuity(const MatXd &q_rad, const VecXd &max_step_rad);
[[nodiscard]] JointContinuityReport check_joint_continuity(const MatXd &q_rad,
                                                           double max_step_rad);
} // namespace piper_control
