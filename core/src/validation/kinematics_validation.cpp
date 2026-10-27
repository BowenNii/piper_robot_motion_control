#include "piper_control/validation/kinematics_validation.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include <cmath>
#include <stdexcept>

namespace piper_control {
IkValidationReport check_ik_solution(const MatXd &S, const Mat4 &M,
                                     const VecXd &q, const Mat4 &target,
                                     const VecXd &lower, const VecXd &upper,
                                     const IkValidationConfig &cfg) {
  if (q.size() == 0 || !q.allFinite() || S.rows() != 6 ||
      S.cols() != q.size() || !S.allFinite() || !check_se3(M).ok ||
      !check_se3(target).ok || !std::isfinite(cfg.position_tolerance_m) ||
      cfg.position_tolerance_m < 0 ||
      !std::isfinite(cfg.rotation_tolerance_rad) ||
      cfg.rotation_tolerance_rad < 0)
    throw std::invalid_argument("Invalid IK validation input/config");
  IkValidationReport r;
  // 先检查解的物理范围，再把候选解送回 FK，核对是否达到目标。
  const MatXd samples = q.transpose();
  r.limits = check_joint_limits(samples, lower, upper, cfg.limit_tolerance_rad);
  r.T_fk = forward_poe_space(S, q, M);
  r.error = pose_error(r.T_fk, target);
  r.ok = r.limits.ok && r.error.position_m <= cfg.position_tolerance_m &&
         r.error.rotation_rad <= cfg.rotation_tolerance_rad;
  return r;
}
JointContinuityReport check_joint_continuity(const MatXd &q,
                                             const VecXd &threshold) {
  if (q.rows() < 1 || q.cols() < 1 || !q.allFinite() ||
      threshold.size() != q.cols() || !threshold.allFinite() ||
      (threshold.array() < 0).any())
    throw std::invalid_argument("Invalid continuity input/threshold");
  JointContinuityReport r;
  // 同时对每一列关节做相邻行差分；第 k 个差值对应原序列第 k+1 行。
  r.delta_rad = q.bottomRows(q.rows() - 1) - q.topRows(q.rows() - 1);
  if (!r.delta_rad.allFinite())
    throw std::invalid_argument("Joint difference overflow");
  for (Eigen::Index row = 0; row < r.delta_rad.rows(); ++row)
    for (Eigen::Index joint = 0; joint < q.cols(); ++joint)
      if (std::abs(r.delta_rad(row, joint)) > threshold(joint))
        r.jumps.push_back({static_cast<std::size_t>(row + 1),
                           static_cast<std::size_t>(joint),
                           r.delta_rad(row, joint)});
  r.ok = r.jumps.empty();
  return r;
}
JointContinuityReport check_joint_continuity(const MatXd &q, double threshold) {
  return check_joint_continuity(q, VecXd::Constant(q.cols(), threshold));
}
} // namespace piper_control
