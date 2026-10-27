#include "piper_control/common/error_statistics.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/robot_model/piper_model.hpp"
#include "piper_control/validation/kinematics_validation.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace piper_control;
void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
template <class F> void invalid(F f) {
  bool thrown = false;
  try {
    f();
  } catch (const std::invalid_argument &) {
    thrown = true;
  }
  require(thrown, "expected invalid_argument");
}
int main() try {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  Mat4 a = Mat4::Identity(), b = a;
  require(check_se3(a).ok, "identity SE3");
  require(!check_se3(MatXd::Identity(3, 3)).ok, "shape");
  b(0, 0) = -1;
  require(!check_se3(b).ok, "reflection");
  b = a;
  b(3, 0) = 0.01;
  require(!check_se3(b).ok, "last row");
  b = a;
  b(0, 0) = nan;
  require(!check_se3(b).finite, "NaN");
  invalid([&] { (void)check_se3(a, -1); });
  invalid([&] { (void)pose_error(a, b); });
  b = a;
  b(0, 3) = 0.003;
  b(1, 3) = 0.004;
  require(std::abs(pose_error(b, a).position_m - 0.005) < 1e-15, "distance");
  require((pose_error(b, a).delta_p_m - Vec3(0.003, 0.004, 0)).norm() < 1e-15,
          "signed delta");
  for (double angle :
       {0.0, 1e-10, 0.3, std::acos(-1.0) - 1e-10, std::acos(-1.0)}) {
    b = a;
    b.topLeftCorner<3, 3>() = rot_z(angle);
    require(std::abs(pose_error(a, b).rotation_rad - angle) < 1e-12,
            "rotation near zero/pi");
  }

  ErrorAccumulator stats;
  require(stats.statistics().count == 0 && std::isnan(stats.statistics().mean),
          "empty statistics");
  stats.add(3);
  stats.add(4);
  require(stats.statistics().mean == 3.5 && stats.statistics().max == 4,
          "mean/max");
  require(std::abs(stats.statistics().rms - std::sqrt(12.5)) < 1e-14, "rms");
  invalid([&] { stats.add(-1); });
  invalid([&] { stats.add(nan); });
  require(stats.statistics().count == 2, "failed add unchanged");
  stats.reset();
  stats.add(0);
  require(stats.statistics().rms == 0, "zero");
  stats.reset();
  stats.add(1e308);
  stats.add(1e308);
  require(std::isfinite(stats.statistics().rms) &&
              stats.statistics().rms == 1e308,
          "scaled RMS");

  MatXd q(3, 2);
  q << 0, 0, 1, -1, 1.1, -1.2;
  VecXd lower = VecXd::Constant(2, -1), upper = VecXd::Constant(2, 1);
  auto limits = check_joint_limits(q, lower, upper);
  require(!limits.ok && limits.valid_rows[0] && limits.valid_rows[1] &&
              !limits.valid_rows[2] && limits.violations.size() == 2,
          "limit report");
  require(check_joint_limits(q, lower, upper, 0.21).ok, "limit tolerance");
  invalid([&] { (void)check_joint_limits(q, upper, lower); });
  invalid([&] { (void)check_joint_limits(q, lower, VecXd::Zero(3)); });
  invalid([&] { (void)check_joint_limits(q, lower, upper, nan); });
  MatXd bad = q;
  bad(0, 0) = nan;
  invalid([&] { (void)check_joint_limits(bad, lower, upper); });
  auto continuity = check_joint_continuity(q, 0.5);
  require(!continuity.ok && continuity.jumps.size() == 2 &&
              continuity.jumps[0].row == 1 && continuity.jumps[1].joint == 1,
          "jump indices");
  require(check_joint_continuity(q, 1.0).ok, "threshold equality");
  require(check_joint_continuity(MatXd(q.topRows(1)), 0.0).delta_rad.rows() ==
              0,
          "single sample");
  invalid([&] { (void)check_joint_continuity(MatXd(0, 2), 0.1); });
  invalid([&] { (void)check_joint_continuity(q, -1); });
  invalid([&] { (void)check_joint_continuity(q, VecXd::Zero(3)); });
  bad.resize(2, 1);
  bad << 3.14, -3.14;
  require(!check_joint_continuity(bad, 1.0).ok, "no automatic angle wrap");

  const auto model = make_piper_model();
  VecXd zero = VecXd::Zero(6);
  auto ik = check_ik_solution(model.S_list, model.M, zero, model.M, model.q_min,
                              model.q_max);
  require(ik.ok, "valid IK candidate");
  Mat4 shifted = model.M;
  shifted(0, 3) += 0.01;
  require(!check_ik_solution(model.S_list, model.M, zero, shifted, model.q_min,
                             model.q_max)
               .ok,
          "wrong target");
  zero(0) = model.q_max(0) + 0.1;
  const auto target = forward_poe_space(model.S_list, zero, model.M);
  require(!check_ik_solution(model.S_list, model.M, zero, target, model.q_min,
                             model.q_max)
               .ok,
          "correct FK but out of limits");
  invalid([&] {
    (void)check_ik_solution(MatXd::Zero(5, 6), model.M, zero, target,
                            model.q_min, model.q_max);
  });
  IkValidationConfig cfg;
  cfg.position_tolerance_m = -1;
  invalid([&] {
    (void)check_ik_solution(model.S_list, model.M, zero, target, model.q_min,
                            model.q_max, cfg);
  });
  std::cout << "Validation modules PASS\n";
  return 0;
} catch (const std::exception &e) {
  std::cerr << e.what() << '\n';
  return 1;
}
