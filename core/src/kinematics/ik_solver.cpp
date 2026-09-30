#include "piper_control/kinematics/ik_solver.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/kinematics/jacobian.hpp"
#include "piper_control/lie_group/se3.hpp"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace piper_control {

namespace {
enum class Method { Newton, Dls, Adaptive, Transpose };
bool positive(double x) { return std::isfinite(x) && x > 0; }
bool valid_pose(const Mat4 &T) {
  if (!T.allFinite())
    return false;
  const Mat3 R = T.topLeftCorner<3, 3>();
  return (R.transpose() * R - Mat3::Identity()).norm() < 1e-8 &&
         std::abs(R.determinant() - 1) < 1e-8 &&
         (T.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-8;
}
struct Error {
  Vec6 body;
  double position;
  double rotation;
};
Error error(const Mat4 &T, const Mat4 &target) {
  const Mat3 R =
      T.topLeftCorner<3, 3>().transpose() * target.topLeftCorner<3, 3>();
  Vec3 skew_part;
  skew_part << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
  return {se3_log(se3_inverse(T) * target),
          (T.topRightCorner<3, 1>() - target.topRightCorner<3, 1>()).norm(),
          std::atan2(0.5 * skew_part.norm(),
                     std::clamp((R.trace() - 1) / 2, -1.0, 1.0))};
}
IKResult solve(Method method, const MatXd &S, const Mat4 &M, const Mat4 &target,
               const VecXd &initial, const IKOptions &o) {
  const auto n = initial.size();
  const bool bounded = o.q_min.size() != 0 || o.q_max.size() != 0;
  if (n == 0 || S.rows() != 6 || S.cols() != n || !S.allFinite() ||
      !initial.allFinite() || !valid_pose(M) || !valid_pose(target) ||
      o.max_iterations < 0 || o.max_backtracks < 0 ||
      !positive(o.position_tolerance) || !positive(o.orientation_tolerance) ||
      !positive(o.max_joint_step) || !positive(o.rotation_weight) ||
      !positive(o.svd_tolerance) || !positive(o.sigma_threshold) ||
      !positive(o.lambda_max) || !std::isfinite(o.lambda) || o.lambda < 0)
    throw std::invalid_argument("IK: invalid input or options");
  if (bounded && (o.q_min.size() != n || o.q_max.size() != n ||
                  !o.q_min.allFinite() || !o.q_max.allFinite()))
    throw std::invalid_argument("IK: invalid bounds dimensions");
  if (bounded && ((o.q_min.array() > o.q_max.array()).any() ||
                  (initial.array() < o.q_min.array()).any() ||
                  (initial.array() > o.q_max.array()).any()))
    throw std::invalid_argument("IK: initial state outside bounds");
  IKResult r;
  r.q = initial;
  const auto cost = [&](const Error &e) {
    Vec6 b = e.body;
    b.head<3>() *= o.rotation_weight;
    return b.squaredNorm();
  };
  for (int k = 0; k <= o.max_iterations; ++k) {
    r.iterations = k;
    const Mat4 T = forward_poe_space(S, r.q, M);
    const Error e = error(T, target);
    r.position_error = e.position;
    r.orientation_error = e.rotation;
    if (!e.body.allFinite() || !std::isfinite(e.position) ||
        !std::isfinite(e.rotation)) {
      r.status = IKStatus::kNumericalFailure;
      return r;
    }
    if (e.position <= o.position_tolerance &&
        e.rotation <= o.orientation_tolerance) {
      r.success = true;
      r.status = IKStatus::kConverged;
      return r;
    }
    if (k == o.max_iterations)
      break;
    MatXd A = adjoint_inverse(T) * jacobian_space(S, r.q);
    A.topRows(3) *= o.rotation_weight;
    Vec6 b = e.body;
    b.head<3>() *= o.rotation_weight;
    VecXd step;
    if (method == Method::Transpose) {
      const VecXd g = A.transpose() * b;
      const double denominator = (A * g).squaredNorm();
      step = VecXd::Zero(n);
      if (denominator > 0)
        step = (g.squaredNorm() / denominator) * g;
    } else {
      const Eigen::JacobiSVD<MatXd> svd(A, Eigen::ComputeThinU |
                                               Eigen::ComputeThinV);
      const VecXd sigma = svd.singularValues();
      double lambda = method == Method::Dls ? o.lambda : 0;
      if (method == Method::Adaptive) {
        // 少于6列时，全六维任务存在缺失方向。
        const double smin = n < 6 ? 0 : sigma.tail(1)(0);
        const double ratio = std::min(1.0, smin / o.sigma_threshold);
        lambda = o.lambda_max * std::sqrt(std::max(0.0, 1 - ratio * ratio));
      }
      VecXd gain = sigma;
      for (Eigen::Index i = 0; i < sigma.size(); ++i)
        gain(i) = lambda > 0
                      ? sigma(i) / (sigma(i) * sigma(i) + lambda * lambda)
                      : (sigma(i) > o.svd_tolerance ? 1 / sigma(i) : 0);
      step = svd.matrixV() * gain.asDiagonal() * svd.matrixU().transpose() * b;
    }
    if (!step.allFinite()) {
      r.status = IKStatus::kNumericalFailure;
      return r;
    }
    const double largest = step.cwiseAbs().maxCoeff();
    if (largest > o.max_joint_step)
      step *= o.max_joint_step / largest;
    bool accepted = false;
    double scale = 1;
    for (int j = 0; j <= o.max_backtracks; ++j, scale *= 0.5) {
      VecXd candidate = r.q + scale * step;
      if (bounded)
        candidate = candidate.cwiseMax(o.q_min).cwiseMin(o.q_max);
      const Error trial = error(forward_poe_space(S, candidate, M), target);
      if (trial.body.allFinite() && cost(trial) < cost(e)) {
        r.q = candidate;
        accepted = true;
        break;
      }
    }
    if (!accepted) {
      r.status = IKStatus::kStalled;
      return r;
    }
  }
  r.status = IKStatus::kMaxIterations;
  return r;
}
} // namespace

IKResult solve_ik_newton(const MatXd &S, const Mat4 &M, const Mat4 &T,
                         const VecXd &q, const IKOptions &o) {
  return solve(Method::Newton, S, M, T, q, o);
}
IKResult solve_ik_dls(const MatXd &S, const Mat4 &M, const Mat4 &T,
                      const VecXd &q, const IKOptions &o) {
  return solve(Method::Dls, S, M, T, q, o);
}
IKResult solve_ik_adaptive_dls(const MatXd &S, const Mat4 &M, const Mat4 &T,
                               const VecXd &q, const IKOptions &o) {
  return solve(Method::Adaptive, S, M, T, q, o);
}
IKResult solve_ik_transpose(const MatXd &S, const Mat4 &M, const Mat4 &T,
                            const VecXd &q, const IKOptions &o) {
  return solve(Method::Transpose, S, M, T, q, o);
}

} // namespace piper_control
