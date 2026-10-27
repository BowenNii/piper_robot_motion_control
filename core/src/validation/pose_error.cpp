#include "piper_control/validation/pose_error.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace piper_control {
Se3ValidationReport check_se3(const MatXd &T, double tolerance) {
  if (!std::isfinite(tolerance) || tolerance < 0.0)
    throw std::invalid_argument("SE3 tolerance must be finite and nonnegative");
  Se3ValidationReport r;
  r.orthogonality_error = r.last_row_error =
      std::numeric_limits<double>::infinity();
  r.determinant = std::numeric_limits<double>::quiet_NaN();
  r.shape_valid = T.rows() == 4 && T.cols() == 4;
  r.finite = T.allFinite();
  if (!r.shape_valid || !r.finite)
    return r;
  const Mat3 R = T.topLeftCorner<3, 3>();
  r.orthogonality_error = (R.transpose() * R - Mat3::Identity()).norm();
  r.determinant = R.determinant();
  r.last_row_error = (T.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm();
  r.ok = r.orthogonality_error <= tolerance &&
         std::abs(r.determinant - 1.0) <= tolerance &&
         r.last_row_error <= tolerance;
  return r;
}

PoseError pose_error(const Mat4 &actual, const Mat4 &reference) {
  if (!check_se3(actual).ok || !check_se3(reference).ok)
    throw std::invalid_argument("pose_error requires valid SE3 poses");
  PoseError result;
  result.delta_p_m =
      actual.topRightCorner<3, 1>() - reference.topRightCorner<3, 1>();
  result.position_m = result.delta_p_m.stableNorm();
  if (!result.delta_p_m.allFinite() || !std::isfinite(result.position_m))
    throw std::invalid_argument("Position difference overflow");
  const Mat3 R = actual.topLeftCorner<3, 3>().transpose() *
                 reference.topLeftCorner<3, 3>();
  const Vec3 sine((R(2, 1) - R(1, 2)) / 2, (R(0, 2) - R(2, 0)) / 2,
                  (R(1, 0) - R(0, 1)) / 2);
  // atan2比acos(trace)更能保留接近零转角的精度，也适用于接近pi。
  result.rotation_rad =
      std::atan2(sine.norm(), std::clamp((R.trace() - 1) / 2, -1.0, 1.0));
  return result;
}
} // namespace piper_control
