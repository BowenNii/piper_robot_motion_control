#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

Mat4 twist_hat(const Twist& xi);

Mat4 se3_from_rt(const Mat3& R, const Vec3& p);

Mat4 se3_inverse(const Mat4& T);

Mat6 adjoint(const Mat4& T);

Mat6 adjoint_inverse(const Mat4& T);

Mat6 little_ad(const Twist& V);

/**
 * @brief SE(3) exponential map.
 *
 * xi = [phi; rho]
 * phi = omega * theta
 * rho = v * theta
 */
Mat4 se3_exp(const Twist& xi);

/**
 * @brief SE(3) logarithm map.
 *
 * 返回 xi = [phi; rho].
 */
Twist se3_log(const Mat4& T);

}  // namespace piper_control