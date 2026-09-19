#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

Mat3 skew(const Vec3& v);

Vec3 vee(const Mat3& skew_matrix);

Mat3 rot_x(double theta);

Mat3 rot_y(double theta);

Mat3 rot_z(double theta);

/**
 * @brief SO(3) exponential map.
 *
 * 输入 rotation_vector = omega * theta
 * 输出 R = exp([omega] * theta)
 */
Mat3 so3_exp(const Vec3& rotation_vector);

/**
 * @brief SO(3) logarithm map.
 *
 * 输出 rotation_vector = omega * theta
 * 其中 theta = ||rotation_vector||, theta ∈ [0, pi].
 */
Vec3 so3_log(const Mat3& R);

/**
 * @brief 根据单位旋转轴和旋转角构造 SO(3).
 */
Mat3 rot_axis_angle(const Vec3& axis, double theta);

}  // namespace piper_control