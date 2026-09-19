#include "piper_control/lie_group/so3.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "piper_control/common/constants.hpp"
#include "piper_control/math/math.hpp"

namespace piper_control
{

Mat3 skew(const Vec3& v)
{
    Mat3 S;

    S << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
         -v.y(), v.x(), 0.0;

    return S;
}

Vec3 vee(const Mat3& skew_matrix)
{
    Vec3 v;

    v << skew_matrix(2, 1),
         skew_matrix(0, 2),
         skew_matrix(1, 0);

    return v;
}

Mat3 rot_x(double theta)
{
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    Mat3 R;

    R << 1.0, 0.0, 0.0,
         0.0, c, -s,
         0.0, s, c;

    return R;
}

Mat3 rot_y(double theta)
{
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    Mat3 R;

    R << c, 0.0, s,
         0.0, 1.0, 0.0,
         -s, 0.0, c;

    return R;
}

Mat3 rot_z(double theta)
{
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    Mat3 R;

    R << c, -s, 0.0,
         s, c, 0.0,
         0.0, 0.0, 1.0;

    return R;
}

Mat3 so3_exp(const Vec3& rotation_vector)
{
    const double theta = rotation_vector.norm();

    if (theta < kSmallAngle)
    {
        const Mat3 Phi = skew(rotation_vector);

        return Mat3::Identity()
             + Phi
             + 0.5 * Phi * Phi;
    }

    const Vec3 omega = rotation_vector / theta;
    const Mat3 Omega = skew(omega);

    return Mat3::Identity()
         + std::sin(theta) * Omega
         + (1.0 - std::cos(theta)) * Omega * Omega;
}

Mat3 rot_axis_angle(const Vec3& axis, double theta)
{
    const double norm = axis.norm();

    assert(norm > kEpsilon);

    const Vec3 omega = axis / norm;
    const Mat3 Omega = skew(omega);

    return Mat3::Identity()
         + std::sin(theta) * Omega
         + (1.0 - std::cos(theta)) * Omega * Omega;
}

Vec3 so3_log(const Mat3& R)
{
    const double trace_value = R.trace();

    double cos_theta = 0.5 * (trace_value - 1.0);

    cos_theta = clamp(cos_theta, -1.0, 1.0);

    const double theta = std::acos(cos_theta);

    // θ ≈ 0
    if (theta < kSmallAngle)
    {
        return vee(0.5 * (R - R.transpose()));
    }

    // θ ≈ π
    if ((kPi - theta) < kSmallAngle)
    {
        Vec3 axis;

        const double d0 = std::max(0.0, 0.5 * (R(0, 0) + 1.0));
        const double d1 = std::max(0.0, 0.5 * (R(1, 1) + 1.0));
        const double d2 = std::max(0.0, 0.5 * (R(2, 2) + 1.0));

        if (d0 >= d1 && d0 >= d2)
        {
            axis.x() = std::sqrt(d0);

            if (axis.x() > kEpsilon)
            {
                axis.y() = (R(0, 1) + R(1, 0))
                         / (4.0 * axis.x());

                axis.z() = (R(0, 2) + R(2, 0))
                         / (4.0 * axis.x());
            }
            else
            {
                axis = Vec3::UnitX();
            }
        }
        else if (d1 >= d0 && d1 >= d2)
        {
            axis.y() = std::sqrt(d1);

            if (axis.y() > kEpsilon)
            {
                axis.x() = (R(0, 1) + R(1, 0))
                         / (4.0 * axis.y());

                axis.z() = (R(1, 2) + R(2, 1))
                         / (4.0 * axis.y());
            }
            else
            {
                axis = Vec3::UnitY();
            }
        }
        else
        {
            axis.z() = std::sqrt(d2);

            if (axis.z() > kEpsilon)
            {
                axis.x() = (R(0, 2) + R(2, 0))
                         / (4.0 * axis.z());

                axis.y() = (R(1, 2) + R(2, 1))
                         / (4.0 * axis.z());
            }
            else
            {
                axis = Vec3::UnitZ();
            }
        }

        axis.normalize();

        return theta * axis;
    }

    // 普通情况：
    // phi = theta / (2 sin(theta)) * vee(R - R^T)
    const Vec3 axis =
        vee(R - R.transpose()) / (2.0 * std::sin(theta));

    return theta * axis.normalized();
}

}  // namespace piper_control