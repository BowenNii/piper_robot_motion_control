#include "piper_control/lie_group/se3.hpp"

#include <cassert>
#include <cmath>

#include "piper_control/common/constants.hpp"
#include "piper_control/lie_group/so3.hpp"

namespace piper_control
{

Mat4 twist_hat(const Twist& xi)
{
    Mat4 Xi = Mat4::Zero();

    Xi.block<3, 3>(0, 0) = skew(xi.head<3>());
    Xi.block<3, 1>(0, 3) = xi.tail<3>();

    return Xi;
}

Mat4 se3_from_rt(const Mat3& R, const Vec3& p)
{
    Mat4 T = Mat4::Identity();

    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = p;

    return T;
}

Mat4 se3_inverse(const Mat4& T)
{
    const Mat3 R = T.block<3, 3>(0, 0);
    const Vec3 p = T.block<3, 1>(0, 3);

    Mat4 T_inv = Mat4::Identity();

    const Mat3 R_transpose = R.transpose();

    T_inv.block<3, 3>(0, 0) = R_transpose;
    T_inv.block<3, 1>(0, 3) = -R_transpose * p;

    return T_inv;
}

Mat6 adjoint(const Mat4& T)
{
    const Mat3 R = T.block<3, 3>(0, 0);
    const Vec3 p = T.block<3, 1>(0, 3);

    Mat6 Ad = Mat6::Zero();

    Ad.block<3, 3>(0, 0) = R;
    Ad.block<3, 3>(3, 0) = skew(p) * R;
    Ad.block<3, 3>(3, 3) = R;

    return Ad;
}

Mat6 adjoint_inverse(const Mat4& T)
{
    return adjoint(se3_inverse(T));
}

Mat6 little_ad(const Twist& V)
{
    const Vec3 omega = V.head<3>();
    const Vec3 v = V.tail<3>();

    Mat6 ad = Mat6::Zero();

    ad.block<3, 3>(0, 0) = skew(omega);
    ad.block<3, 3>(3, 0) = skew(v);
    ad.block<3, 3>(3, 3) = skew(omega);

    return ad;
}

Mat4 se3_exp(const Twist& xi)
{
    const Vec3 phi = xi.head<3>();
    const Vec3 rho = xi.tail<3>();

    const double theta = phi.norm();

    Mat3 R;
    Mat3 V;

    if (theta < kSmallAngle)
    {
        const Mat3 Phi = skew(phi);

        R = Mat3::Identity()
          + Phi
          + 0.5 * Phi * Phi;

        V = Mat3::Identity()
          + 0.5 * Phi
          + (1.0 / 6.0) * Phi * Phi;
    }
    else
    {
        const Mat3 Phi = skew(phi);
        const double theta2 = theta * theta;
        const double theta3 = theta2 * theta;

        R = Mat3::Identity()
          + (std::sin(theta) / theta) * Phi
          + ((1.0 - std::cos(theta)) / theta2) * Phi * Phi;

        V = Mat3::Identity()
          + ((1.0 - std::cos(theta)) / theta2) * Phi
          + ((theta - std::sin(theta)) / theta3) * Phi * Phi;
    }

    const Vec3 p = V * rho;

    return se3_from_rt(R, p);
}

Twist se3_log(const Mat4& T)
{
    const Mat3 R = T.block<3, 3>(0, 0);
    const Vec3 p = T.block<3, 1>(0, 3);

    const Vec3 phi = so3_log(R);

    const double theta = phi.norm();

    Twist xi = Twist::Zero();

    if (theta < kSmallAngle)
    {
        xi.head<3>().setZero();
        xi.tail<3>() = p;

        return xi;
    }

    const Mat3 Phi = skew(phi);

    double A;

    if (theta < 1e-4)
    {
        const double theta2 = theta * theta;
        const double theta4 = theta2 * theta2;

        A = 1.0 / 12.0
          + theta2 / 720.0
          + theta4 / 30240.0;
    }
    else
    {
        const double half_theta = 0.5 * theta;

        const double cot_half_theta =
            std::cos(half_theta) / std::sin(half_theta);

        A = (1.0 - half_theta * cot_half_theta)
          / (theta * theta);
    }

    const Mat3 V_inv =
          Mat3::Identity()
        - 0.5 * Phi
        + A * Phi * Phi;

    const Vec3 rho = V_inv * p;

    xi.head<3>() = phi;
    xi.tail<3>() = rho;

    return xi;
}

}  // namespace piper_control