#include "piper_control/lie_group/se3.hpp"

#include <cmath>

#include "piper_control/common/constants.hpp"
#include "piper_control/lie_group/so3.hpp"

namespace piper_control
{

Mat4 twist_hat(const Twist& xi)
{
    const Vec3 omega = xi.head<3>();
    const Vec3 v = xi.tail<3>();

    Mat4 Xi = Mat4::Zero();

    Xi.block<3, 3>(0, 0) = skew(omega);
    Xi.block<3, 1>(0, 3) = v;

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

    const Mat3 R_inv = R.transpose();
    const Vec3 p_inv = -R_inv * p;

    return se3_from_rt(R_inv, p_inv);
}

Mat6 adjoint(const Mat4& T)
{
    const Mat3 R = T.block<3, 3>(0, 0);
    const Vec3 p = T.block<3, 1>(0, 3);

    Mat6 Ad = Mat6::Zero();

    Ad.block<3, 3>(0, 0) = R;
    Ad.block<3, 3>(0, 3).setZero();
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
    ad.block<3, 3>(0, 3).setZero();
    ad.block<3, 3>(3, 0) = skew(v);
    ad.block<3, 3>(3, 3) = skew(omega);

    return ad;
}

Mat4 se3_exp(const Twist& xi)
{
    const Vec3 phi = xi.head<3>();
    const Vec3 rho = xi.tail<3>();

    const double theta = phi.norm();

    Mat3 V = Mat3::Identity();

    if (theta < kSmallAngle)
    {
        const Mat3 Phi = skew(phi);
        V += 0.5 * Phi + (1.0 / 6.0) * Phi * Phi;
    }
    else
    {
        const Mat3 Phi = skew(phi);
        const double theta2 = theta * theta;
        const double theta3 = theta2 * theta;

        const double A = (1.0 - std::cos(theta)) / theta2;
        const double B = (theta - std::sin(theta)) / theta3;

        V += A * Phi + B * Phi * Phi;
    }

    const Mat3 R = so3_exp(phi);
    const Vec3 p = V * rho;

    return se3_from_rt(R, p);
}

Twist se3_log(const Mat4& T)
{
    const Mat3 R = T.block<3, 3>(0, 0);
    const Vec3 p = T.block<3, 1>(0, 3);

    const Vec3 phi = so3_log(R);
    const double theta = phi.norm();

    Mat3 V_inv = Mat3::Identity();

    const Mat3 Phi = skew(phi);

    if (theta < kSmallAngle)
    {
        V_inv += -0.5 * Phi + (1.0 / 12.0) * Phi * Phi;
    }
    else
    {
        const double half_theta = 0.5 * theta;

        const double sin_half = std::sin(half_theta);
        const double cos_half = std::cos(half_theta);

        const double cot_half = cos_half / sin_half;

        const double A =
            (1.0 - 0.5 * theta * cot_half) / (theta * theta);

        V_inv += -0.5 * Phi + A * Phi * Phi;
    }

    const Vec3 rho = V_inv * p;

    Twist xi;
    xi.head<3>() = phi;
    xi.tail<3>() = rho;

    return xi;
}

}  // namespace piper_control