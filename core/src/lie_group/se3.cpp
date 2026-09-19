#include "piper_control/lie_group/se3.hpp"

#include <cmath>

#include "piper_control/common/constants.hpp"
#include "piper_control/lie_group/so3.hpp"

namespace piper_control
{

/*【SE3-01】
 * @brief 将 6×1 twist 转换为 4×4 se(3) 矩阵。
 */
Mat4 twist_hat(const Twist& xi)
{
    Mat4 result = Mat4::Zero();

    result.block<3, 3>(0, 0) = skew(xi.head<3>());
    result.block<3, 1>(0, 3) = xi.tail<3>();

    return result;
}

/*【SE3-02】
 * @brief 根据旋转矩阵和平移向量构造 SE(3) 齐次变换矩阵。
 */
Mat4 se3_from_rt(const Mat3& R, const Vec3& p)
{
    Mat4 T = Mat4::Identity();

    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = p;

    return T;
}

/*【SE3-03】
 * @brief 计算 SE(3) 齐次变换矩阵的解析逆。
 */
Mat4 se3_inverse(const Mat4& T)
{
    Mat4 T_inv = Mat4::Identity();

    const Mat3 R = T.block<3, 3>(0, 0);
    const Vec3 p = T.block<3, 1>(0, 3);

    const Mat3 R_t = R.transpose();

    T_inv.block<3, 3>(0, 0) = R_t;
    T_inv.block<3, 1>(0, 3) = -R_t * p;

    return T_inv;
}

/*【SE3-04】
 * @brief 计算 SE(3) 变换的伴随矩阵。
 */
Mat6 adjoint(const Mat4& T)
{
    Mat6 Ad = Mat6::Zero();

    const Mat3 R = T.block<3, 3>(0, 0);
    const Vec3 p = T.block<3, 1>(0, 3);

    Ad.block<3, 3>(0, 0) = R;
    Ad.block<3, 3>(3, 0) = skew(p) * R;
    Ad.block<3, 3>(3, 3) = R;

    return Ad;
}

/*【SE3-05】
 * @brief 计算 SE(3) 伴随矩阵的逆。
 */
Mat6 adjoint_inverse(const Mat4& T)
{
    return adjoint(se3_inverse(T));
}

/*【SE3-06】
 * @brief 计算 twist 的 little adjoint 矩阵。
 */
Mat6 little_ad(const Twist& V)
{
    Mat6 ad = Mat6::Zero();

    const Mat3 omega_hat = skew(V.head<3>());
    const Mat3 v_hat = skew(V.tail<3>());

    ad.block<3, 3>(0, 0) = omega_hat;
    ad.block<3, 3>(3, 0) = v_hat;
    ad.block<3, 3>(3, 3) = omega_hat;

    return ad;
}

/*【SE3-07】
 * @brief 计算 SE(3) 指数映射。
 *
 * xi = [phi; rho]
 */
Mat4 se3_exp(const Twist& xi)
{
    const Vec3 phi = xi.head<3>();
    const Vec3 rho = xi.tail<3>();

    const double theta = phi.norm();

    if (theta < kSmallAngle)
    {
        return se3_from_rt(
            Mat3::Identity(),
            rho);
    }

    const Mat3 Phi = skew(phi);
    const double theta_sq = theta * theta;
    const double theta_cube = theta_sq * theta;

    const double A =
        (1.0 - std::cos(theta)) / theta_sq;

    const double B =
        (theta - std::sin(theta)) / theta_cube;

    const Mat3 V =
        Mat3::Identity()
        + A * Phi
        + B * Phi * Phi;

    const Mat3 R = so3_exp(phi);
    const Vec3 p = V * rho;

    return se3_from_rt(R, p);
}

/*【SE3-08】
 * @brief 计算 SE(3) 对数映射。
 *
 * T → xi = [phi; rho]
 */
Twist se3_log(const Mat4& T)
{
    const Mat3 R = T.block<3, 3>(0, 0);
    const Vec3 p = T.block<3, 1>(0, 3);

    const Vec3 phi = so3_log(R);
    const double theta = phi.norm();

    Twist xi;

    if (theta < kSmallAngle)
    {
        xi.head<3>() = Vec3::Zero();
        xi.tail<3>() = p;

        return xi;
    }

    const Mat3 Phi = skew(phi);

    const double half_theta = 0.5 * theta;
    const double tan_half = std::tan(half_theta);

    double A;

    if (std::abs(tan_half) < kEpsilon)
    {
        A = 0.0;
    }
    else
    {
        A =
            (1.0 / (theta * theta))
            - (1.0 / (2.0 * theta * tan_half));
    }

    const Mat3 V_inv =
        Mat3::Identity()
        - 0.5 * Phi
        + A * Phi * Phi;

    xi.head<3>() = phi;
    xi.tail<3>() = V_inv * p;

    return xi;
}

}  // namespace piper_control