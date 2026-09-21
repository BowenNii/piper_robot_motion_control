#include "piper_control/lie_group/se3.hpp"
#include "piper_control/common/constants.hpp"
#include "piper_control/lie_group/so3.hpp"

namespace piper_control
{

/**
 * 【SE3-01】 twist_hat
 *
 * @brief 将 6×1 twist 转换为 4×4 se(3) 矩阵。
 * hat 算子：把 6 维旋量向量 ξ → 4×4 李代数矩阵
 * 数学定义：
 *
 *     ξ = [ω]
 *         [v]
 *
 *     ξ^ = [ [ω]^  v ]
 *          [  0      0 ]
 *
 * 其中：
 *     [ω]^ = skew(ω)
 *
 * 输入：
 *     xi = [ω; v] ∈ R⁶
 *
 * 输出：
 *     ξ^ ∈ se(3)
 */
Mat4 twist_hat(const Twist& xi)
{
    Mat4 result = Mat4::Zero();

    result.block<3, 3>(0, 0) = skew(xi.head<3>());
    result.block<3, 1>(0, 3) = xi.tail<3>();

    return result;
}


/**
 * 【SE3-02】 se3_from_rt
 *
 * @brief 根据旋转矩阵和平移向量构造 SE(3) 齐次变换矩阵。
 *
 * 数学定义：
 *
 *     T = [ R  p ]
 *         [ 0  1 ]
 *
 * 其中：
 *     R ∈ SO(3)
 *     p ∈ R³
 *
 * 输入：
 *     R：旋转矩阵
 *     p：平移向量
 *
 * 输出：
 *     T ∈ SE(3)
 */
Mat4 se3_from_rt(const Mat3& R, const Vec3& p)
{
    Mat4 T = Mat4::Identity();

    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = p;

    return T;
}


/**
 * 【SE3-03】 se3_inverse
 *
 * @brief 计算 SE(3) 齐次变换矩阵的解析逆。
 *
 * 对：
 *
 *     T = [ R  p ]
 *         [ 0  1 ]
 *
 * 有：
 *
 *     T⁻¹ = [ Rᵀ  -Rᵀp ]
 *            [  0     1  ]
 *
 * 利用旋转矩阵：
 *
 *     R⁻¹ = Rᵀ
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


/**
 * 【SE3-04】 adjoint
 *
 * @brief 计算 SE(3) 变换的伴随矩阵。
 *
 * 对：
 *
 *     T = [ R  p ]
 *         [ 0  1 ]
 *
 * 其伴随矩阵为：
 *
 *     Ad_T =
 *
 *     [ R       0 ]
 *     [ [p]^R    R ]
 *
 * 并满足：
 *
 *     V' = Ad_T V
 *
 * 其中 V 为 6×1 twist。
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


/**
 * 【SE3-05】 adjoint_inverse
 *
 * @brief 计算 SE(3) 伴随矩阵的逆。
 *
 * 数学定义：
 *
 *     Ad_T⁻¹ = Ad_{T⁻¹}
 *
 * 因此：
 *
 *     Ad_T⁻¹ = adjoint(T⁻¹)
 */
Mat6 adjoint_inverse(const Mat4& T)
{
    return adjoint(se3_inverse(T));
}


/**
 * 【SE3-06】 little_ad
 *
 * @brief 计算 twist 的 little adjoint 矩阵。
 *
 * 对：
 *
 *     V = [ω]
 *         [v]
 *
 * 有：
 *
 *     ad_V =
 *
 *     [ [ω]^   0   ]
 *     [ [v]^  [ω]^ ]
 *
 * 并满足 Lie bracket：
 *
 *     [V₁, V₂] = ad_{V₁} V₂
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


/**
 * 【SE3-07】 se3_exp
 *
 * @brief 计算 SE(3) 指数映射。
 *
 * 输入：
 *
 *     ξ = [φ]
 *         [ρ]
 *
 * 其中：
 *     φ ∈ R³：旋转向量
 *     ρ ∈ R³：平移部分
 *
 * 指数映射：
 *
 *     T = exp(ξ^)
 *
 *         = [ R  p ]
 *           [ 0  1 ]
 *
 * 其中：
 *
 *     R = exp([φ]^)
 *
 *     p = V(φ) ρ
 *
 *     V(φ) =
 *         I
 *       + A[φ]^
 *       + B[φ]²
 *
 *     A = (1 - cosθ) / θ²
 *
 *     B = (θ - sinθ) / θ³
 *
 *     θ = ||φ||
 *
 * 注意：
 *     当 θ → 0 时使用小角度近似。
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


/**
 * 【SE3-08】 se3_log
 *
 * @brief 计算 SE(3) 对数映射。
 *
 * 输入：
 *
 *     T = [ R  p ]
 *         [ 0  1 ]
 *
 * 对数映射：
 *
 *     ξ^ = log(T)
 *
 *     ξ = [φ]
 *         [ρ]
 *
 * 其中：
 *
 *     φ = log(R)
 *
 *     ρ = V⁻¹ p
 *
 * V⁻¹ 的计算：
 *
 *     V⁻¹ =
 *         I
 *       - 1/2 [φ]^
 *       + A [φ]²
 *
 * 其中：
 *
 *     A = 1/θ² - 1/(2θ tan(θ/2))
 *
 *     θ = ||φ||
 *
 * 当 θ → 0 时使用小角度近似。
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