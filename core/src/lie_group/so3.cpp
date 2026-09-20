#include "piper_control/lie_group/so3.hpp"

#include <cassert>
#include <cmath>

#include "piper_control/common/constants.hpp"
#include "piper_control/math/math.hpp"

namespace piper_control
{

/**
 * 【SO3-01】 skew
 *
 * @brief 根据 3×1 向量构造 3×3 反对称矩阵。
 *
 * 数学定义：
 *
 *     v = [v₁]
 *         [v₂]
 *         [v₃]
 *
 *     [v]^ = [  0  -v₃   v₂ ]
 *             [ v₃   0   -v₁ ]
 *             [-v₂  v₁    0  ]
 *
 * 并满足：
 *
 *     [v]^ x = v × x
 *
 * 输入：
 *     v ∈ R³
 *
 * 输出：
 *     [v]^ ∈ so(3)
 */
Mat3 skew(const Vec3& v)
{
    Mat3 S;

    S << 0.0, -v(2), v(1),
         v(2), 0.0, -v(0),
        -v(1), v(0), 0.0;

    return S;
}


/**
 * 【SO3-02】 vee
 *
 * @brief 从 3×3 反对称矩阵提取 3×1 向量。
 *
 * 数学定义：
 *
 *     [v]^ =
 *     [  0  -v₃   v₂ ]
 *     [ v₃   0   -v₁ ]
 *     [-v₂  v₁    0  ]
 *
 * 则：
 *
 *     vee([v]^) = v
 *
 * 输入：
 *     skew_matrix ∈ so(3)
 *
 * 输出：
 *     v ∈ R³
 *
 * 注意：
 *     该函数默认输入矩阵为反对称矩阵。
 */
Vec3 vee(const Mat3& skew_matrix)
{
    Vec3 v;

    v << skew_matrix(2, 1),
         skew_matrix(0, 2),
         skew_matrix(1, 0);

    return v;
}


/**
 * 【SO3-03】 rot_x
 *
 * @brief 计算绕 X 轴旋转 theta 的旋转矩阵。
 *
 * 数学定义：
 *
 *     Rₓ(θ) =
 *
 *     [ 1    0       0   ]
 *     [ 0   cosθ   -sinθ ]
 *     [ 0   sinθ    cosθ ]
 *
 * 输入：
 *     theta：旋转角，单位 rad
 *
 * 输出：
 *     Rₓ(θ) ∈ SO(3)
 */
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


/**
 * 【SO3-04】 rot_y
 *
 * @brief 计算绕 Y 轴旋转 theta 的旋转矩阵。
 *
 * 数学定义：
 *
 *     Rᵧ(θ) =
 *
 *     [ cosθ   0   sinθ ]
 *     [  0     1    0   ]
 *     [-sinθ   0   cosθ]
 *
 * 输入：
 *     theta：旋转角，单位 rad
 *
 * 输出：
 *     Rᵧ(θ) ∈ SO(3)
 */
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


/**
 * 【SO3-05】 rot_z
 *
 * @brief 计算绕 Z 轴旋转 theta 的旋转矩阵。
 *
 * 数学定义：
 *
 *     R_z(θ) =
 *
 *     [ cosθ  -sinθ   0 ]
 *     [ sinθ   cosθ   0 ]
 *     [  0      0     1 ]
 *
 * 输入：
 *     theta：旋转角，单位 rad
 *
 * 输出：
 *     R_z(θ) ∈ SO(3)
 */
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


/**
 * 【SO3-06】 so3_exp
 *
 * @brief 计算 SO(3) 指数映射。
 *
 * 输入：
 *
 *     φ ∈ R³
 *
 * 其中：
 *
 *     θ = ||φ||
 *
 *     a = φ / θ
 *
 * 指数映射：
 *
 *     R = exp([φ]^)
 *
 * 根据 Rodrigues 公式：
 *
 *     R = I
 *       + sin(θ)[a]^
 *       + (1-cos(θ))[a]²
 *
 * 其中：
 *
 *     [a]^ = skew(a)
 *
 * 当 θ → 0 时：
 *
 *     exp([φ]^) → I
 *
 * 因此在小角度情况下返回单位矩阵。
 *
 * 输入：
 *     rotation_vector：旋转向量 φ，单位 rad
 *
 * 输出：
 *     R ∈ SO(3)
 */
Mat3 so3_exp(const Vec3& rotation_vector)
{
    const double theta = rotation_vector.norm();

    if (theta < kSmallAngle)
    {
        return Mat3::Identity();
    }

    const Vec3 axis = rotation_vector / theta;
    const Mat3 K = skew(axis);

    return Mat3::Identity()
         + std::sin(theta) * K
         + (1.0 - std::cos(theta)) * K * K;
}


/**
 * 【SO3-07】 so3_log
 *
 * @brief 计算 SO(3) 对数映射。
 *
 * 输入：
 *
 *     R ∈ SO(3)
 *
 * 对数映射：
 *
 *     [φ]^ = log(R)
 *
 * 其中：
 *
 *     θ = ||φ||
 *
 * 旋转角：
 *
 *     θ = acos((tr(R)-1)/2)
 *
 * 当 θ ∉ {0, π} 时：
 *
 *     [φ]^ =
 *
 *     θ / (2 sinθ) (R - Rᵀ)
 *
 * 因此：
 *
 *     φ =
 *     θ vee((R - Rᵀ)/(2 sinθ))
 *
 * 特殊情况：
 *
 *     θ → 0：
 *         φ → 0
 *
 *     θ → π：
 *         使用旋转矩阵对角元素提取旋转轴，
 *         避免除以 sinθ ≈ 0。
 *
 * 输入：
 *     R ∈ SO(3)
 *
 * 输出：
 *     φ ∈ R³
 */
Vec3 so3_log(const Mat3& R)
{
    const double cos_theta =
        clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);

    const double theta = std::acos(cos_theta);

    if (theta < kSmallAngle)
    {
        return Vec3::Zero();
    }

    if (kPi - theta < kSmallAngle)
    {
        Vec3 axis;

        /*
         * θ ≈ π 时：
         *
         *     R = I + 2[a]^2
         *
         * 因此：
         *
         *     a₁² = (R₁₁ + 1) / 2
         *     a₂² = (R₂₂ + 1) / 2
         *     a₃² = (R₃₃ + 1) / 2
         *
         * 选择数值上最大的分量作为旋转轴主要分量，
         * 以降低除以接近零数值造成的误差。
         */

        const double xx =
            std::max(0.0, (R(0, 0) + 1.0) * 0.5);

        const double yy =
            std::max(0.0, (R(1, 1) + 1.0) * 0.5);

        const double zz =
            std::max(0.0, (R(2, 2) + 1.0) * 0.5);

        if (xx >= yy && xx >= zz)
        {
            axis(0) = std::sqrt(xx);

            if (axis(0) > kEpsilon)
            {
                axis(1) =
                    (R(0, 1) + R(1, 0))
                    / (4.0 * axis(0));

                axis(2) =
                    (R(0, 2) + R(2, 0))
                    / (4.0 * axis(0));
            }
            else
            {
                axis = Vec3::UnitX();
            }
        }
        else if (yy >= zz)
        {
            axis(1) = std::sqrt(yy);

            if (axis(1) > kEpsilon)
            {
                axis(0) =
                    (R(0, 1) + R(1, 0))
                    / (4.0 * axis(1));

                axis(2) =
                    (R(1, 2) + R(2, 1))
                    / (4.0 * axis(1));
            }
            else
            {
                axis = Vec3::UnitY();
            }
        }
        else
        {
            axis(2) = std::sqrt(zz);

            if (axis(2) > kEpsilon)
            {
                axis(0) =
                    (R(0, 2) + R(2, 0))
                    / (4.0 * axis(2));

                axis(1) =
                    (R(1, 2) + R(2, 1))
                    / (4.0 * axis(2));
            }
            else
            {
                axis = Vec3::UnitZ();
            }
        }

        axis.normalize();

        return theta * axis;
    }

    const Mat3 S =
        (R - R.transpose())
        / (2.0 * std::sin(theta));

    return theta * vee(S);
}


/**
 * 【SO3-08】 rot_axis_angle
 *
 * @brief 根据旋转轴和旋转角构造 SO(3) 旋转矩阵。
 *
 * 数学定义：
 *
 *     a = axis / ||axis||
 *
 *     K = [a]^
 *
 *     R = exp(θ[a]^)
 *
 * 根据 Rodrigues 公式：
 *
 *     R = I
 *       + sin(θ)K
 *       + (1-cos(θ))K²
 *
 * 输入：
 *     axis：旋转轴
 *     theta：旋转角，单位 rad
 *
 * 输出：
 *     R ∈ SO(3)
 *
 * 注意：
 *     当 ||axis|| 接近 0 时，无法定义旋转轴，
 *     此时返回单位矩阵。
 */
Mat3 rot_axis_angle(const Vec3& axis, double theta)
{
    const double norm = axis.norm();

    if (norm < kEpsilon)
    {
        return Mat3::Identity();
    }

    const Vec3 unit_axis = axis / norm;
    const Mat3 K = skew(unit_axis);

    return Mat3::Identity()
         + std::sin(theta) * K
         + (1.0 - std::cos(theta)) * K * K;
}

}  // namespace piper_control