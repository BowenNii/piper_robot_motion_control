#include "piper_control/lie_group/so3.hpp"

#include <cassert>
#include <cmath>

#include "piper_control/common/constants.hpp"
#include "piper_control/math/math.hpp"

namespace piper_control
{

/*【SO3-01】
 * @brief 根据 3×1 向量构造 3×3 反对称矩阵。
 */
Mat3 skew(const Vec3& v)
{
    Mat3 S;

    S << 0.0, -v(2), v(1),
         v(2), 0.0, -v(0),
        -v(1), v(0), 0.0;

    return S;
}

/*【SO3-02】
 * @brief 从 3×3 反对称矩阵提取 3×1 向量。
 */
Vec3 vee(const Mat3& skew_matrix)
{
    Vec3 v;

    v << skew_matrix(2, 1),
         skew_matrix(0, 2),
         skew_matrix(1, 0);

    return v;
}

/*【SO3-03】
 * @brief 计算绕 X 轴的旋转矩阵。
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

/*【SO3-04】
 * @brief 计算绕 Y 轴的旋转矩阵。
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

/*【SO3-05】
 * @brief 计算绕 Z 轴的旋转矩阵。
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

/*【SO3-06】
 * @brief 计算 SO(3) 指数映射。
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

/*【SO3-07】
 * @brief 计算 SO(3) 对数映射。
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

        const double xx = std::max(0.0, (R(0, 0) + 1.0) * 0.5);
        const double yy = std::max(0.0, (R(1, 1) + 1.0) * 0.5);
        const double zz = std::max(0.0, (R(2, 2) + 1.0) * 0.5);

        if (xx >= yy && xx >= zz)
        {
            axis(0) = std::sqrt(xx);

            if (axis(0) > kEpsilon)
            {
                axis(1) = (R(0, 1) + R(1, 0)) / (4.0 * axis(0));
                axis(2) = (R(0, 2) + R(2, 0)) / (4.0 * axis(0));
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
                axis(0) = (R(0, 1) + R(1, 0)) / (4.0 * axis(1));
                axis(2) = (R(1, 2) + R(2, 1)) / (4.0 * axis(1));
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
                axis(0) = (R(0, 2) + R(2, 0)) / (4.0 * axis(2));
                axis(1) = (R(1, 2) + R(2, 1)) / (4.0 * axis(2));
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
        (R - R.transpose()) / (2.0 * std::sin(theta));

    return theta * vee(S);
}

/*【SO3-08】
 * @brief 根据旋转轴和旋转角构造 SO(3) 旋转矩阵。
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