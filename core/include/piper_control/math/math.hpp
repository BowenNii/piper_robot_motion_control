#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

#include "piper_control/common/constants.hpp"

namespace piper_control
{

/*【MATH-01】
 * @brief 对数值进行限幅。
 *
 * @param value：待限幅数值。
 * @param low：下限。
 * @param high：上限。
 *
 * @return：限幅后的数值。
 */
template <typename T>
inline T clamp(T value, T low, T high)
{
    if (low > high)
    {
        std::swap(low, high);
    }

    return std::clamp(value, low, high);
}

/*【MATH-02】
 * @brief 将矩阵中绝对值小于阈值的元素置零。
 *
 * @param matrix：输入矩阵。
 * @param eps：截断阈值。
 *
 * @return：处理后的矩阵。
 */
template <typename Derived>
inline Derived trim_small(
    const Eigen::MatrixBase<Derived>& matrix,
    double eps = 1e-10)
{
    Derived result = matrix.derived();

    for (Eigen::Index i = 0; i < result.rows(); ++i)
    {
        for (Eigen::Index j = 0; j < result.cols(); ++j)
        {
            if (std::abs(result(i, j)) < eps)
            {
                result(i, j) = 0.0;
            }
        }
    }

    return result;
}

/*【MATH-03】
 * @brief 将角度从 degree 转换为 rad。
 *
 * @param deg：角度，单位 degree。
 *
 * @return：角度，单位 rad。
 */
inline double deg_to_rad(double deg)
{
    return deg * kPi / 180.0;
}

/*【MATH-04】
 * @brief 将角度从 rad 转换为 degree。
 *
 * @param rad：角度，单位 rad。
 *
 * @return：角度，单位 degree。
 */
inline double rad_to_deg(double rad)
{
    return rad * 180.0 / kPi;
}

}  // namespace piper_control