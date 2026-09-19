#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/*【SING-01】
 * @brief 计算 Jacobian 的奇异值。
 *
 * @param J：m×n Jacobian 矩阵。
 *
 * @return：奇异值向量。
 */
VecXd singular_values(
    const MatXd& J);

/*【SING-02】
 * @brief 计算 Jacobian 的最小奇异值。
 *
 * @param J：m×n Jacobian 矩阵。
 *
 * @return：最小奇异值。
 */
double minimum_singular_value(
    const MatXd& J);

/*【SING-03】
 * @brief 计算 Jacobian 的条件数。
 *
 * @param J：m×n Jacobian 矩阵。
 *
 * @return：条件数。
 */
double condition_number(
    const MatXd& J);

/*【SING-04】
 * @brief 计算 Jacobian 的可操作度。
 *
 * @param J：m×n Jacobian 矩阵。
 *
 * @return：可操作度。
 */
double manipulability(
    const MatXd& J);

/*【SING-05】
 * @brief 使用 SVD 计算 Jacobian 伪逆。
 *
 * @param J：m×n Jacobian 矩阵。
 * @param tolerance：奇异值截断阈值。
 *
 * @return：n×m Jacobian 伪逆。
 */
MatXd pseudoinverse_svd(
    const MatXd& J,
    double tolerance = 1e-10);

/*【SING-06】
 * @brief 使用阻尼最小二乘法计算 Jacobian 伪逆。
 *
 * @param J：m×n Jacobian 矩阵。
 * @param lambda：阻尼系数。
 *
 * @return：n×m DLS 伪逆。
 */
MatXd dls_pseudoinverse(
    const MatXd& J,
    double lambda);

}  // namespace piper_control