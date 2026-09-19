#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/*【JAC-01】
 * @brief 计算空间 Jacobian。
 *
 * @param S_list：6×n 空间螺旋轴矩阵。
 * @param q：n×1 关节位置向量。
 *
 * @return：6×n 空间 Jacobian。
 */
MatXd jacobian_space(
    const MatXd& S_list,
    const VecXd& q);

/*【JAC-02】
 * @brief 计算本体 Jacobian。
 *
 * @param B_list：6×n 本体螺旋轴矩阵。
 * @param q：n×1 关节位置向量。
 *
 * @return：6×n 本体 Jacobian。
 */
MatXd jacobian_body(
    const MatXd& B_list,
    const VecXd& q);

/*【JAC-03】
 * @brief 根据空间 Jacobian 和关节速度计算空间 twist。
 *
 * @param J_space：6×n 空间 Jacobian。
 * @param q_dot：n×1 关节速度向量。
 *
 * @return：6×1 空间 twist。
 */
Vec6 twist_from_space_jacobian(
    const MatXd& J_space,
    const VecXd& q_dot);

/*【JAC-04】
 * @brief 根据本体 Jacobian 和关节速度计算本体 twist。
 *
 * @param J_body：6×n 本体 Jacobian。
 * @param q_dot：n×1 关节速度向量。
 *
 * @return：6×1 本体 twist。
 */
Vec6 twist_from_body_jacobian(
    const MatXd& J_body,
    const VecXd& q_dot);

}  // namespace piper_control