#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/*【FK-01】
 * @brief 使用 POE 空间形式计算正运动学。
 *
 * @param S_list：6×n 空间螺旋轴矩阵，每列为一个 screw axis。
 * @param q：n×1 关节位置向量。
 * @param M：home configuration 下的末端齐次变换矩阵。
 *
 * @return：当前关节位置 q 下的末端齐次变换矩阵 T(q)。
 */
Mat4 forward_poe(
    const MatXd& S_list,
    const VecXd& q,
    const Mat4& M);

}  // namespace piper_control