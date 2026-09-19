#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/*【SCREW-01】
 * @brief 根据旋转轴和轴上一点构造旋转关节 screw axis。
 *
 * @param omega：3×1 旋转轴方向向量。
 * @param point：3×1 旋转轴上一点。
 *
 * @return：6×1 旋转关节 screw axis。
 */
Twist make_revolute_screw_axis(
    const Vec3& omega,
    const Vec3& point);

/*【SCREW-02】
 * @brief 根据移动方向构造移动关节 screw axis。
 *
 * @param direction：3×1 移动方向向量。
 *
 * @return：6×1 移动关节 screw axis。
 */
Twist make_prismatic_screw_axis(
    const Vec3& direction);

/*【SCREW-03】
 * @brief 计算 screw axis 的指数映射。
 *
 * @param S：6×1 screw axis。
 * @param theta：关节变量。
 *
 * @return：4×4 齐次变换矩阵 exp([S]theta)。
 */
Mat4 screw_exp(
    const Twist& S,
    double theta);

}  // namespace piper_control