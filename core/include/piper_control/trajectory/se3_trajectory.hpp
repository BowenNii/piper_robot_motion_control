#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/**
 * @file se3_trajectory.hpp
 * @brief SE(3) 位姿轨迹插值。
 *
 * TRAJ-01:
 * 给定起始位姿 T0、目标位姿 T1 和插值系数 s，
 * 使用 SE(3) 的指数映射在两个位姿之间进行插值：
 *
 *     T(s) = T0 * exp( s * log(T0^{-1} T1) )
 *
 * 其中：
 * - T0, T1 ∈ SE(3)
 * - s ∈ [0, 1]
 * - s = 0 时返回 T0
 * - s = 1 时返回 T1
 *
 * 输入：
 * - T0：起始 SE(3) 位姿。
 * - T1：目标 SE(3) 位姿。
 * - s ：插值系数。
 *
 * 输出：
 * - 插值得到的 SE(3) 位姿。
 *
 * 注意：
 * - s 会被限制到 [0, 1]。
 * - 旋转部分通过 SO(3) logarithm / exponential 进行插值。
 * - 平移部分通过 SE(3) logarithm / exponential 一致地进行插值。
 * - T0 与 T1 必须采用相同的坐标系约定。
 */

/**
 * 【TRAJ-01】
 *
 * @brief 计算两个 SE(3) 位姿之间的指数插值。
 *
 * 数学定义：
 *
 *     T_relative = T0^{-1} T1
 *
 *     ξ = log(T_relative)
 *
 *     T(s) = T0 exp(s ξ)
 *
 * 其中 s 会首先被限制到 [0, 1]。
 *
 * @param T0 起始 SE(3) 位姿。
 * @param T1 目标 SE(3) 位姿。
 * @param s  插值系数，通常位于 [0, 1]。
 *
 * @return 插值得到的 SE(3) 位姿。
 */
Mat4 interpolate_se3(
    const Mat4& T0,
    const Mat4& T1,
    double s);

}  // namespace piper_control