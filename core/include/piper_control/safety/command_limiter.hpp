#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/**
 * @file command_limiter.hpp
 * @brief 机器人控制指令限幅。
 *
 * Safety / command limiter API:
 *
 * SAFE-01:
 * 对每个关节的速度指令进行对称限幅：
 *
 *     q_dot_limited(i)
 *         = clamp(q_dot(i), -q_dot_max(i), q_dot_max(i))
 *
 * 输入：
 * - q_dot     : n维关节速度指令，单位 rad/s
 * - q_dot_max : n维关节最大允许速度，单位 rad/s
 *
 * 输出：
 * - 限幅后的 n维关节速度指令。
 *
 * 注意：
 * - q_dot 与 q_dot_max 必须具有相同维度。
 * - q_dot_max 的符号不影响限幅结果，函数内部使用其绝对值。
 * - 本函数只负责速度指令限幅，不负责关节位置限制、
 *   加速度限制、力矩限制或通信安全检查。
 */

/**
 * 【SAFE-01】
 *
 * @brief 对关节速度指令进行对称限幅。
 *
 * 数学定义：
 *
 *     q_dot_limited(i)
 *       = clamp(q_dot(i),
 *               -|q_dot_max(i)|,
 *                |q_dot_max(i)|)
 *
 * @param q_dot     n维关节速度指令，单位 rad/s。
 * @param q_dot_max n维关节最大允许速度，单位 rad/s。
 *
 * @return 限幅后的关节速度指令。
 *
 * @note q_dot 与 q_dot_max 的维度必须一致。
 */
VecXd limit_joint_velocity(
    const VecXd& q_dot,
    const VecXd& q_dot_max);

}  // namespace piper_control