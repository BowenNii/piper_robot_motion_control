#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/*【SAFE-01】
 * @brief 对关节速度指令进行限幅。
 *
 * @param q_dot：n×1 关节速度指令。
 * @param q_dot_max：n×1 关节最大速度绝对值。
 *
 * @return：限幅后的 n×1 关节速度指令。
 */
VecXd limit_joint_velocity(
    const VecXd& q_dot,
    const VecXd& q_dot_max);

}  // namespace piper_control