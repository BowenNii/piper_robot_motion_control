#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/*【EST-02】
 * @brief 根据关节位置估计关节速度。
 *
 * @param q：当前关节位置。
 * @param q_previous：上一时刻关节位置。
 * @param dt：采样周期。
 *
 * @return：估计的关节速度。
 */
VecXd estimate_joint_velocity(
    const VecXd& q,
    const VecXd& q_previous,
    double dt);

}  // namespace piper_control