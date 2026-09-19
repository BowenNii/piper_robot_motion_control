#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

struct PiperModel
{
    MatXd S_list;
    MatXd B_list;
    Mat4 M;

    VecXd q_min;
    VecXd q_max;
    VecXd dq_max;
};

/*【MODEL-01】
 * @brief 创建 PiPER 机器人模型。
 *
 * @return：包含 PiPER 运动学参数和关节限制的机器人模型。
 */
PiperModel make_piper_model();

}  // namespace piper_control