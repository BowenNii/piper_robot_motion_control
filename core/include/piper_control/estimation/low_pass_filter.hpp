#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/*【EST-01】
 * @brief 一阶低通滤波。
 *
 * @param input：输入信号。
 * @param alpha：滤波系数。
 * @param state：上一时刻滤波状态。
 *
 * @return：滤波后的输出信号。
 */
double low_pass_filter(
    double input,
    double alpha,
    double& state);

}  // namespace piper_control