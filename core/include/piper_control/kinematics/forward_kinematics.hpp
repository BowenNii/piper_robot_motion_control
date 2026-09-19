#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

/**
 * @brief POE空间形式正运动学.
 *
 * T(q) = exp([S1]q1) ... exp([Sn]qn) M
 */
Mat4 forward_poe(
    const MatXd& S_list,
    const VecXd& q,
    const Mat4& M);

}  // namespace piper_control
