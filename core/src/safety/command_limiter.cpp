#include "piper_control/safety/command_limiter.hpp"

#include <cassert>

#include "piper_control/math/math.hpp"

namespace piper_control
{

/*【SAFE-01】
 * @brief 对关节速度指令进行限幅。
 */
VecXd limit_joint_velocity(
    const VecXd& q_dot,
    const VecXd& q_dot_max)
{
    assert(q_dot.size() == q_dot_max.size());

    VecXd result = q_dot;

    for (Eigen::Index i = 0; i < q_dot.size(); ++i)
    {
        result(i) = clamp(
            q_dot(i),
            -std::abs(q_dot_max(i)),
             std::abs(q_dot_max(i)));
    }

    return result;
}

}  // namespace piper_control