#include "piper_control/safety/command_limiter.hpp"

#include <cassert>
#include <cmath>

#include "piper_control/math/math.hpp"

namespace piper_control
{

/*【SAFE-01】
 * @brief 对关节速度指令进行对称限幅。
 *
 * 数学定义：
 *
 *     q_dot_limited(i)
 *       = clamp(q_dot(i),
 *               -|q_dot_max(i)|,
 *                |q_dot_max(i)|)
 *
 * 对每个关节独立进行限幅。
 *
 * 数值/安全注意：
 * - 使用 |q_dot_max(i)| 保证最大速度参数的符号不会改变限幅区间。
 * - 输入维度不一致时通过 assert 直接暴露接口使用错误。
 */
VecXd limit_joint_velocity(
    const VecXd& q_dot,
    const VecXd& q_dot_max)
{
    assert(q_dot.size() == q_dot_max.size());

    VecXd result = q_dot;

    for (Eigen::Index i = 0; i < q_dot.size(); ++i)
    {
        const double max_velocity = std::abs(q_dot_max(i));

        result(i) = clamp(
            q_dot(i),
            -max_velocity,
             max_velocity);
    }

    return result;
}

}  // namespace piper_control