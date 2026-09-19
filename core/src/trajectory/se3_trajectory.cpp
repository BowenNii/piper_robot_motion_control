#include "piper_control/trajectory/se3_trajectory.hpp"

#include <algorithm>

#include "piper_control/lie_group/se3.hpp"

namespace piper_control
{

/*【TRAJ-01】
 * @brief 计算两个 SE(3) 位姿之间的指数插值。
 */
Mat4 interpolate_se3(
    const Mat4& T0,
    const Mat4& T1,
    double s)
{
    const double s_clamped = std::clamp(s, 0.0, 1.0);

    const Mat4 T_relative =
        se3_inverse(T0) * T1;

    const Twist xi =
        se3_log(T_relative);

    return T0 * se3_exp(s_clamped * xi);
}

}  // namespace piper_control