#include "piper_control/trajectory/se3_trajectory.hpp"

#include <algorithm>

#include "piper_control/lie_group/se3.hpp"

namespace piper_control
{

/*【TRAJ-01】
 * @brief 计算两个 SE(3) 位姿之间的指数插值。
 *
 * 数学实现：
 *
 *     T_relative = T0^{-1} T1
 *
 *     ξ = log(T_relative)
 *
 *     T(s) = T0 exp(s ξ)
 *
 * 该方法保证轨迹在 SE(3) 流形上进行插值，而不是分别
 * 对 4×4 齐次变换矩阵的元素进行线性插值。
 *
 * 数值/边界处理：
 * - s < 0 时使用 0。
 * - s > 1 时使用 1。
 * - 因此输出始终位于 T0 到 T1 的插值区间内。
 */
Mat4 interpolate_se3(
    const Mat4& T0,
    const Mat4& T1,
    double s)
{
    const double s_clamped =
        std::clamp(s, 0.0, 1.0);

    const Mat4 T_relative =
        se3_inverse(T0) * T1;

    const Twist xi =
        se3_log(T_relative);

    return T0 * se3_exp(s_clamped * xi);
}

}  // namespace piper_control