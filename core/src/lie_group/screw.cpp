#include "piper_control/lie_group/screw.hpp"

#include <cassert>

#include "piper_control/common/constants.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/so3.hpp"

namespace piper_control
{

/*【SCREW-01】
 * @brief 根据旋转轴和轴上一点构造旋转关节 screw axis。
 */
Twist make_revolute_screw_axis(
    const Vec3& omega,
    const Vec3& point)
{
    const double norm = omega.norm();

    assert(norm > kEpsilon);

    const Vec3 unit_omega = omega / norm;
    const Vec3 v = -skew(unit_omega) * point;

    Twist S;

    S.head<3>() = unit_omega;
    S.tail<3>() = v;

    return S;
}

/*【SCREW-02】
 * @brief 根据移动方向构造移动关节 screw axis。
 */
Twist make_prismatic_screw_axis(
    const Vec3& direction)
{
    const double norm = direction.norm();

    assert(norm > kEpsilon);

    Twist S = Twist::Zero();

    S.tail<3>() = direction / norm;

    return S;
}

/*【SCREW-03】
 * @brief 计算 screw axis 的指数映射。
 */
Mat4 screw_exp(
    const Twist& S,
    double theta)
{
    return se3_exp(S * theta);
}

}  // namespace piper_control