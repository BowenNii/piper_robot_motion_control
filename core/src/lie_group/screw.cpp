#include "piper_control/lie_group/screw.hpp"

#include <cassert>

#include "piper_control/common/constants.hpp"
#include "piper_control/lie_group/se3.hpp"

namespace piper_control
{

Twist make_revolute_screw_axis(
    const Vec3& omega,
    const Vec3& point)
{
    const double norm = omega.norm();

    assert(norm > kEpsilon);

    const Vec3 omega_unit = omega / norm;
    const Vec3 v = -omega_unit.cross(point);

    Twist S;

    S.head<3>() = omega_unit;
    S.tail<3>() = v;

    return S;
}

Twist make_prismatic_screw_axis(
    const Vec3& direction)
{
    const double norm = direction.norm();

    assert(norm > kEpsilon);

    Twist S = Twist::Zero();

    S.tail<3>() = direction / norm;

    return S;
}

Mat4 screw_exp(
    const Twist& S,
    double theta)
{
    return se3_exp(S * theta);
}

}  // namespace piper_control