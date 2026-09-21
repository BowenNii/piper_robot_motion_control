#include "piper_control/control/cartesian_velocity_controller.hpp"

#include <algorithm>
#include <cassert>

#include "piper_control/kinematics/singularity.hpp"

namespace piper_control
{

namespace
{

[[nodiscard]] double smoothstep(double x)
{
    const double t = std::clamp(x, 0.0, 1.0);

    return t * t * (3.0 - 2.0 * t);
}

}  // namespace

CartesianVelocityResult solve_cartesian_velocity(
    const MatXd& J,
    const Vec6& v_command,
    const CartesianVelocityControllerConfig& config)
{
    assert(J.rows() == 6);
    assert(J.cols() > 0);

    assert(config.lambda_max >= 0.0);
    assert(config.sigma_safe > 0.0);
    assert(config.sigma_critical > 0.0);
    assert(config.sigma_safe > config.sigma_critical);

    assert(config.minimum_speed_scale >= 0.0);
    assert(config.minimum_speed_scale <= 1.0);

    CartesianVelocityResult result;

    result.sigma_min = minimum_singular_value(J);

    if (result.sigma_min >= config.sigma_safe)
    {
        result.region = SingularityRegion::kSafe;
        result.speed_scale = 1.0;
    }
    else if (result.sigma_min <= config.sigma_critical)
    {
        result.region = SingularityRegion::kCritical;
        result.speed_scale = config.minimum_speed_scale;
    }
    else
    {
        result.region = SingularityRegion::kDamped;

        const double normalized_sigma =
            (result.sigma_min - config.sigma_critical)
            / (config.sigma_safe - config.sigma_critical);

        const double transition =
            smoothstep(normalized_sigma);

        result.speed_scale =
            config.minimum_speed_scale
            + (1.0 - config.minimum_speed_scale) * transition;
    }

    const Vec6 v_safe =
        result.speed_scale * v_command;

    // dls_adaptive 内部会根据 sigma_safe 选择：
    //
    // 1. lambda = 0：零阻尼 SVD 伪逆；
    // 2. lambda > 0：DLS 伪逆。
    const MatXd J_pinv =
        dls_adaptive(
            J,
            config.lambda_max,
            config.sigma_safe);

    result.q_dot_command =
        J_pinv * v_safe;

    return result;
}

}  // namespace piper_control