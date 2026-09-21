#include <cassert>
#include <cmath>
#include <iostream>

#include "piper_control/control/cartesian_velocity_controller.hpp"

namespace
{

constexpr double kTolerance = 1e-10;

bool is_close(
    double a,
    double b,
    double tolerance = kTolerance)
{
    return std::abs(a - b) < tolerance;
}

template <typename DerivedA, typename DerivedB>
bool is_vector_close(
    const Eigen::MatrixBase<DerivedA>& a,
    const Eigen::MatrixBase<DerivedB>& b,
    double tolerance = kTolerance)
{
    if (a.rows() != b.rows() || a.cols() != b.cols())
    {
        return false;
    }

    return (a.derived() - b.derived())
               .array()
               .abs()
               .maxCoeff()
           < tolerance;
}

}  // namespace

int main()
{
    using namespace piper_control;

    const CartesianVelocityControllerConfig config{
        .lambda_max = 0.1,
        .sigma_safe = 0.1,
        .sigma_critical = 0.01,
        .minimum_speed_scale = 0.2
    };

    const Vec6 v_command =
        Vec6::Ones();

    // ============================================================
    // 安全区：J = I，最小奇异值为 1.0。
    // 应使用全速度，关节速度等于末端速度命令。
    // ============================================================
    {
        const MatXd J =
            MatXd::Identity(6, 6);

        const CartesianVelocityResult result =
            solve_cartesian_velocity(
                J,
                v_command,
                config);

        assert(result.region == SingularityRegion::kSafe);
        assert(is_close(result.sigma_min, 1.0));
        assert(is_close(result.speed_scale, 1.0));
        assert(is_vector_close(
            result.q_dot_command,
            v_command));
    }

    // ============================================================
    // 过渡区：sigma_min = 0.05，位于 critical 和 safe 之间。
    // 应启用 DLS 且速度缩放位于 minimum_speed_scale 与 1 之间。
    // ============================================================
    {
        MatXd J =
            MatXd::Identity(6, 6);

        J(5, 5) = 0.05;

        const CartesianVelocityResult result =
            solve_cartesian_velocity(
                J,
                v_command,
                config);

        assert(result.region == SingularityRegion::kDamped);
        assert(is_close(result.sigma_min, 0.05));

        assert(result.speed_scale > config.minimum_speed_scale);
        assert(result.speed_scale < 1.0);

        assert(result.q_dot_command.allFinite());
    }

    // ============================================================
    // 严重奇异区：最后一维秩亏。
    // 应进入 kCritical，使用最小速度比例，输出不能发散。
    // ============================================================
    {
        MatXd J =
            MatXd::Identity(6, 6);

        J(5, 5) = 0.0;

        const CartesianVelocityResult result =
            solve_cartesian_velocity(
                J,
                v_command,
                config);

        assert(result.region == SingularityRegion::kCritical);
        assert(is_close(result.sigma_min, 0.0));

        assert(is_close(
            result.speed_scale,
            config.minimum_speed_scale));

        assert(result.q_dot_command.allFinite());

        // 完全退化方向上的关节速度应接近 0。
        assert(is_close(
            result.q_dot_command(5),
            0.0));
    }

    std::cout
        << "All Cartesian velocity controller tests passed.\n";

    return 0;
}