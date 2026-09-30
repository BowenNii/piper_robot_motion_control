#include <cassert>
#include <cmath>
#include <iostream>
#include <random>

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

    CartesianVelocityControllerConfig config;

    config.lambda_max = 0.1;
    config.sigma_safe = 0.1;
    config.sigma_critical = 0.01;
    config.minimum_speed_scale = 0.2;

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

    // ============================================================
// 【CTRL-06-04】固定随机种子批量测试
//
// 随机生成 Safe、Damped、Critical 三种区域的对角 Jacobian。
// 对角 Jacobian 的奇异值就是对角线绝对值，因此可明确控制
// sigma_min，并验证控制器的状态、速度缩放和数值稳定性。
// ============================================================
{
    std::mt19937 rng(42);

    std::uniform_real_distribution<double> velocity_dist(
        -1.0,
        1.0);

    bool random_pass = true;

    for (int sample = 0; sample < 300; ++sample)
    {
        SingularityRegion expected_region;
        double expected_sigma_min = 0.0;

        // 每 3 组分别测试 Safe、Damped、Critical。
        const int region_index = sample % 3;

        if (region_index == 0)
        {
            // Safe：
            //
            // sigma_min > sigma_safe
            std::uniform_real_distribution<double> sigma_dist(
                1.1 * config.sigma_safe,
                5.0 * config.sigma_safe);

            expected_sigma_min = sigma_dist(rng);

            expected_region =
                SingularityRegion::kSafe;
        }
        else if (region_index == 1)
        {
            // Damped：
            //
            // sigma_critical < sigma_min < sigma_safe
            std::uniform_real_distribution<double> sigma_dist(
                1.01 * config.sigma_critical,
                0.99 * config.sigma_safe);

            expected_sigma_min = sigma_dist(rng);

            expected_region =
                SingularityRegion::kDamped;
        }
        else
        {
            // Critical：
            //
            // sigma_min <= sigma_critical
            std::uniform_real_distribution<double> sigma_dist(
                0.0,
                config.sigma_critical);

            expected_sigma_min = sigma_dist(rng);

            expected_region =
                SingularityRegion::kCritical;
        }

        // 构造对角 Jacobian。
        //
        // 前五个奇异值设为较大值，最后一个设为目标最小奇异值。
        MatXd J =
            10.0 * config.sigma_safe
            * MatXd::Identity(6, 6);

        J(5, 5) = expected_sigma_min;

        // 构造随机目标末端 Twist。
        Vec6 v_command_random;

        for (int i = 0; i < 6; ++i)
        {
            v_command_random(i) =
                velocity_dist(rng);
        }

        const CartesianVelocityResult result =
            solve_cartesian_velocity(
                J,
                v_command_random,
                config);

        // 1. 最小奇异值应与人为构造的值一致。
        if (!is_close(
                result.sigma_min,
                expected_sigma_min,
                1e-10))
        {
            random_pass = false;
            break;
        }

        // 2. 控制器区域判断必须正确。
        if (result.region != expected_region)
        {
            random_pass = false;
            break;
        }

        // 3. 速度缩放必须始终位于合法范围。
        if (result.speed_scale < config.minimum_speed_scale
            || result.speed_scale > 1.0)
        {
            random_pass = false;
            break;
        }

        // 4. Safe 区域必须不缩放速度。
        if (result.region == SingularityRegion::kSafe
            && !is_close(result.speed_scale, 1.0))
        {
            random_pass = false;
            break;
        }

        // 5. Critical 区域必须使用最小速度比例。
        if (result.region == SingularityRegion::kCritical
            && !is_close(
                result.speed_scale,
                config.minimum_speed_scale))
        {
            random_pass = false;
            break;
        }

        // 6. 无论处于何种奇异性区域，输出都不能出现 NaN 或 Inf。
        if (!result.q_dot_command.allFinite())
        {
            random_pass = false;
            break;
        }
    }

    assert(random_pass);

    std::cout
        << "[PASS] CTRL-06-04 固定随机种子批量测试\n";
}

    std::cout
        << "All Cartesian velocity controller tests passed.\n";

    return 0;
}