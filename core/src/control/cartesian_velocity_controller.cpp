#include "piper_control/control/cartesian_velocity_controller.hpp"

#include <algorithm>
#include <cassert>

#include "piper_control/kinematics/singularity.hpp"

namespace piper_control
{

namespace
{

/*
 * @brief
 * 计算平滑插值函数 smoothstep。
 *
 * 对输入 t ∈ [0, 1]：
 *
 *     h(t) = 3t² - 2t³
 *
 * 满足：
 *
 *     h(0) = 0
 *     h(1) = 1
 *     h'(0) = 0
 *     h'(1) = 0
 *
 * 与线性插值相比，smoothstep 在进入或退出奇异区域时
 * 不会产生速度缩放系数的一阶突变。
 */
[[nodiscard]] double smoothstep(double x)
{
    const double t =
        std::clamp(x, 0.0, 1.0);

    return t * t * (3.0 - 2.0 * t);
}

}  // namespace


/*
 * 【CTRL-06】 solve_cartesian_velocity
 *
 * @brief
 * 将期望笛卡尔 Twist 映射为经奇异抑制后的关节速度命令。
 *
 * ------------------------------------------------------------
 * 总体控制律
 * ------------------------------------------------------------
 *
 *     q_dot_command = J_DLS^+ * V_safe
 *
 * 其中：
 *
 *     J_DLS^+ : 自适应 DLS 伪逆
 *     V_safe  : 经奇异性速度缩放后的末端目标 Twist
 *
 * ------------------------------------------------------------
 * 1. 奇异值计算
 * ------------------------------------------------------------
 *
 * 对 Jacobian：
 *
 *     J = U * Sigma * V^T
 *
 * 最小奇异值：
 *
 *     sigma_min = min(sigma_i)
 *
 * sigma_min 越接近零，代表 Jacobian 越接近奇异。
 *
 * ------------------------------------------------------------
 * 2. 速度缩放策略
 * ------------------------------------------------------------
 *
 * 设：
 *
 *     sigma_safe     : 安全阈值
 *     sigma_critical : 严重奇异阈值
 *     s_min          : minimum_speed_scale
 *
 * 速度缩放系数 s 定义为：
 *
 *                1,                  sigma_min >= sigma_safe
 *
 *     s =          smooth transition, sigma_critical < sigma_min < sigma_safe
 *
 *                s_min,              sigma_min <= sigma_critical
 *
 * 在过渡区中：
 *
 *     t = (sigma_min - sigma_critical)
 *         / (sigma_safe - sigma_critical)
 *
 *     h(t) = 3t² - 2t³
 *
 *     s = s_min + (1 - s_min) * h(t)
 *
 * 最终安全末端速度：
 *
 *     V_safe = s * V_command
 *
 * ------------------------------------------------------------
 * 3. 自适应 DLS 伪逆
 * ------------------------------------------------------------
 *
 * dls_adaptive() 内部采用：
 *
 *     lambda = 0,
 *         sigma_min >= sigma_safe
 *
 *     lambda = lambda_max
 *              * sqrt(1 - (sigma_min / sigma_safe)²),
 *         sigma_min < sigma_safe
 *
 * 当 lambda = 0 时：
 *
 *     J_DLS^+ = J^+
 *
 * 即使用零阻尼 SVD 伪逆。
 *
 * 当 lambda > 0 时：
 *
 *     J_DLS^+
 *     = J^T * (J * J^T + lambda² * I)^-1
 *
 * DLS 阻尼可以防止接近奇异时，关节速度被过度放大。
 *
 * ------------------------------------------------------------
 * 4. 最终关节速度
 * ------------------------------------------------------------
 *
 *     q_dot_command = J_DLS^+ * V_safe
 *
 * 注意：
 *     本函数只计算控制器原始输出。
 *     最终 q_dot_command 必须继续经过 safety/command_limiter，
 *     再下发至 PiPER 真机。
 */
CartesianVelocityResult solve_cartesian_velocity(
    const MatXd& J,
    const Vec6& v_command,
    const CartesianVelocityControllerConfig& config)
{
    // Jacobian 必须将 n 个关节速度映射为 6 维 Twist：
    //
    //     V = J * q_dot
    //
    // 所以 J 的行数必须为 6。
    assert(J.rows() == 6);
    assert(J.cols() > 0);

    // 配置参数合法性检查。
    assert(config.lambda_max >= 0.0);

    assert(config.sigma_safe > 0.0);
    assert(config.sigma_critical > 0.0);

    // 安全阈值必须大于严重奇异阈值。
    assert(config.sigma_safe > config.sigma_critical);

    // 速度比例必须位于 [0, 1]。
    assert(config.minimum_speed_scale >= 0.0);
    assert(config.minimum_speed_scale <= 1.0);

    CartesianVelocityResult result;

    // --------------------------------------------------------
    // Step 1：计算当前 Jacobian 的最小奇异值。
    //
    //     sigma_min = min(sigma_i)
    // --------------------------------------------------------
    result.sigma_min =
        minimum_singular_value(J);

    // --------------------------------------------------------
    // Step 2：按照 sigma_min 所在区间确定奇异性状态和速度比例。
    // --------------------------------------------------------

    // 安全区：
    //
    //     sigma_min >= sigma_safe
    //
    // 不降速：
    //
    //     speed_scale = 1.0
    if (result.sigma_min >= config.sigma_safe)
    {
        result.region =
            SingularityRegion::kSafe;

        result.speed_scale = 1.0;
    }

    // 严重奇异区：
    //
    //     sigma_min <= sigma_critical
    //
    // 使用配置的最小速度比例：
    //
    //     speed_scale = minimum_speed_scale
    else if (result.sigma_min <= config.sigma_critical)
    {
        result.region =
            SingularityRegion::kCritical;

        result.speed_scale =
            config.minimum_speed_scale;
    }

    // 过渡区：
    //
    //     sigma_critical < sigma_min < sigma_safe
    //
    // 平滑地将 speed_scale 从 minimum_speed_scale 提升至 1.0。
    else
    {
        result.region =
            SingularityRegion::kDamped;

        // 将 sigma_min 从区间：
        //
        //     [sigma_critical, sigma_safe]
        //
        // 归一化映射至：
        //
        //     [0, 1]
        const double normalized_sigma =
            (result.sigma_min - config.sigma_critical)
            / (config.sigma_safe - config.sigma_critical);

        // 使用 smoothstep 减少进入/退出过渡区时的突变。
        const double transition =
            smoothstep(normalized_sigma);

        // 速度缩放：
        //
        //     speed_scale
        //     = minimum_speed_scale
        //       + (1 - minimum_speed_scale) * transition
        result.speed_scale =
            config.minimum_speed_scale
            + (1.0 - config.minimum_speed_scale) * transition;
    }

    // --------------------------------------------------------
    // Step 3：缩放原始目标末端速度。
    //
    //     V_safe = speed_scale * V_command
    // --------------------------------------------------------
    const Vec6 v_safe =
        result.speed_scale * v_command;

    // --------------------------------------------------------
    // Step 4：获取自适应 DLS 伪逆。
    //
    // 远离奇异点：
    //
    //     J_pinv = J^+
    //
    // 接近奇异点：
    //
    //     J_pinv = J_DLS^+
    // --------------------------------------------------------
    const MatXd J_pinv =
        dls_adaptive(
            J,
            config.lambda_max,
            config.sigma_safe);

    // --------------------------------------------------------
    // Step 5：计算原始关节速度命令。
    //
    //     q_dot_command = J_pinv * V_safe
    //
    // 维度：
    //
    //     (n×6) * (6×1) = (n×1)
    // --------------------------------------------------------
    result.q_dot_command =
        J_pinv * v_safe;

    return result;
}

}  // namespace piper_control