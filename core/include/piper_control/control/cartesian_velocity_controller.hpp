#ifndef PIPER_CONTROL_CONTROL_CARTESIAN_VELOCITY_CONTROLLER_HPP
#define PIPER_CONTROL_CONTROL_CARTESIAN_VELOCITY_CONTROLLER_HPP

#include "piper_control/common/types.hpp"

namespace piper_control
{

// ============================================================
// SingularityRegion
//
// @brief
// 表示当前 Jacobian 的奇异性区域状态。
//
// kSafe：
//     sigma_min >= sigma_safe。
//     Jacobian 远离奇异，使用零阻尼 SVD 伪逆，速度不缩放。
//
// kDamped：
//     sigma_critical < sigma_min < sigma_safe。
//     接近奇异，使用自适应 DLS，并平滑降低末端速度。
//
// kCritical：
//     sigma_min <= sigma_critical。
//     严重奇异，保持最小速度比例，避免关节速度失控。
// ============================================================

enum class SingularityRegion
{
    kSafe = 0,
    kDamped = 1,
    kCritical = 2
};


// ============================================================
// CartesianVelocityControllerConfig
//
// @brief
// 笛卡尔速度控制器与奇异抑制策略配置。
//
// 注意：
//     当前默认值仅用于离线开发和单元测试。
//     PiPER 真机参数应根据 Jacobian 扫描与低速实验结果配置。
// ============================================================

struct CartesianVelocityControllerConfig
{
    // 自适应 DLS 最大阻尼系数：
    //
    //     lambda ∈ [0, lambda_max]
    //
    // lambda_max 越大，奇异附近关节速度越受抑制；
    // 但末端速度跟踪误差也会增加。
    double lambda_max = 0.1;

    // 安全最小奇异值阈值：
    //
    //     sigma_min >= sigma_safe
    //
    // 时，不进行速度缩放，DLS 阻尼 lambda = 0。
    double sigma_safe = 0.1;

    // 严重奇异最小奇异值阈值：
    //
    //     sigma_min <= sigma_critical
    //
    // 时，进入 kCritical 状态。
    //
    // 必须满足：
    //
    //     0 < sigma_critical < sigma_safe
    double sigma_critical = 0.01;

    // 严重奇异区的最小速度比例：
    //
    //     V_safe = minimum_speed_scale * V_command
    //
    // 必须位于 [0, 1]。
    //
    // 0.0：严重奇异时停止末端速度命令。
    // 1.0：严重奇异时不降速，不建议用于真机。
    double minimum_speed_scale = 0.1;
};


// ============================================================
// CartesianVelocityResult
//
// @brief
// 笛卡尔速度控制器单个控制周期的计算结果。
// ============================================================

struct CartesianVelocityResult
{
    // 关节速度命令：
    //
    //     q_dot_command = J_DLS^+ * V_safe
    //
    // 维度为 n×1，其中 n 为机器人自由度。
    VecXd q_dot_command;

    // 当前 Jacobian 最小奇异值：
    //
    //     sigma_min = min(sigma_i)
    //
    // sigma_min 越接近 0，越接近奇异。
    double sigma_min = 0.0;

    // 末端目标速度缩放比例：
    //
    //     V_safe = speed_scale * V_command
    //
    // 安全区为 1.0；
    // 严重奇异区为 minimum_speed_scale。
    double speed_scale = 1.0;

    // 当前离散奇异性状态。
    SingularityRegion region =
        SingularityRegion::kSafe;
};


// ============================================================
// 【CTRL-06】 solve_cartesian_velocity
//
// @brief
// 将期望笛卡尔 Twist 映射为经奇异性抑制后的关节速度命令。
//
// ------------------------------------------------------------
// 输入输出关系
// ------------------------------------------------------------
//
// 标准速度微分运动学：
//
//     V_command = J(q) * q_dot
//
// 其中：
//
//     J(q)      : 6×n Jacobian。
//     q_dot     : n×1 关节速度。
//     V_command : 6×1 末端目标 Twist，排列为 [omega; v]。
//
// 本控制器输出：
//
//     q_dot_command = J_DLS^+ * V_safe
//
// 其中：
//
//     V_safe = speed_scale * V_command
//
//     J_DLS^+ 为自适应 DLS 伪逆。
//
// ------------------------------------------------------------
// 奇异性状态判断
// ------------------------------------------------------------
//
// 设：
//
//     sigma_min = minimum_singular_value(J)
//
// 安全区：
//
//     sigma_min >= sigma_safe
//
//     region      = kSafe
//     speed_scale = 1.0
//
// 过渡区：
//
//     sigma_critical < sigma_min < sigma_safe
//
//     region      = kDamped
//
//     t = (sigma_min - sigma_critical)
//         / (sigma_safe - sigma_critical)
//
//     h(t) = 3t^2 - 2t^3
//
//     speed_scale
//     = minimum_speed_scale
//       + (1 - minimum_speed_scale) * h(t)
//
// 严重奇异区：
//
//     sigma_min <= sigma_critical
//
//     region      = kCritical
//     speed_scale = minimum_speed_scale
//
// ------------------------------------------------------------
// 自适应 DLS
// ------------------------------------------------------------
//
// dls_adaptive() 内部计算：
//
//     lambda = 0,
//         sigma_min >= sigma_safe
//
//     lambda = lambda_max
//              * sqrt(1 - (sigma_min / sigma_safe)^2),
//         sigma_min < sigma_safe
//
// 当 lambda = 0 时：
//
//     J_DLS^+ = J^+
//
// 即零阻尼 SVD 伪逆。
//
// 当 lambda > 0 时：
//
//     J_DLS^+
//     = J^T * (J * J^T + lambda^2 * I)^-1
//
// ------------------------------------------------------------
// 安全说明
// ------------------------------------------------------------
//
// 本函数只负责奇异性抑制与关节速度计算。
// 返回的 q_dot_command 必须继续通过：
//
//     safety/command_limiter
//     safety/joint_limits
//     safety/watchdog
//
// 后才能下发至 PiPER 真机。
//
// 输入：
//     J         : 6×n Jacobian。
//     v_command : 6×1 期望末端 Twist，排列为 [omega; v]。
//     config    : 奇异性阈值、DLS 阻尼与速度缩放策略。
//
// 输出：
//     CartesianVelocityResult：
//         q_dot_command：原始关节速度命令。
//         sigma_min：最小奇异值。
//         speed_scale：末端速度缩放比例。
//         region：Safe、Damped 或 Critical 状态。
// ============================================================

[[nodiscard]] CartesianVelocityResult solve_cartesian_velocity(
    const MatXd& J,
    const Vec6& v_command,
    const CartesianVelocityControllerConfig& config);

}  // namespace piper_control

#endif  // PIPER_CONTROL_CONTROL_CARTESIAN_VELOCITY_CONTROLLER_HPP