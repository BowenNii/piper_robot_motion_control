#ifndef PIPER_CONTROL_CONTROL_CARTESIAN_VELOCITY_CONTROLLER_HPP
#define PIPER_CONTROL_CONTROL_CARTESIAN_VELOCITY_CONTROLLER_HPP

#include "piper_control/common/types.hpp"

namespace piper_control
{

enum class SingularityRegion
{
    kSafe,
    kDamped,
    kCritical
};

struct CartesianVelocityControllerConfig
{
    // 自适应 DLS 的最大阻尼系数。
    double lambda_max = 0.1;

    // 最小奇异值高于该阈值时，使用零阻尼伪逆。
    double sigma_safe = 0.1;

    // 最小奇异值低于该阈值时，进入严重奇异区。
    double sigma_critical = 0.01;

    // 严重奇异区仍允许的最小末端速度比例。
    // 设为 0.0 表示严重奇异时停止末端速度命令。
    double minimum_speed_scale = 0.1;
};

struct CartesianVelocityResult
{
    // 最终关节速度命令，维度为 n×1。
    VecXd q_dot_command;

    // 当前 Jacobian 的最小奇异值。
    double sigma_min = 0.0;

    // 对原始末端速度命令施加的缩放比例。
    double speed_scale = 1.0;

    // 当前奇异性区域。
    SingularityRegion region = SingularityRegion::kSafe;
};

/*
 * @brief
 * 将空间/笛卡尔速度命令映射为关节速度命令。
 *
 * 输入：
 *   J         : 6×n Jacobian，Twist 排列必须为 [omega; v]。
 *   v_command : 6×1 期望末端 Twist。
 *   config    : 奇异性抑制与速度缩放参数。
 *
 * 输出：
 *   q_dot_command : n×1 原始关节速度命令。
 *
 * 注意：
 *   本函数只负责控制策略和奇异性抑制。
 *   输出必须继续经过 safety/command_limiter 等模块后，才能下发真机。
 */
[[nodiscard]] CartesianVelocityResult solve_cartesian_velocity(
    const MatXd& J,
    const Vec6& v_command,
    const CartesianVelocityControllerConfig& config);

}  // namespace piper_control

#endif  // PIPER_CONTROL_CONTROL_CARTESIAN_VELOCITY_CONTROLLER_HPP