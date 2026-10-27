#pragma once
#include "piper_control/common/types.hpp"

namespace piper_control {
// t与duration均为秒；返回无量纲s及单位为1/s、1/s²的解析导数。
// 五次时间缩放仅保证路径进度的平滑启停，不保证IK路径可达或无碰撞。
struct PathTimeState { double s, ds_dt, d2s_dt2; };
PathTimeState quintic_time_scaling(double t, double duration);

struct JointTimeDerivatives { MatXd velocity, acceleration; };
// 等时间间隔、每行一个样本、每列一个关节，至少4行。
// 内点中心差分；首尾采用单边公式，不人为把端点导数设为零。
// 这是离线数值估计，会放大IK误差，不能证明点间极值或真机安全。
JointTimeDerivatives differentiate_joint_samples(const MatXd& q, double dt);
} // namespace piper_control
