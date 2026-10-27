#pragma once
#include <filesystem>
#include <vector>
#include "piper_control/kinematics/ik_solver.hpp"
#include "piper_control/robot_model/piper_model.hpp"
#include "piper_control/validation/pose_error.hpp"

namespace experiment_io {
// 一行对应一个等时间采样点；s仍是无量纲路径进度。
struct IkTrajectorySample {
    int index = 0, iterations = 0;
    double s = 0.0, weighted_sigma_min = 0.0, max_abs_delta_q_rad = 0.0;
    piper_control::Vec6 q_rad = piper_control::Vec6::Zero();
    piper_control::Vec6 delta_q_rad = piper_control::Vec6::Zero();
    piper_control::Mat4 target = piper_control::Mat4::Identity();
    piper_control::Mat4 fk = piper_control::Mat4::Identity();
    piper_control::PoseError error;
    double t_s=0, ds_dt=0, d2s_dt2=0;
    piper_control::Vec6 dq_rad_s=piper_control::Vec6::Zero();
    piper_control::Vec6 ddq_rad_s2=piper_control::Vec6::Zero();
};
struct IkTrajectoryTiming {
    double duration_s=10;
    int path_mode=0; // 0=逐点SE3插值IK；1=终点IK后关节空间五次轨迹。
    // 人工选择的离线分析上限，不是厂家参数或真机安全认证。
    double velocity_limit_rad_s=0.1, acceleration_limit_rad_s2=0.1;
};
// 完整序列 -> 独占创建CSV，保留原75列并追加时间信息，17位有效数字。
// 配置/限位/起终角重复保存在每行，CSV可单独追溯；算法来源为固定DLS。
// 所有路径点成功后才调用；父目录必须存在，拒绝覆盖含符号链接的已有路径。
// I/O失败可能留下不完整文件，必须根据非零退出码识别，不能当作成功输出。
void write_ik_trajectory_csv(const std::filesystem::path& path,
    const std::vector<IkTrajectorySample>& samples,
    const piper_control::PiperModel& model, const piper_control::IKOptions& options,
    const piper_control::VecXd& q_start, const piper_control::VecXd& q_goal,
    double max_sample_step_rad, const IkTrajectoryTiming& timing);
}
