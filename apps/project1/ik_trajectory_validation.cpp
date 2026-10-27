#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "experiment_io/ik_trajectory_csv.hpp"
#include "piper_control/common/error_statistics.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/kinematics/ik_solver.hpp"
#include "piper_control/kinematics/jacobian.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/robot_model/piper_model.hpp"
#include "piper_control/safety/joint_limits.hpp"
#include "piper_control/trajectory/se3_trajectory.hpp"
#include "piper_control/trajectory/time_parameterization.hpp"
#include "piper_control/validation/kinematics_validation.hpp"

/*
时间参数化后的连续位姿轨迹离线IK验证，不连接CAN、不发送指令。
运行：./build/debug/ik_trajectory_validation [新的输出CSV路径] [执行时长秒]
示例：./build/debug/ik_trajectory_validation /tmp/ik_timed.csv 10

流程：起终点FK -> 五次时间缩放s(t) -> SE(3)插值 -> DLS逐点逆解
      -> FK回代、限位、增量检查 -> 数值速度/加速度检查 -> 导出CSV。
每个目标使用上一帧已通过全部检查的解作为初值，失败立即停止。

默认10秒、501点；不传输出路径时只打印，不写CSV。输出拒绝覆盖已有文件。
s仍是无量纲路径进度，t才是时间；interpolate_se3保留原来的空间路径。
速度/加速度为差分估计，首尾用单边公式，不强制置零。
0.1 rad/s和0.1 rad/s²仅为自设离线约束，不是厂家参数。
本实验不检查碰撞、采样点之间的连续路径或动力学，不表示可以直接上真机。
*/
namespace {
const char* status_name(piper_control::IKStatus status)
{
    using piper_control::IKStatus;
    switch (status) {
    case IKStatus::kConverged: return "Converged";
    case IKStatus::kMaxIterations: return "MaxIterations";
    case IKStatus::kStalled: return "Stalled";
    case IKStatus::kNumericalFailure: return "NumericalFailure";
    }
    return "Unknown";
}
} // namespace

int main(int argc, char* argv[])
{
    using namespace piper_control;
    try {
        // 0. 可选参数：输出CSV路径、执行时长。总时长改变后dt也会改变。
        if (argc>3 || (argc==2 && std::string(argv[1])=="--help")) {
            std::cout << "Usage: ik_trajectory_validation [new-output.csv] [duration-seconds]\n";
            return argc>3 ? 1 : 0;
        }
        const std::filesystem::path csv_path=argc>=2 ? argv[1] : "";
        experiment_io::IkTrajectoryTiming timing; // 默认duration_s=10
        if (argc==3) {
            std::size_t used=0;
            timing.duration_s=std::stod(argv[2],&used);
            if (used!=std::string(argv[2]).size() || !std::isfinite(timing.duration_s)
                || timing.duration_s<=0)
                throw std::invalid_argument("Duration must be a finite positive number of seconds");
        }
        if (argc>=2) {
            const auto parent=csv_path.has_parent_path() ? csv_path.parent_path()
                                                       : std::filesystem::path(".");
            const auto status=std::filesystem::symlink_status(csv_path);
            if (!std::filesystem::is_directory(parent) || std::filesystem::exists(status))
                throw std::invalid_argument("CSV parent missing or output already exists");
        }
        // 1. 选用限位内部、彼此较近的起终关节角，单位rad。
        const auto model = make_piper_model();
        VecXd q_start(6), q_goal(6);
        q_start << 0.30, 0.30, -0.60, 0.30, 0.30, 0.50;
        q_goal  << 0.35, 0.35, -0.65, 0.35, 0.35, 0.55;
        MatXd endpoints(2, 6);
        endpoints.row(0) = q_start.transpose();
        endpoints.row(1) = q_goal.transpose();
        if (!check_joint_limits(endpoints, model.q_min, model.q_max).ok) {
            std::cerr << "起终关节角超限，请检查实验参数。\n";
            return 1;
        }
        const Mat4 T_start = forward_poe_space(model.S_list, q_start, model.M);
        const Mat4 T_goal = forward_poe_space(model.S_list, q_goal, model.M);

        // 2. 等时间采样：501个点含起终点，默认dt=10/500=0.02秒。
        // max_sample_step是相邻输出解阈值，不是IK内部单次迭代步长。
        constexpr int sample_count = 501;
        static_assert(sample_count>=4, "Time derivatives require at least four samples");
        const double dt=timing.duration_s/(sample_count-1);
        constexpr double max_sample_step_rad = 0.05;
        IKOptions options;
        options.sigma_threshold=1e-6;
        options.q_min = model.q_min;
        options.q_max = model.q_max;
        options.max_iterations = 1000;
        // 数值差分会放大求解误差，因此比旧几何路径实验收紧容差。
        options.position_tolerance = 1e-10;    // m
        options.orientation_tolerance = 1e-10; // rad
        options.lambda = 1e-3;
        options.max_joint_step = 0.1;         // IK内部每次迭代，rad
        options.rotation_weight = 0.2;
        IkValidationConfig validation;
        validation.position_tolerance_m = options.position_tolerance;
        validation.rotation_tolerance_rad = options.orientation_tolerance;
        validation.limit_tolerance_rad = 0.0;

        VecXd previous_q = q_start;
        ErrorAccumulator position_stats, rotation_stats;
        double max_sample_step = 0.0;
        double minimum_sigma = std::numeric_limits<double>::infinity();
        int total_iterations = 0, max_iterations = 0;
        std::vector<experiment_io::IkTrajectorySample> samples;
        samples.reserve(sample_count);
        std::cout << std::scientific << std::setprecision(6)
                  << "Offline IK trajectory validation; solver=DLS\n"
                  << "Quintic s(t); duration_s=" << timing.duration_s << ", dt_s=" << dt
                  << ". No robot commands.\n"
                  << "index  t_s  s  iterations  position_error_m  rotation_error_rad"
                  << "  weighted_sigma_min  max_abs_delta_q_rad\n";

        // 3. 逐点生成目标并求解。起终点可达不保证中间位姿都可达。
        for (int k = 0; k < sample_count; ++k) {
            // t均匀增加；s(t)从慢到快再到慢，不再均匀增加s。
            const double t=timing.duration_s*static_cast<double>(k)/(sample_count-1);
            const auto time_state=quintic_time_scaling(t,timing.duration_s);
            const double s=time_state.s;
            // 此函数决定空间路径，时间缩放决定沿路径前进的快慢。
            const Mat4 target = interpolate_se3(T_start, T_goal, s);
            const auto result = solve_ik_dls(
                model.S_list, model.M, target, previous_q, options);
            if (!result.success) {
                std::cerr << "Stopped at index " << k << ", s=" << s
                          << ", status=" << status_name(result.status)
                          << ", iterations=" << result.iterations
                          << ", position_m=" << result.position_error
                          << ", rotation_rad=" << result.orientation_error
                          << "\nNo later targets were solved.\n";
                return 1;
            }

            // 4. 不能只相信success，使用FK回代及原始关节限位再检查。
            const auto checked = check_ik_solution(
                model.S_list, model.M, result.q, target,
                model.q_min, model.q_max, validation);
            if (!checked.ok) {
                std::cerr << "FK residual or joint-limit check failed at index "
                          << k << '\n';
                return 1;
            }
            
            // 5. 检查上一已接受解 -> 当前候选解，不做2pi绕回或角度裁剪。
            MatXd pair(2, 6);
            pair.row(0) = previous_q.transpose();
            pair.row(1) = result.q.transpose();
            const auto continuity =
                check_joint_continuity(pair, max_sample_step_rad);
            const double sample_step = continuity.delta_rad.cwiseAbs().maxCoeff();
            if (!continuity.ok) {
                std::cerr << "Joint discontinuity at index " << k
                          << ", max delta_q_rad=" << sample_step << '\n';
                return 1;
            }

            // 6. 与IK相同约定：A=D*J_body，记录奇异性，不将它当硬件安全判据。
            MatXd A = adjoint_inverse(checked.T_fk)
                    * jacobian_space(model.S_list, result.q);
            A.topRows(3) *= options.rotation_weight;
            const Eigen::JacobiSVD<MatXd> svd(A);
            const double sigma_min = svd.singularValues().tail(1)(0);
            if (!std::isfinite(sigma_min)) {
                std::cerr << "Nonfinite singular value at index " << k << '\n';
                return 1;
            }
            // 保存完整样本，全部检查通过后才一次性导出CSV。
            experiment_io::IkTrajectorySample row;
            row.index=k; row.s=s; row.iterations=result.iterations;
            row.t_s=t; row.ds_dt=time_state.ds_dt; row.d2s_dt2=time_state.d2s_dt2;
            row.q_rad=result.q;
            row.delta_q_rad=continuity.delta_rad.row(0).transpose();
            row.target=target; row.fk=checked.T_fk; row.error=checked.error;
            row.weighted_sigma_min=sigma_min;
            row.max_abs_delta_q_rad=sample_step;
            samples.push_back(row);
            position_stats.add(checked.error.position_m);
            rotation_stats.add(checked.error.rotation_rad);
            minimum_sigma = std::min(minimum_sigma, sigma_min);
            max_sample_step = std::max(max_sample_step, sample_step);
            total_iterations += result.iterations;
            max_iterations = std::max(max_iterations, result.iterations);
            std::cout << k << ' ' << t << ' ' << s << ' ' << result.iterations << ' '
                      << checked.error.position_m << ' ' << checked.error.rotation_rad
                      << ' ' << sigma_min << ' ' << sample_step << '\n';
            previous_q = result.q; // 只有全部检查通过，才作为下一帧初值。
        }

        // 7. 用等时间关节角计算速度和加速度；不是用delta_q冒充速度。
        MatXd joint_samples(sample_count,6);
        for(int k=0;k<sample_count;++k)
            joint_samples.row(k)=samples[k].q_rad.transpose();
        const auto derivatives=differentiate_joint_samples(joint_samples,dt);
        const double max_velocity=derivatives.velocity.cwiseAbs().maxCoeff();
        const double max_acceleration=derivatives.acceleration.cwiseAbs().maxCoeff();
        std::cout << "\nEstimated peak |dq| (rad/s): " << max_velocity
                  << " / offline limit " << timing.velocity_limit_rad_s
                  << "\nEstimated peak |ddq| (rad/s^2): " << max_acceleration
                  << " / offline limit " << timing.acceleration_limit_rad_s2 << '\n';
        if(max_velocity>timing.velocity_limit_rad_s || max_acceleration>timing.acceleration_limit_rad_s2) {
            std::cerr << "Offline derivative limit exceeded; increase duration or review path/IK. No CSV written.\n";
            return 1;
        }
        for(int k=0;k<sample_count;++k) {
            samples[k].dq_rad_s=derivatives.velocity.row(k).transpose();
            samples[k].ddq_rad_s2=derivatives.acceleration.row(k).transpose();
        }

        // 8. 全部通过后导出并汇总，不要求终态q等于q_goal（可能有其他逆解）。
        const auto p = position_stats.statistics();
        const auto r = rotation_stats.statistics();
        if (argc>=2) {
            experiment_io::write_ik_trajectory_csv(csv_path,samples,model,options,
                                                  q_start,q_goal,max_sample_step_rad,timing);
            std::cout << "CSV saved: " << csv_path << " (" << samples.size() << " rows)\n";
        }
        std::cout << "\nValidated " << sample_count << '/' << sample_count << " samples.\n"
                  << "Position residual mean / RMS / max (m): "
                  << p.mean << " / " << p.rms << " / " << p.max
                  << "\nRotation residual mean / RMS / max (rad): "
                  << r.mean << " / " << r.rms << " / " << r.max
                  << "\nMaximum adjacent joint step (rad): " << max_sample_step
                  << "\nMinimum weighted singular value: " << minimum_sigma
                  << "\nTotal / maximum per-sample IK iterations: "
                  << total_iterations << " / " << max_iterations
                  << "\nFinal q (rad): " << previous_q.transpose()
                  << "\nOffline checks passed; NOT a collision or real-time control test.\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Trajectory validation failed: " << e.what() << '\n';
        return 1;
    }
}
