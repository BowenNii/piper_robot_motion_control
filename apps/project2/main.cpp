#include <iomanip>
#include <iostream>

#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/kinematics/ik_solver.hpp"
#include "piper_control/robot_model/piper_model.hpp"
#include "piper_control/validation/pose_error.hpp"

int main()
{
    using namespace piper_control;

    // 1. 准备机械臂模型。
    const auto model = make_piper_model();

    // 2. 设置目标关节角，单位rad。
    // 它用于生成一个已知可达的目标位姿，不直接作为逆解结果。
    VecXd q_goal(6);
    q_goal << 0.3, 0.3, -0.6, 0.3, 0.001, 0.5; // rad

    const Mat4 T_target =
        forward_poe_space(model.S_list, q_goal, model.M);

    // 初值与目标不同，避免一开始就满足收敛条件。
    VecXd q_initial = q_goal;
    q_initial(0) += 0.5;
    q_initial(1) += 0.5;
    q_initial(2) += 0.5;
    q_initial(3) -= 0.5;
    q_initial(4) += 0.5;
    q_initial(5) -= 0.5;

    // 检查目标角度和初值均在限位内。
    const auto within_limits = [&](const VecXd& q) {
        return q.allFinite()
            && (q.array() >= model.q_min.array()).all()
            && (q.array() <= model.q_max.array()).all();
    };

    if (!within_limits(q_goal) || !within_limits(q_initial))
    {
        std::cerr << "目标角度或初始角度超出限位。\n";
        return 1;
    }

    // 4. 设置求解参数。
    IKOptions options;
    options.q_min = model.q_min;
    options.q_max = model.q_max;
    options.max_iterations = 1000;
    options.position_tolerance = 1e-6;     // m
    options.orientation_tolerance = 1e-6;  // rad
    options.lambda = 1e-3;                 // 固定DLS阻尼
    options.lambda_max = 0.1;
    options.max_joint_step = 0.1;          // 每次迭代最大关节增量，rad

    // 5. 保存四个求解器及其名称。
    using Solver = decltype(&solve_ik_newton);

    const Solver solvers[] = {
        solve_ik_newton,
        solve_ik_dls,
        solve_ik_adaptive_dls,
        solve_ik_transpose
    };

    const char* names[] = {
        "Newton", "DLS", "Adaptive", "Transpose"
    };

    std::cout << std::boolalpha << std::scientific
              << std::setprecision(6);

    // 6. 使用同一目标、同一初值，分别求解。
    for (int i = 0; i < 4; ++i)
    {
        const auto result = solvers[i](
            model.S_list, model.M, T_target, q_initial, options);

        std::cout << "\n=== " << names[i] << " ===\n"
                  << "是否收敛：" << result.success << '\n'
                  << "迭代次数：" << result.iterations << '\n';

        if (!result.q.allFinite())
        {
            std::cout << "返回关节角包含无效数值，不能回代。\n";
            continue;
        }

        // 7. 将返回的关节角送回FK，而非只相信success。
        const Mat4 T_check =
            forward_poe_space(model.S_list, result.q, model.M);

        const auto error = pose_error(T_check, T_target);
        const bool limits_ok = within_limits(result.q);
        const bool residual_ok =
            error.position_m <= options.position_tolerance
            && error.rotation_rad <= options.orientation_tolerance;

        std::cout << "求解关节角(rad)：" << result.q.transpose() << '\n'
                  << "回代位置残差(m)：" << error.position_m << '\n'
                  << "回代姿态残差(rad)：" << error.rotation_rad << '\n'
                  << "满足关节限位：" << limits_ok << '\n'
                  << "回代残差达标：" << residual_ok << '\n';
    }

    return 0;
}