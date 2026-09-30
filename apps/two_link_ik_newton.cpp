#include <iomanip>
#include <iostream>

#include "piper_control/common/constants.hpp"
#include "piper_control/common/types.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/kinematics/ik_solver.hpp"
#include "piper_control/lie_group/screw.hpp"
#include "piper_control/lie_group/se3.hpp"

using namespace piper_control;

int main()
{
    // 两连杆平面机械臂：L1 = L2 = 1 m。
    // q = [0, 0] 时，末端位于 (2, 0, 0)。
    Mat4 T0 = Mat4::Identity();
    T0(0, 3) = 2.0;

    // 物体旋量：两个关节轴在末端零位坐标系中的轴上一点。
    MatXd B_list(6, 2);
    B_list.col(0) = make_revolute_screw_axis(
        Vec3(0.0, 0.0, 1.0), Vec3(-2.0, 0.0, 0.0));
    B_list.col(1) = make_revolute_screw_axis(
        Vec3(0.0, 0.0, 1.0), Vec3(-1.0, 0.0, 0.0));

    // 当前 IK 接口需要空间旋量，不能直接传 B_list。
    const MatXd S_list = adjoint(T0) * B_list;

    // 用一组已知关节角生成可达的目标位姿。
    VecXd q_target(2);
    q_target << kPi / 6.0, kPi / 2.0;
    const Mat4 T_target = forward_poe_body(B_list, q_target, T0);

    // 逆解从这组关节角开始迭代，不是从零位开始。
    VecXd q_init(2);
    q_init << 0.0, kPi / 6.0;

    const Mat4 T_init = forward_poe_space(S_list, q_init, T0);

    std::cout << std::fixed << std::setprecision(8);
    std::cout << "初始关节角 q_init (rad): "
              << q_init.transpose() << "\n\n";
    std::cout << "初始位姿 T_init:\n" << T_init << "\n\n";
    std::cout << "目标位姿 T_target:\n" << T_target << "\n\n";

    // 注意第二个参数是零位末端位姿 T0，不是 T_init。
    const IKResult result_ik_newton =
        solve_ik_newton(S_list, T0, T_target, q_init);
    const IKResult result_ik_dls =
        solve_ik_dls(S_list, T0, T_target, q_init);
    const IKResult result_ik_adaptive_dls =
        solve_ik_adaptive_dls(S_list, T0, T_target, q_init);
    const IKResult result_ik_transpose =
        solve_ik_transpose(S_list, T0, T_target, q_init);

    if (!result_ik_newton.success)
    {
        std::cerr << "Newton 逆解失败，停止使用该结果。\n";
        return 1;
    }
    if (!result_ik_dls.success)
    {
        std::cerr << "DLS 逆解失败，停止使用该结果。\n";
        return 1;
    }
    if (!result_ik_adaptive_dls.success)
    {
        std::cerr << "Adaptive DLS 逆解失败，停止使用该结果。\n";
        return 1;
    } 
    if (!result_ik_transpose.success)
    {
        std::cerr << "Transpose 逆解失败，停止使用该结果。\n";
        return 1;
    }

    const Mat4 T_check_ik_newton =
        forward_poe_space(S_list, result_ik_newton.q, T0);
    const Mat4 T_check_ik_dls =
        forward_poe_space(S_list, result_ik_dls.q, T0);
    const Mat4 T_check_ik_adaptive_dls =
        forward_poe_space(S_list, result_ik_adaptive_dls.q, T0);
    const Mat4 T_check_ik_transpose =
        forward_poe_space(S_list, result_ik_transpose.q, T0);

    std::cout << "\nnewton逆解回代位姿 T_check:\n" << T_check_ik_newton << '\n';
    std::cout << "位姿矩阵差的范数: "
              << (T_check_ik_newton - T_target).norm() << '\n';
    std::cout << "\ndls逆解回代位姿 T_check:\n" << T_check_ik_dls << '\n';
    std::cout << "位姿矩阵差的范数: "
              << (T_check_ik_dls - T_target).norm() << '\n';
    std::cout << "\n自适应dls逆解回代位姿 T_check:\n" << T_check_ik_adaptive_dls << '\n';
    std::cout << "位姿矩阵差的范数: "
              << (T_check_ik_adaptive_dls - T_target).norm() << '\n';
    std::cout << "\ntranspose逆解回代位姿 T_check:\n" << T_check_ik_transpose << '\n';
    std::cout << "位姿矩阵差的范数: "
              << (T_check_ik_transpose - T_target).norm() << '\n';
    
    std::cout << "\nnewton逆解关节角 q:\n" << result_ik_newton.q.transpose() << '\n';
    std::cout << "dls逆解关节角 q:\n" << result_ik_dls.q.transpose() << '\n';
    std::cout << "自适应dls逆解关节角 q:\n" << result_ik_adaptive_dls.q.transpose() << '\n';
    std::cout << "transpose逆解关节角 q:\n" << result_ik_transpose.q.transpose() << '\n';
    
    std::cout << "\nnewton逆解迭代次数: " << result_ik_newton.iterations << '\n';
    std::cout << "dls逆解迭代次数: " << result_ik_dls.iterations << '\n';
    std::cout << "自适应dls逆解迭代次数: " << result_ik_adaptive_dls.iterations << '\n';
    std::cout << "transpose逆解迭代次数: " << result_ik_transpose.iterations << '\n';
    
    return 0;
}