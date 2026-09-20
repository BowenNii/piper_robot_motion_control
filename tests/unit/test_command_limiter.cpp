#include <cmath>
#include <iostream>
#include <string>

#include "piper_control/safety/command_limiter.hpp"
#include "piper_control/common/types.hpp"

namespace
{
using namespace piper_control;
constexpr double kTol = 1e-10;

template <typename DerivedA, typename DerivedB>
bool expect_vector_close(
    const Eigen::MatrixBase<DerivedA>& A,
    const Eigen::MatrixBase<DerivedB>& B,
    double tol,
    const std::string& test_name)
{
    if (A.rows() != B.rows() || A.cols() != B.cols())
    {
        std::cerr << "[FAIL] " << test_name 
                  << " : dimension mismatch\n";
        return false;
    }

    double max_err = (A.derived() - B.derived()).array().abs().maxCoeff();
    if (max_err >= tol)
    {
        std::cerr << "[FAIL] " << test_name 
                  << " : max error = " << std::scientific << max_err << "\n";
        return false;
    }
    std::cout << "[PASS] " << test_name << "\n";
    return true;
}

} // namespace

int main()
{
    using namespace piper_control;
    bool all_pass = true;

    // 【SAFE-01】超限速度对称限幅
    {
        VecXd q_dot(6);
        q_dot << 1.0, -2.0, 0.5, -0.3, 3.0, -4.0;
        VecXd q_dot_max(6);
        q_dot_max << 0.5, 1.0, 0.8, 0.2, 2.0, 3.0;
        
        VecXd expected(6);
        expected << 0.5, -1.0, 0.5, -0.2, 2.0, -3.0;
        
        const VecXd result = limit_joint_velocity(q_dot, q_dot_max);
        all_pass &= expect_vector_close(result, expected, kTol, "SAFE-01 超限速度对称限幅");
    }

    // 【SAFE-02】未超限速度保持不变
    {
        VecXd q_dot(3);
        q_dot << 0.1, -0.2, 0.3;
        VecXd q_dot_max(3);
        q_dot_max << 1.0, 1.0, 1.0;
        
        const VecXd result = limit_joint_velocity(q_dot, q_dot_max);
        all_pass &= expect_vector_close(result, q_dot, kTol, "SAFE-02 未超限速度保持");
    }

    // 【SAFE-03】负限值按绝对值限幅
    {
        VecXd q_dot(3);
        q_dot << 2.0, -2.0, 0.5;
        VecXd q_dot_max(3);
        q_dot_max << -1.0, -1.5, -0.2;
        
        VecXd expected(3);
        expected << 1.0, -1.5, 0.2;
        
        const VecXd result = limit_joint_velocity(q_dot, q_dot_max);
        all_pass &= expect_vector_close(result, expected, kTol, "SAFE-03 负限值绝对值处理");
    }

    // 【SAFE-04】边界：刚好等于限值
    {
        VecXd q_dot(2);
        q_dot << 1.0, -1.0;
        VecXd q_dot_max(2);
        q_dot_max << 1.0, 1.0;
        
        const VecXd result = limit_joint_velocity(q_dot, q_dot_max);
        all_pass &= expect_vector_close(result, q_dot, kTol, "SAFE-04 刚好等于限值");
    }

    // 【SAFE-05】边界：零输入
    {
        VecXd q_dot = VecXd::Zero(4);
        VecXd q_dot_max(4);
        q_dot_max << 1.0, 2.0, 3.0, 4.0;
        
        const VecXd result = limit_joint_velocity(q_dot, q_dot_max);
        all_pass &= expect_vector_close(result, VecXd::Zero(4), kTol, "SAFE-05 零速度输入");
    }

    // ==================== 最终结果 ====================
    if (all_pass)
    {
        std::cout << "\n==============================\n";
        std::cout << "All command limiter tests passed.\n";
        std::cout << "==============================\n";
        return 0;
    }
    else
    {
        std::cerr << "\n==============================\n";
        std::cerr << "Some command limiter tests FAILED!\n";
        std::cerr << "==============================\n";
        return 1;
    }
}
