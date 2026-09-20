#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <random>
#include <string>

#include "piper_control/kinematics/jacobian.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/lie_group/screw.hpp"
#include "piper_control/common/types.hpp"

namespace
{
using namespace piper_control;

constexpr double kStrictTol    = 1e-9;
constexpr double kNumericalTol = 1e-6;
constexpr double kEps          = 1e-7;

// ========== 通用校验工具 ==========
template <typename DerivedA, typename DerivedB>
bool expect_matrix_close(
    const Eigen::MatrixBase<DerivedA>& A,
    const Eigen::MatrixBase<DerivedB>& B,
    double tol,
    const std::string& test_name)
{
    if (A.rows() != B.rows() || A.cols() != B.cols())
    {
        std::cerr << "[FAIL] " << test_name 
                  << " : dimension mismatch (" 
                  << A.rows() << "x" << A.cols() << " vs " 
                  << B.rows() << "x" << B.cols() << ")\n";
        return false;
    }

    double max_err = (A.derived() - B.derived()).array().abs().maxCoeff();
    if (max_err >= tol)
    {
        std::cerr << "[FAIL] " << test_name 
                  << " : max error = " << std::scientific << max_err 
                  << ", tol = " << tol << "\n";
        return false;
    }
    std::cout << "[PASS] " << test_name << "\n";
    return true;
}

// ========== 数值法空间雅可比（和你接口一致，无M） ==========
MatXd numerical_jacobian_space(
    const MatXd& S_list,
    const VecXd& q)
{
    const int n = q.size();
    MatXd J = MatXd::Zero(6, n);
    const Mat4 T = forward_poe_space(S_list, q, Mat4::Identity());

    for (int i = 0; i < n; ++i)
    {
        VecXd q_plus = q;
        VecXd q_minus = q;
        q_plus(i) += kEps;
        q_minus(i) -= kEps;

        const Mat4 T_plus  = forward_poe_space(S_list, q_plus, Mat4::Identity());
        const Mat4 T_minus = forward_poe_space(S_list, q_minus, Mat4::Identity());
        const Mat4 dT      = (T_plus - T_minus) / (2.0 * kEps);
        const Mat4 V_hat   = dT * se3_inverse(T);

        Twist V_numerical;
        V_numerical.head<3>() = vee(V_hat.block<3, 3>(0, 0));
        V_numerical.tail<3>() = V_hat.block<3, 1>(0, 3);
        J.col(i) = V_numerical;
    }
    return J;
}

// ========== 构造零位体螺旋轴B_list ==========
// 由空间螺旋轴S_list + 零位末端变换，得到零位体螺旋轴
MatXd make_body_screws(const MatXd& S_list)
{
    const Mat4 T0 = forward_poe_space(
        S_list, VecXd::Zero(S_list.cols()), Mat4::Identity());
    const Mat6 Adj_inv = adjoint(se3_inverse(T0));
    return Adj_inv * S_list;
}

} // namespace

int main()
{
    using namespace piper_control;
    bool all_pass = true;

    // ==================== 测试用2轴机械臂 ====================
    MatXd S_list(6, 2);
    S_list.col(0) = make_revolute_screw_axis(Vec3::UnitZ(), Vec3::Zero());
    S_list.col(1) = make_revolute_screw_axis(Vec3::UnitZ(), Vec3(1.0, 0.0, 0.0));
    const MatXd B_list = make_body_screws(S_list); // 构造体螺旋轴

    VecXd q(2);
    q << 0.3, -0.2;

    // ==================== 测试1：零位空间雅可比 = 空间螺旋轴 ====================
    {
        const VecXd q_zero = VecXd::Zero(2);
        const MatXd J = jacobian_space(S_list, q_zero);
        all_pass &= expect_matrix_close(J, S_list, kStrictTol, "JAC-01 零位空间雅可比");
    }

    // ==================== 测试2：空间雅可比 vs 数值微分 ====================
    {
        const MatXd J_analytical = jacobian_space(S_list, q);
        const MatXd J_numerical  = numerical_jacobian_space(S_list, q);
        all_pass &= expect_matrix_close(J_analytical, J_numerical, kNumericalTol, "JAC-02 空间雅可比数值校验");
    }

    // ==================== 测试3：零位体雅可比 = 体螺旋轴 ====================
    {
        const VecXd q_zero = VecXd::Zero(2);
        const MatXd J_body = jacobian_body(B_list, q_zero);
        all_pass &= expect_matrix_close(J_body, B_list, kStrictTol, "JAC-03 零位体雅可比");
    }

    // ==================== 测试4：空间-体雅可比伴随变换关系（核心验证） ====================
    // 核心性质：J_body(q) = Ad(T(q)⁻¹) * J_space(q)
    {
        const MatXd J_space = jacobian_space(S_list, q);
        const MatXd J_body  = jacobian_body(B_list, q);

        const Mat4 T = forward_poe_space(S_list, q, Mat4::Identity());
        const Mat6 Adj_Tinv = adjoint(se3_inverse(T));
        const MatXd J_body_by_adj = Adj_Tinv * J_space;

        all_pass &= expect_matrix_close(J_body, J_body_by_adj, kStrictTol, "JAC-04 空间-体雅可比伴随变换");
    }

    // ==================== 测试5：雅可比速度映射 ====================
    {
        const MatXd J_space = jacobian_space(S_list, q);
        VecXd q_dot(2);
        q_dot << 0.5, -0.2;

        const Twist V_space = J_space * q_dot;
        const Twist V_check = twist_from_space_jacobian(J_space, q_dot);

        all_pass &= expect_matrix_close(V_space, V_check, kStrictTol, "JAC-05 空间雅可比速度映射");
    }

    // ==================== 测试6：100组随机角度数值校验 ====================
    {
        std::mt19937 rng(42);
        std::uniform_real_distribution<double> dist(-M_PI, M_PI);
        bool random_pass = true;

        for (int i = 0; i < 100; ++i)
        {
            VecXd q_rand(2);
            q_rand << dist(rng), dist(rng);

            const MatXd J_ana = jacobian_space(S_list, q_rand);
            const MatXd J_num = numerical_jacobian_space(S_list, q_rand);

            double err = (J_ana - J_num).array().abs().maxCoeff();
            if (err >= kNumericalTol)
            {
                std::cerr << "[FAIL] JAC-06 随机角度 第" << i << "组，误差: " 
                          << std::scientific << err << "\n";
                random_pass = false;
                break;
            }
        }
        if (random_pass) std::cout << "[PASS] JAC-06 100组随机角度数值校验\n";
        all_pass &= random_pass;
    }

    // ==================== 最终结果 ====================
    if (all_pass)
    {
        std::cout << "\n==============================\n";
        std::cout << "All Jacobian tests passed.\n";
        std::cout << "==============================\n";
        return 0;
    }
    else
    {
        std::cerr << "\n==============================\n";
        std::cerr << "Some Jacobian tests FAILED!\n";
        std::cerr << "==============================\n";
        return 1;
    }
}
