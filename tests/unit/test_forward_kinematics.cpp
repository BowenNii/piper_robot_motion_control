#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <string>

#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/screw.hpp"
#include "piper_control/common/constants.hpp"
#include "piper_control/common/types.hpp"

namespace
{

using namespace piper_control;

constexpr double kTol = 1e-10;

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
                  << " : dimension mismatch\n";
        return false;
    }

    const double max_err =
        (A.derived() - B.derived()).array().abs().maxCoeff();

    if (max_err >= tol)
    {
        std::cerr << "[FAIL] " << test_name
                  << " : max error = "
                  << std::scientific << max_err << "\n";
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

    // ============================================================
    // 【FK-01】 Space POE：零位返回初始位姿 M
    // ============================================================
    {
        MatXd S_list(6, 1);

        S_list.col(0) =
            make_revolute_screw_axis(
                Vec3::UnitZ(),
                Vec3::Zero());

        const VecXd q = VecXd::Zero(1);

        const Mat4 M =
            se3_from_rt(
                Mat3::Identity(),
                Vec3(1.0, 2.0, 3.0));

        const Mat4 T =
            forward_poe_space(S_list, q, M);

        all_pass &= expect_matrix_close(
            T,
            M,
            kTol,
            "FK-01 Space POE 零位返回 M");
    }

    // ============================================================
    // 【FK-02】 Space POE：单关节旋转解析真值
    // ============================================================
    {
        MatXd S_list(6, 1);

        S_list.col(0) =
            make_revolute_screw_axis(
                Vec3::UnitZ(),
                Vec3::Zero());

        VecXd q(1);
        q(0) = kPi / 2.0;

        const Mat4 M =
            se3_from_rt(
                Mat3::Identity(),
                Vec3(1.0, 0.0, 0.0));

        const Mat4 T =
            forward_poe_space(S_list, q, M);

        const Mat4 expected =
            se3_from_rt(
                rot_z(kPi / 2.0),
                Vec3(0.0, 1.0, 0.0));

        all_pass &= expect_matrix_close(
            T,
            expected,
            kTol,
            "FK-02 Space POE 单关节 90°旋转");
    }

    // ============================================================
    // 【FK-03】 Space POE：两关节指数连乘顺序
    // ============================================================
    {
        MatXd S_list(6, 2);

        S_list.col(0) =
            make_revolute_screw_axis(
                Vec3::UnitZ(),
                Vec3::Zero());

        S_list.col(1) =
            make_revolute_screw_axis(
                Vec3::UnitY(),
                Vec3(1.0, 0.0, 0.0));

        VecXd q(2);
        q << 0.3, -0.2;

        const Mat4 M = Mat4::Identity();

        const Mat4 T =
            forward_poe_space(S_list, q, M);

        const Mat4 expected =
            screw_exp(S_list.col(0), q(0))
            * screw_exp(S_list.col(1), q(1))
            * M;

        all_pass &= expect_matrix_close(
            T,
            expected,
            kTol,
            "FK-03 Space POE 两关节指数连乘");
    }

    // ============================================================
    // 【FK-04】 Space POE：负角度逆变换对称性
    // ============================================================
    {
        MatXd S_list(6, 1);

        S_list.col(0) =
            make_revolute_screw_axis(
                Vec3::UnitZ(),
                Vec3::Zero());

        VecXd q_pos(1);
        VecXd q_neg(1);

        q_pos(0) = 0.5;
        q_neg(0) = -0.5;

        const Mat4 M = Mat4::Identity();

        const Mat4 T_pos =
            forward_poe_space(S_list, q_pos, M);

        const Mat4 T_neg =
            forward_poe_space(S_list, q_neg, M);

        const Mat4 T_inv =
            se3_inverse(T_pos);

        all_pass &= expect_matrix_close(
            T_neg,
            T_inv,
            kTol,
            "FK-04 Space POE 负角度逆变换");
    }

    // ============================================================
    // 【FK-05】 Body POE：零位返回初始位姿 M
    // ============================================================
    {
        MatXd B_list(6, 1);

        B_list.col(0) =
            make_revolute_screw_axis(
                Vec3::UnitZ(),
                Vec3::Zero());

        const VecXd q = VecXd::Zero(1);

        const Mat4 M =
            se3_from_rt(
                Mat3::Identity(),
                Vec3(1.0, 2.0, 3.0));

        const Mat4 T =
            forward_poe_body(B_list, q, M);

        all_pass &= expect_matrix_close(
            T,
            M,
            kTol,
            "FK-05 Body POE 零位返回 M");
    }

    // ============================================================
    // 【FK-06】 Body POE：单关节指数连乘
    // ============================================================
    {
        MatXd B_list(6, 1);

        B_list.col(0) =
            make_revolute_screw_axis(
                Vec3::UnitZ(),
                Vec3::Zero());

        VecXd q(1);
        q(0) = kPi / 2.0;

        const Mat4 M =
            se3_from_rt(
                Mat3::Identity(),
                Vec3(1.0, 0.0, 0.0));

        const Mat4 T =
            forward_poe_body(B_list, q, M);

        const Mat4 expected =
            M * screw_exp(B_list.col(0), q(0));

        all_pass &= expect_matrix_close(
            T,
            expected,
            kTol,
            "FK-06 Body POE 单关节指数连乘");
    }

    // ============================================================
    // 【FK-07】 Space POE 与 Body POE 等价性
    //
    // B_i = Ad_{M^{-1}} S_i
    //
    // 因此：
    //
    // exp(S1 q1)...exp(Sn qn) M
    //
    // =
    //
    // M exp(B1 q1)...exp(Bn qn)
    // ============================================================
    {
        MatXd S_list(6, 2);

        S_list.col(0) =
            make_revolute_screw_axis(
                Vec3::UnitZ(),
                Vec3::Zero());

        S_list.col(1) =
            make_revolute_screw_axis(
                Vec3::UnitY(),
                Vec3(1.0, 0.0, 0.0));

        VecXd q(2);
        q << 0.3, -0.2;

        const Mat4 M =
            se3_from_rt(
                rot_z(0.4),
                Vec3(0.5, 0.3, 0.8));

        // B_i = Ad_{M^{-1}} S_i
        const MatXd B_list =
            adjoint_inverse(M) * S_list;

        const Mat4 T_space =
            forward_poe_space(
                S_list,
                q,
                M);

        const Mat4 T_body =
            forward_poe_body(
                B_list,
                q,
                M);

        all_pass &= expect_matrix_close(
            T_space,
            T_body,
            kTol,
            "FK-07 Space POE 与 Body POE 等价性");
    }

    // ============================================================
    // 【FK-08】 Body POE：两关节指数连乘顺序
    // ============================================================
    {
        MatXd B_list(6, 2);

        B_list.col(0) =
            make_revolute_screw_axis(
                Vec3::UnitZ(),
                Vec3::Zero());

        B_list.col(1) =
            make_revolute_screw_axis(
                Vec3::UnitY(),
                Vec3(1.0, 0.0, 0.0));

        VecXd q(2);
        q << 0.2, -0.35;

        const Mat4 M =
            se3_from_rt(
                rot_x(0.3),
                Vec3(0.4, -0.2, 0.6));

        const Mat4 T =
            forward_poe_body(
                B_list,
                q,
                M);

        const Mat4 expected =
            M
            * screw_exp(B_list.col(0), q(0))
            * screw_exp(B_list.col(1), q(1));

        all_pass &= expect_matrix_close(
            T,
            expected,
            kTol,
            "FK-08 Body POE 两关节指数连乘");
    }

    // ============================================================
    // 最终结果
    // ============================================================
    if (all_pass)
    {
        std::cout << "\n==============================\n";
        std::cout << "All forward kinematics tests passed.\n";
        std::cout << "==============================\n";

        return 0;
    }

    std::cerr << "\n==============================\n";
    std::cerr << "Some forward kinematics tests FAILED!\n";
    std::cerr << "==============================\n";

    return 1;
}