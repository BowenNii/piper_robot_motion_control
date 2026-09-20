#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <string>
#include <random>

#include "piper_control/lie_group/screw.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/lie_group/se3.hpp"
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
        std::cerr << "[FAIL] " << test_name << " : dimension mismatch\n";
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

    // ==================== 测试1：旋转关节螺旋轴构造 ====================
    {
        const Vec3 omega(0.0, 0.0, 1.0);
        const Vec3 point(1.0, 0.0, 0.0);
        const Twist S = make_revolute_screw_axis(omega, point);

        Twist expected;
        expected << 0.0, 0.0, 1.0,
                   0.0, -1.0, 0.0;
        all_pass &= expect_matrix_close(S, expected, kTol, "SCREW-01 旋转关节螺旋轴构造");
    }

    // ==================== 测试2：非单位轴自动归一化 ====================
    {
        const Vec3 omega(0.0, 0.0, 2.0);
        const Vec3 point(1.0, 0.0, 0.0);
        const Twist S = make_revolute_screw_axis(omega, point);

        Twist expected;
        expected << 0.0, 0.0, 1.0,
                   0.0, -1.0, 0.0;
        all_pass &= expect_matrix_close(S, expected, kTol, "SCREW-02 非单位轴自动归一化");
    }

    // ==================== 测试3：移动关节螺旋轴构造 ====================
    {
        const Vec3 direction(1.0, 2.0, 0.0);
        const Twist S = make_prismatic_screw_axis(direction);

        Twist expected = Twist::Zero();
        expected.tail<3>() = direction.normalized();
        all_pass &= expect_matrix_close(S, expected, kTol, "SCREW-03 移动关节螺旋轴构造");
    }

    // ==================== 测试4：旋转螺旋指数映射 ====================
    {
        const Twist S = make_revolute_screw_axis(Vec3::UnitZ(), Vec3::Zero());
        const double theta = kPi / 2.0;
        const Mat4 T = screw_exp(S, theta);
        const Mat4 expected = se3_from_rt(rot_z(theta), Vec3::Zero());
        all_pass &= expect_matrix_close(T, expected, kTol, "SCREW-04 旋转螺旋指数映射");
    }

    // ==================== 测试5：移动螺旋指数映射 ====================
    {
        const Twist S = make_prismatic_screw_axis(Vec3::UnitX());
        const double theta = 0.5;
        const Mat4 T = screw_exp(S, theta);
        const Mat4 expected = se3_from_rt(Mat3::Identity(), Vec3(theta, 0.0, 0.0));
        all_pass &= expect_matrix_close(T, expected, kTol, "SCREW-05 移动螺旋指数映射");
    }

    // ==================== 测试6：指数-对数互逆校验 ====================
    {
        const Twist S = make_revolute_screw_axis(Vec3::UnitZ(), Vec3(1.0, 0.0, 0.0));
        const double theta = 0.7;
        const Mat4 T = screw_exp(S, theta);
        const Twist xi = se3_log(T);
        
        // 对数映射结果应等于 S·theta
        all_pass &= expect_matrix_close(xi, S * theta, 1e-9, "SCREW-06 指数-对数互逆");
    }

    // ==================== 测试7：100组随机角度普适性校验 ====================
    {
        std::mt19937 rng(42);
        std::uniform_real_distribution<double> dist(-M_PI, M_PI);
        bool random_pass = true;

        const Twist S = make_revolute_screw_axis(Vec3::UnitZ(), Vec3(1.0, 2.0, 0.0));
        for (int i = 0; i < 100; ++i)
        {
            const double theta = dist(rng);
            const Mat4 T = screw_exp(S, theta);
            const Twist xi = se3_log(T);

            double err = (xi - S * theta).array().abs().maxCoeff();
            if (err >= 1e-9)
            {
                std::cerr << "[FAIL] SCREW-07 随机角度 第" << i << "组，误差: " 
                          << std::scientific << err << "\n";
                random_pass = false;
                break;
            }
        }
        if (random_pass) std::cout << "[PASS] SCREW-07 100组随机角度互逆校验\n";
        all_pass &= random_pass;
    }

    // ==================== 最终结果 ====================
    if (all_pass)
    {
        std::cout << "\n==============================\n";
        std::cout << "All Screw tests passed.\n";
        std::cout << "==============================\n";
        return 0;
    }
    else
    {
        std::cerr << "\n==============================\n";
        std::cerr << "Some Screw tests FAILED!\n";
        std::cerr << "==============================\n";
        return 1;
    }
}
