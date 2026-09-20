#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <string>
#include <random>

#include "piper_control/trajectory/se3_trajectory.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/common/constants.hpp"
#include "piper_control/common/types.hpp"

namespace
{
using namespace piper_control;
constexpr double kTol = 1e-9;

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

    // ==================== 测试1：起点边界 ====================
    {
        const Mat4 T0 = se3_from_rt(Mat3::Identity(), Vec3(0.0, 0.0, 0.0));
        const Mat4 T1 = se3_from_rt(Mat3::Identity(), Vec3(1.0, 2.0, 3.0));
        const Mat4 T = interpolate_se3(T0, T1, 0.0);
        all_pass &= expect_matrix_close(T, T0, kTol, "TRAJ-01 起点边界");
    }

    // ==================== 测试2：终点边界 ====================
    {
        const Mat4 T0 = se3_from_rt(Mat3::Identity(), Vec3(0.0, 0.0, 0.0));
        const Mat4 T1 = se3_from_rt(Mat3::Identity(), Vec3(1.0, 2.0, 3.0));
        const Mat4 T = interpolate_se3(T0, T1, 1.0);
        all_pass &= expect_matrix_close(T, T1, kTol, "TRAJ-02 终点边界");
    }

    // ==================== 测试3：纯平移中点 ====================
    {
        const Mat4 T0 = se3_from_rt(Mat3::Identity(), Vec3(0.0, 0.0, 0.0));
        const Mat4 T1 = se3_from_rt(Mat3::Identity(), Vec3(2.0, 4.0, 6.0));
        const Mat4 expected = se3_from_rt(Mat3::Identity(), Vec3(1.0, 2.0, 3.0));
        const Mat4 T = interpolate_se3(T0, T1, 0.5);
        all_pass &= expect_matrix_close(T, expected, kTol, "TRAJ-03 纯平移中点");
    }

    // ==================== 测试4：纯旋转中点 ====================
    {
        const Mat4 T0 = se3_from_rt(Mat3::Identity(), Vec3::Zero());
        const Mat4 T1 = se3_from_rt(rot_z(kPi / 2.0), Vec3::Zero());
        const Mat4 expected = se3_from_rt(rot_z(kPi / 4.0), Vec3::Zero());
        const Mat4 T = interpolate_se3(T0, T1, 0.5);
        all_pass &= expect_matrix_close(T, expected, kTol, "TRAJ-04 纯旋转中点");
    }

    // ==================== 测试5：平移+旋转复合插值 ====================
    {
        const Mat4 T0 = se3_from_rt(Mat3::Identity(), Vec3(0.0, 0.0, 0.0));
        const Mat4 T1 = se3_from_rt(rot_z(kPi / 2.0), Vec3(1.0, 0.0, 0.0));
        const Mat4 T_half = interpolate_se3(T0, T1, 0.5);
        
        // 验证：再插0.5回到终点
        const Mat4 T_back = interpolate_se3(T_half, T1, 1.0);
        all_pass &= expect_matrix_close(T_back, T1, kTol, "TRAJ-05 复合插值一致性");
    }

    // ==================== 测试6：超范围限幅 ====================
    {
        const Mat4 T0 = Mat4::Identity();
        const Mat4 T1 = se3_from_rt(Mat3::Identity(), Vec3(1.0, 0.0, 0.0));
        const Mat4 T_low = interpolate_se3(T0, T1, -1.0);
        const Mat4 T_high = interpolate_se3(T0, T1, 2.0);
        
        all_pass &= expect_matrix_close(T_low, T0, kTol, "TRAJ-06 负范围限幅");
        all_pass &= expect_matrix_close(T_high, T1, kTol, "TRAJ-07 正范围限幅");
    }

    // ==================== 测试7：100组随机位姿对称性校验 ====================
    {
        std::mt19937 rng(42);
        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        bool random_pass = true;

        for (int i = 0; i < 100; ++i)
        {
            // 随机构造起点终点
            Vec3 axis = Vec3(dist(rng), dist(rng), dist(rng)).normalized();
            double angle = dist(rng) * kPi;
            Vec3 trans(dist(rng), dist(rng), dist(rng));
            const Mat4 T0 = Mat4::Identity();
            const Mat4 T1 = se3_from_rt(rot_axis_angle(axis, angle), trans);

            // 对称性：s 和 1-s 对称
            const Mat4 T_s = interpolate_se3(T0, T1, 0.3);
            const Mat4 T_1s = interpolate_se3(T1, T0, 0.7);
            
            double err = (T_s - T_1s).array().abs().maxCoeff();
            if (err >= kTol)
            {
                std::cerr << "[FAIL] TRAJ-08 随机对称性 第" << i << "组，误差: " 
                          << std::scientific << err << "\n";
                random_pass = false;
                break;
            }
        }
        if (random_pass) std::cout << "[PASS] TRAJ-08 100组随机位姿对称性校验\n";
        all_pass &= random_pass;
    }

    // ==================== 最终结果 ====================
    if (all_pass)
    {
        std::cout << "\n==============================\n";
        std::cout << "All SE(3) trajectory tests passed.\n";
        std::cout << "==============================\n";
        return 0;
    }
    else
    {
        std::cerr << "\n==============================\n";
        std::cerr << "Some SE(3) trajectory tests FAILED!\n";
        std::cerr << "==============================\n";
        return 1;
    }
}
