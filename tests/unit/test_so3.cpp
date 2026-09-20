#include <cmath>
#include <iostream>
#include <string>
#include <random>

#include "piper_control/lie_group/so3.hpp"
#include "piper_control/common/constants.hpp"
#include "piper_control/common/types.hpp"

namespace
{
using namespace piper_control;

constexpr double kStrictTol = 1e-10;  // 常规角度严格公差
constexpr double kNormalTol = 1e-6;   // 大角度/边界放宽公差

// ========== 矩阵接近校验 ==========
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

// ========== 标量接近校验 ==========
bool expect_scalar_close(
    const std::string& test_name,
    double actual,
    double expected,
    double tol)
{
    double err = std::abs(actual - expected);
    if (err >= tol)
    {
        std::cerr << "[FAIL] " << test_name 
                  << " : error = " << std::scientific << err << "\n";
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

    // ==================== 【SO3-01】skew 反对称矩阵构造 ====================
    {
        const Vec3 v(1.0, 2.0, 3.0);
        const Mat3 expected = 
            (Mat3() <<
             0.0, -3.0,  2.0,
             3.0,  0.0, -1.0,
            -2.0,  1.0,  0.0).finished();
        all_pass &= expect_matrix_close(skew(v), expected, kStrictTol, "SO3-01 skew反对称矩阵构造");
    }

    // ==================== 【SO3-02】skew-vee 互逆还原 ====================
    {
        const Vec3 v(1.0, -2.0, 3.0);
        const Vec3 recovered = vee(skew(v));
        all_pass &= expect_matrix_close(recovered, v, kStrictTol, "SO3-02 skew-vee互逆还原");
    }

    // ==================== 【SO3-03】rot_x 绕X轴旋转 ====================
    {
        const double theta = kPi / 2.0;
        const Mat3 expected = 
            (Mat3() <<
             1.0, 0.0,  0.0,
             0.0, 0.0, -1.0,
             0.0, 1.0,  0.0).finished();
        all_pass &= expect_matrix_close(rot_x(theta), expected, kStrictTol, "SO3-03 rot_x绕X轴90°");
    }

    // ==================== 【SO3-04】rot_y 绕Y轴旋转 ====================
    {
        const double theta = kPi / 2.0;
        const Mat3 expected = 
            (Mat3() <<
             0.0, 0.0, 1.0,
             0.0, 1.0, 0.0,
            -1.0, 0.0, 0.0).finished();
        all_pass &= expect_matrix_close(rot_y(theta), expected, kStrictTol, "SO3-04 rot_y绕Y轴90°");
    }

    // ==================== 【SO3-05】rot_z 绕Z轴旋转 ====================
    {
        const double theta = kPi / 2.0;
        const Mat3 expected = 
            (Mat3() <<
             0.0, -1.0, 0.0,
             1.0,  0.0, 0.0,
             0.0,  0.0, 1.0).finished();
        all_pass &= expect_matrix_close(rot_z(theta), expected, kStrictTol, "SO3-05 rot_z绕Z轴90°");
    }

    // ==================== 【SO3-06】so3_exp 指数映射 ====================
    {
        const double theta = kPi / 3.0;
        const Vec3 rot_vec(0.0, 0.0, theta);
        all_pass &= expect_matrix_close(so3_exp(rot_vec), rot_z(theta), kStrictTol, "SO3-06 so3_exp指数映射");
    }

    // ==================== 【SO3-07】so3_log 对数映射 常规角度往返 ====================
    {
        const double theta = kPi / 4.0;
        const Vec3 axis = Vec3(0.3, 0.5, 0.7).normalized();
        const Mat3 R = rot_axis_angle(axis, theta);
        const Vec3 phi = so3_log(R);

        all_pass &= expect_scalar_close("SO3-07 so3_log模长往返", phi.norm(), theta, kStrictTol);
        all_pass &= expect_scalar_close("SO3-07 so3_log转轴方向", std::abs(phi.normalized().dot(axis)), 1.0, kStrictTol);
    }

    // ==================== 【SO3-08】rot_axis_angle 非单位轴自动归一化 ====================
    {
        const double theta = kPi / 6.0;
        const Vec3 axis_non_unit(0.0, 0.0, 2.5); // 非单位轴
        all_pass &= expect_matrix_close(rot_axis_angle(axis_non_unit, theta), rot_z(theta), kStrictTol, "SO3-08 rot_axis_angle非单位轴归一化");
    }

    // ==================== 边界测试1：小角度数值稳定性 ====================
    {
        const double theta = 1e-8;
        const Vec3 axis(1.0, 0.0, 0.0);
        const Mat3 R = rot_axis_angle(axis, theta);
        const Vec3 phi = so3_log(R);
        all_pass &= expect_scalar_close("SO3-09 小角度边界往返", phi.norm(), theta, 1e-5);
    }

    // ==================== 边界测试2：接近π大角度稳定性（核心校验） ====================
    {
        const double theta = kPi * 0.98; // 176.4°，接近π
        const Vec3 axis = Vec3(0.5, -0.7, 0.3).normalized();
        const Mat3 R = rot_axis_angle(axis, theta);
        const Vec3 phi = so3_log(R);

        all_pass &= expect_scalar_close("SO3-10 大角度模长往返", phi.norm(), theta, kNormalTol);
        all_pass &= expect_scalar_close("SO3-10 大角度转轴方向", std::abs(phi.normalized().dot(axis)), 1.0, kNormalTol);
    }

    // ==================== 普适测试：100组随机轴角exp-log往返 ====================
    {
        std::mt19937 rng(42); // 固定种子，可复现
        std::uniform_real_distribution<double> axis_dist(-1.0, 1.0);
        std::uniform_real_distribution<double> angle_dist(0.0, kPi * 0.99);
        bool random_pass = true;

        for (int i = 0; i < 100; ++i)
        {
            Vec3 axis(axis_dist(rng), axis_dist(rng), axis_dist(rng));
            axis.normalize();
            const double theta = angle_dist(rng);

            const Mat3 R = rot_axis_angle(axis, theta);
            const Vec3 phi = so3_log(R);

            const double err_norm = std::abs(phi.norm() - theta);
            const double err_axis = 1.0 - std::abs(phi.normalized().dot(axis));

            if (err_norm >= kNormalTol || err_axis >= kNormalTol)
            {
                std::cerr << "[FAIL] SO3-11 随机角度 第" << i << "组\n";
                std::cerr << "       模长误差: " << std::scientific << err_norm 
                          << ", 轴偏差: " << std::scientific << err_axis << "\n";
                random_pass = false;
                break;
            }
        }
        if (random_pass) std::cout << "[PASS] SO3-11 100组随机轴角exp-log往返\n";
        all_pass &= random_pass;
    }

    // ==================== 最终结果 ====================
    if (all_pass)
    {
        std::cout << "\n==============================\n";
        std::cout << "All SO(3) tests passed.\n";
        std::cout << "==============================\n";
        return 0;
    }
    else
    {
        std::cerr << "\n==============================\n";
        std::cerr << "Some SO(3) tests FAILED!\n";
        std::cerr << "==============================\n";
        return 1;
    }
}
