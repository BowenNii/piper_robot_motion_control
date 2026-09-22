#include <array>
#include <cstddef>
#include <iostream>
#include <random>
#include <string>

#include "piper_control/common/constants.hpp"
#include "piper_control/common/types.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/robot_model/piper_model.hpp"

namespace
{

using namespace piper_control;

constexpr double kTol = 1e-9;
constexpr double kUrdfTol = 1e-8;

template <typename DerivedA, typename DerivedB>
bool expect_matrix_close(
    const Eigen::MatrixBase<DerivedA>& actual,
    const Eigen::MatrixBase<DerivedB>& expected,
    double tolerance,
    const std::string& test_name)
{
    if (actual.rows() != expected.rows()
        || actual.cols() != expected.cols())
    {
        std::cerr << "[FAIL] " << test_name
                  << " : dimension mismatch\n";
        return false;
    }

    const double max_error =
        (actual.derived() - expected.derived())
            .array()
            .abs()
            .maxCoeff();

    if (max_error >= tolerance)
    {
        std::cerr << "[FAIL] " << test_name
                  << " : max error = "
                  << std::scientific << max_error << "\n";
        return false;
    }

    std::cout << "[PASS] " << test_name << "\n";
    return true;
}

// 仅作为测试侧的 URDF 零位数据。
// 它不调用 make_piper_model()，用于独立递推 URDF FK 真值。
struct UrdfJointHome
{
    Vec3 xyz;
    Vec3 rpy;
};

Mat4 make_urdf_home_transform(
    const UrdfJointHome& joint)
{
    const Mat3 R =
        rot_z(joint.rpy.z())
        * rot_y(joint.rpy.y())
        * rot_x(joint.rpy.x());

    return se3_from_rt(R, joint.xyz);
}

// 根据 URDF 递推计算 base_link -> link6 位姿：
//
// T(q) = Π [ T_origin_i * Rz(q_i) ]
//
// 此实现与 POE FK 使用不同的递推形式，作为交叉真值。
Mat4 forward_urdf(
    const VecXd& q)
{
    const std::array<UrdfJointHome, kDof> joints =
    {{
        {Vec3(0.0, 0.0, 0.123),
         Vec3(0.0, 0.0, 0.0)},

        {Vec3(0.0, 0.0, 0.0),
         Vec3(1.5707963, -0.1357866, -3.1415926)},

        {Vec3(0.28503, 0.0, 0.0),
         Vec3(0.0, 0.0, -1.7938494)},

        {Vec3(-0.02198, -0.25075, 0.0),
         Vec3(1.5707963, 0.0, 0.0)},

        {Vec3(0.0, 0.0, 0.0),
         Vec3(-1.5707963, 0.0, 0.0)},

        {Vec3(8.8259e-05, -0.091, 0.0),
         Vec3(1.5707963, 0.0, 0.0)}
    }};

    Mat4 T = Mat4::Identity();

    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        const Eigen::Index index =
            static_cast<Eigen::Index>(i);

        // URDF revolute joint：
        //
        // T_parent_child(q_i)
        // = T_origin_i * Rz(q_i)
        T =
            T
            * make_urdf_home_transform(joints[i])
            * se3_from_rt(
                rot_z(q(index)),
                Vec3::Zero());
    }

    return T;
}

}  // namespace

int main()
{
    bool all_pass = true;

    const PiperModel model =
        make_piper_model();

    // ============================================================
    // 【MODEL-01】模型维度与关节数量
    // ============================================================
    {
        const bool dimension_ok =
            model.S_list.rows() == 6
            && model.S_list.cols() == kDof
            && model.B_list.rows() == 6
            && model.B_list.cols() == kDof
            && model.q_min.size() == kDof
            && model.q_max.size() == kDof
            && model.dq_max.size() == kDof;

        if (!dimension_ok)
        {
            std::cerr << "[FAIL] MODEL-01 模型维度错误\n";
        }
        else
        {
            std::cout << "[PASS] MODEL-01 模型维度正确\n";
        }

        all_pass &= dimension_ok;
    }

    // ============================================================
    // 【MODEL-02】M：PiPER URDF 零位末端位姿标称值
    //
    // M = T_base_link_link6(q = 0)
    // ============================================================
    {
        const Mat4 expected_M =
            (Mat4() <<
                0.087155753313, -0.000000076656,
                0.996194697167,  0.056135203286,

                0.000000030802,  1.000000000000,
                0.000000074254,  0.000000000411,

               -0.996194697167,  0.000000024213,
                0.087155753313,  0.213178344081,

                0.0, 0.0, 0.0, 1.0).finished();

        all_pass &= expect_matrix_close(
            model.M,
            expected_M,
            kUrdfTol,
            "MODEL-02 零位末端位姿 M");
    }

    // ============================================================
    // 【MODEL-03】空间旋量基本性质
    //
    // 每个旋转关节的角速度方向必须是单位向量。
    // ============================================================
    {
        bool screw_axes_ok = true;

        for (Eigen::Index i = 0; i < kDof; ++i)
        {
            const double omega_norm =
                model.S_list.col(i).head<3>().norm();

            if (std::abs(omega_norm - 1.0) >= kTol)
            {
                std::cerr
                    << "[FAIL] MODEL-03 S"
                    << i + 1
                    << " 的角速度方向不是单位向量，norm = "
                    << omega_norm << "\n";

                screw_axes_ok = false;
            }
        }

        if (screw_axes_ok)
        {
            std::cout
                << "[PASS] MODEL-03 六根空间旋量角速度方向均单位化\n";
        }

        all_pass &= screw_axes_ok;
    }

    // ============================================================
    // 【MODEL-04】零位 URDF FK、Space POE、Body POE 一致
    // ============================================================
    {
        const VecXd q_zero =
            VecXd::Zero(kDof);

        const Mat4 T_urdf =
            forward_urdf(q_zero);

        const Mat4 T_space =
            forward_poe_space(
                model.S_list,
                q_zero,
                model.M);

        const Mat4 T_body =
            forward_poe_body(
                model.B_list,
                q_zero,
                model.M);

        all_pass &= expect_matrix_close(
            T_urdf,
            model.M,
            kUrdfTol,
            "MODEL-04 URDF 零位递推等于 M");

        all_pass &= expect_matrix_close(
            T_space,
            T_urdf,
            kUrdfTol,
            "MODEL-05 Space POE 零位等于 URDF");

        all_pass &= expect_matrix_close(
            T_body,
            T_urdf,
            kUrdfTol,
            "MODEL-06 Body POE 零位等于 URDF");
    }

    // ============================================================
    // 【MODEL-07】一组手工指定关节角：URDF、Space POE、Body POE
    // ============================================================
    {
        VecXd q(kDof);

        // 每个数均位于 URDF 关节限制内。
        q << 0.4,
             1.1,
            -0.9,
             0.3,
            -0.4,
             0.6;

        const Mat4 T_urdf =
            forward_urdf(q);

        const Mat4 T_space =
            forward_poe_space(
                model.S_list,
                q,
                model.M);

        const Mat4 T_body =
            forward_poe_body(
                model.B_list,
                q,
                model.M);

        all_pass &= expect_matrix_close(
            T_space,
            T_urdf,
            kUrdfTol,
            "MODEL-07 Space POE 与 URDF FK 一致");

        all_pass &= expect_matrix_close(
            T_body,
            T_urdf,
            kUrdfTol,
            "MODEL-08 Body POE 与 URDF FK 一致");
    }

    // ============================================================
    // 【MODEL-09】身体旋量定义验证
    //
    // B_list = Ad_(M^-1) * S_list
    // ============================================================
    {
        const MatXd expected_B_list =
            adjoint_inverse(model.M)
            * model.S_list;

        all_pass &= expect_matrix_close(
            model.B_list,
            expected_B_list,
            kTol,
            "MODEL-09 B_list = Ad(M^-1) * S_list");
    }

    // ============================================================
    // 【MODEL-10】URDF 关节限制
    // ============================================================
    {
        VecXd expected_q_min(kDof);
        VecXd expected_q_max(kDof);
        VecXd expected_dq_max(kDof);

        expected_q_min <<
            -2.6179938,
             0.0,
            -2.9670597,
            -1.7453292,
            -1.2217304,
            -2.0943951;

        expected_q_max <<
             2.6179938,
             3.1415926,
             0.0,
             1.7453292,
             1.2217304,
             2.0943951;

        expected_dq_max <<
            5.0, 5.0, 5.0,
            5.0, 5.0, 5.0;

        all_pass &= expect_matrix_close(
            model.q_min,
            expected_q_min,
            kTol,
            "MODEL-10 关节下限");

        all_pass &= expect_matrix_close(
            model.q_max,
            expected_q_max,
            kTol,
            "MODEL-11 关节上限");

        all_pass &= expect_matrix_close(
            model.dq_max,
            expected_dq_max,
            kTol,
            "MODEL-12 关节速度上限");
    }

    // ============================================================
    // 【MODEL-13】固定随机种子：100 组限位内关节角 FK 交叉验证
    // ============================================================
    {
        std::mt19937 rng(42);
        bool random_pass = true;

        for (int sample = 0; sample < 100; ++sample)
        {
            VecXd q(kDof);

            for (Eigen::Index i = 0; i < kDof; ++i)
            {
                // 留出 5% 余量，不刻意测试机械限位边界；
                // 此处只验证一般构型的运动学等价性。
                const double lower =
                    model.q_min(i)
                    + 0.05
                    * (model.q_max(i) - model.q_min(i));

                const double upper =
                    model.q_max(i)
                    - 0.05
                    * (model.q_max(i) - model.q_min(i));

                std::uniform_real_distribution<double>
                    q_distribution(lower, upper);

                q(i) = q_distribution(rng);
            }

            const Mat4 T_urdf =
                forward_urdf(q);

            const Mat4 T_space =
                forward_poe_space(
                    model.S_list,
                    q,
                    model.M);

            const Mat4 T_body =
                forward_poe_body(
                    model.B_list,
                    q,
                    model.M);

            const double space_error =
                (T_space - T_urdf)
                    .array()
                    .abs()
                    .maxCoeff();

            const double body_error =
                (T_body - T_urdf)
                    .array()
                    .abs()
                    .maxCoeff();

            if (space_error >= kUrdfTol
                || body_error >= kUrdfTol)
            {
                std::cerr
                    << "[FAIL] MODEL-13 第 "
                    << sample
                    << " 组随机关节角，"
                    << "space error = "
                    << std::scientific
                    << space_error
                    << "，body error = "
                    << body_error
                    << "\n";

                random_pass = false;
                break;
            }
        }

        if (random_pass)
        {
            std::cout
                << "[PASS] MODEL-13 100 组随机关节角 FK 交叉验证\n";
        }

        all_pass &= random_pass;
    }

    if (all_pass)
    {
        std::cout
            << "\nAll PiPER model tests passed.\n";

        return 0;
    }

    std::cerr
        << "\nSome PiPER model tests FAILED!\n";

    return 1;
}