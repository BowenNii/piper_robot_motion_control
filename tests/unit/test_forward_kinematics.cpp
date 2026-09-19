#include <cassert>
#include <cmath>
#include <iostream>

#include "piper_control/kinematics/forward_kinematics.hpp"
#include "piper_control/common/constants.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/screw.hpp"

namespace
{

constexpr double kTol = 1e-10;

template <typename DerivedA, typename DerivedB>
bool is_matrix_close(
    const Eigen::MatrixBase<DerivedA>& A,
    const Eigen::MatrixBase<DerivedB>& B,
    double tol = kTol)
{
    if (A.rows() != B.rows() || A.cols() != B.cols())
    {
        return false;
    }

    return (A.derived() - B.derived()).array().abs().maxCoeff() < tol;
}

}  // namespace

int main()
{
    using namespace piper_control;

    //【FK-01】 q = 0 时应返回 M
    {
        MatXd S_list(6, 1);
        S_list.col(0) =
            make_revolute_screw_axis(
                Vec3::UnitZ(),
                Vec3::Zero());

        const VecXd q =
            VecXd::Zero(1);

        const Mat4 M =
            se3_from_rt(
                Mat3::Identity(),
                Vec3(1.0, 2.0, 3.0));

        const Mat4 T =
            forward_poe(S_list, q, M);

        assert(is_matrix_close(T, M));
    }

    //【FK-01】 验证空间 POE 顺序：
    // T(q) = exp(Sq) M
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
            forward_poe(S_list, q, M);

        const Mat4 expected =
            se3_from_rt(
                rot_z(kPi / 2.0),
                Vec3(0.0, 1.0, 0.0));

        assert(is_matrix_close(T, expected));
    }

    //【FK-01】 两关节 POE
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
            forward_poe(S_list, q, M);

        const Mat4 expected =
            screw_exp(S_list.col(0), q(0))
            * screw_exp(S_list.col(1), q(1))
            * M;

        assert(is_matrix_close(T, expected));
    }

    std::cout
        << "All forward kinematics tests passed.\n";

    return 0;
}