#include <cassert>
#include <cmath>
#include <iostream>

#include "piper_control/trajectory/se3_trajectory.hpp"
#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/common/constants.hpp"

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

    //【TRAJ-01】 起点
    {
        const Mat4 T0 =
            se3_from_rt(
                Mat3::Identity(),
                Vec3(0.0, 0.0, 0.0));

        const Mat4 T1 =
            se3_from_rt(
                Mat3::Identity(),
                Vec3(1.0, 2.0, 3.0));

        const Mat4 T =
            interpolate_se3(T0, T1, 0.0);

        assert(is_matrix_close(T, T0));
    }

    //【TRAJ-01】 终点
    {
        const Mat4 T0 =
            se3_from_rt(
                Mat3::Identity(),
                Vec3(0.0, 0.0, 0.0));

        const Mat4 T1 =
            se3_from_rt(
                Mat3::Identity(),
                Vec3(1.0, 2.0, 3.0));

        const Mat4 T =
            interpolate_se3(T0, T1, 1.0);

        assert(is_matrix_close(T, T1));
    }

    //【TRAJ-01】 纯平移中点
    {
        const Mat4 T0 =
            se3_from_rt(
                Mat3::Identity(),
                Vec3(0.0, 0.0, 0.0));

        const Mat4 T1 =
            se3_from_rt(
                Mat3::Identity(),
                Vec3(2.0, 4.0, 6.0));

        const Mat4 expected =
            se3_from_rt(
                Mat3::Identity(),
                Vec3(1.0, 2.0, 3.0));

        const Mat4 T =
            interpolate_se3(T0, T1, 0.5);

        assert(is_matrix_close(
            T,
            expected));
    }

    //【TRAJ-01】 旋转插值
    {
        const Mat4 T0 =
            se3_from_rt(
                Mat3::Identity(),
                Vec3::Zero());

        const Mat4 T1 =
            se3_from_rt(
                rot_z(kPi / 2.0),
                Vec3::Zero());

        const Mat4 expected =
            se3_from_rt(
                rot_z(kPi / 4.0),
                Vec3::Zero());

        const Mat4 T =
            interpolate_se3(T0, T1, 0.5);

        assert(is_matrix_close(
            T,
            expected));
    }

    //【TRAJ-01】 s 超出范围时进行限幅
    {
        const Mat4 T0 =
            Mat4::Identity();

        const Mat4 T1 =
            se3_from_rt(
                Mat3::Identity(),
                Vec3(1.0, 0.0, 0.0));

        const Mat4 T_low =
            interpolate_se3(T0, T1, -1.0);

        const Mat4 T_high =
            interpolate_se3(T0, T1, 2.0);

        assert(is_matrix_close(T_low, T0));
        assert(is_matrix_close(T_high, T1));
    }

    std::cout
        << "All SE(3) trajectory tests passed.\n";

    return 0;
}