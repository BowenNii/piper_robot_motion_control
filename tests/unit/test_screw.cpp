#include <cassert>
#include <cmath>
#include <iostream>

#include "piper_control/lie_group/screw.hpp"
#include "piper_control/lie_group/so3.hpp"
#include "piper_control/lie_group/se3.hpp"
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

    //【SCREW-01】 make_revolute_screw_axis
    {
        const Vec3 omega(0.0, 0.0, 1.0);
        const Vec3 point(1.0, 0.0, 0.0);

        const Twist S =
            make_revolute_screw_axis(omega, point);

        Twist expected;
        expected << 0.0, 0.0, 1.0,
                    0.0, -1.0, 0.0;

        assert(is_matrix_close(S, expected));
    }

    //【SCREW-01】 非单位旋转轴自动归一化
    {
        const Vec3 omega(0.0, 0.0, 2.0);
        const Vec3 point(1.0, 0.0, 0.0);

        const Twist S =
            make_revolute_screw_axis(omega, point);

        Twist expected;
        expected << 0.0, 0.0, 1.0,
                    0.0, -1.0, 0.0;

        assert(is_matrix_close(S, expected));
    }

    //【SCREW-02】 make_prismatic_screw_axis
    {
        const Vec3 direction(1.0, 2.0, 0.0);

        const Twist S =
            make_prismatic_screw_axis(direction);

        Twist expected = Twist::Zero();
        expected.tail<3>() =
            direction.normalized();

        assert(is_matrix_close(S, expected));
    }

    //【SCREW-03】 revolute screw exponential
    {
        const Twist S =
            make_revolute_screw_axis(
                Vec3::UnitZ(),
                Vec3::Zero());

        const double theta = kPi / 2.0;

        const Mat4 T =
            screw_exp(S, theta);

        const Mat4 expected =
            se3_from_rt(
                rot_z(theta),
                Vec3::Zero());

        assert(is_matrix_close(T, expected));
    }

    //【SCREW-03】 prismatic screw exponential
    {
        const Twist S =
            make_prismatic_screw_axis(
                Vec3::UnitX());

        const double theta = 0.5;

        const Mat4 T =
            screw_exp(S, theta);

        const Mat4 expected =
            se3_from_rt(
                Mat3::Identity(),
                Vec3(theta, 0.0, 0.0));

        assert(is_matrix_close(T, expected));
    }

    std::cout << "All Screw tests passed.\n";

    return 0;
}