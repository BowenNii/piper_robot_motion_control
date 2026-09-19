#include <cassert>
#include <cmath>
#include <iostream>

#include "piper_control/lie_group/so3.hpp"
#include "piper_control/common/constants.hpp"

namespace
{

constexpr double kTol = 1e-10;

bool is_close(double a, double b, double tol = kTol)
{
    return std::abs(a - b) < tol;
}

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

    //【SO3-01】 skew
    {
        const Vec3 v(1.0, 2.0, 3.0);

        const Mat3 expected =
            (Mat3() <<
                0.0, -3.0,  2.0,
                3.0,  0.0, -1.0,
               -2.0,  1.0,  0.0).finished();

        assert(is_matrix_close(skew(v), expected));
    }

    //【SO3-02】 vee
    {
        const Vec3 v(1.0, 2.0, 3.0);

        assert(is_matrix_close(
            vee(skew(v)),
            v));
    }

    //【SO3-03】 rot_x
    {
        const double theta = kPi / 2.0;

        const Mat3 expected =
            (Mat3() <<
                1.0, 0.0, 0.0,
                0.0, 0.0, -1.0,
                0.0, 1.0, 0.0).finished();

        assert(is_matrix_close(rot_x(theta), expected));
    }

    //【SO3-04】 rot_y
    {
        const double theta = kPi / 2.0;

        const Mat3 expected =
            (Mat3() <<
                0.0, 0.0, 1.0,
                0.0, 1.0, 0.0,
               -1.0, 0.0, 0.0).finished();

        assert(is_matrix_close(rot_y(theta), expected));
    }

    //【SO3-05】 rot_z
    {
        const double theta = kPi / 2.0;

        const Mat3 expected =
            (Mat3() <<
                0.0, -1.0, 0.0,
                1.0,  0.0, 0.0,
                0.0,  0.0, 1.0).finished();

        assert(is_matrix_close(rot_z(theta), expected));
    }

    //【SO3-06】 so3_exp
    {
        const double theta = kPi / 2.0;
        const Vec3 rotation_vector(0.0, 0.0, theta);

        assert(is_matrix_close(
            so3_exp(rotation_vector),
            rot_z(theta)));
    }

    //【SO3-07】 so3_log
    {
        const double theta = kPi / 3.0;
        const Vec3 rotation_vector(0.0, 0.0, theta);

        const Vec3 recovered =
            so3_log(so3_exp(rotation_vector));

        assert(is_matrix_close(recovered, rotation_vector));
    }

    //【SO3-08】 rot_axis_angle
    {
        const double theta = kPi / 4.0;

        // 测试非单位轴，函数内部应进行归一化
        const Vec3 axis(0.0, 0.0, 2.0);

        assert(is_matrix_close(
            rot_axis_angle(axis, theta),
            rot_z(theta)));
    }

    std::cout << "All SO(3) tests passed.\n";

    return 0;
}