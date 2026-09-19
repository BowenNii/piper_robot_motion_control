#include <cassert>
#include <cmath>
#include <iostream>

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

    //【SE3-01】 twist_hat
    {
        Twist xi;
        xi << 1.0, 2.0, 3.0,
              4.0, 5.0, 6.0;

        const Mat4 expected =
            (Mat4() <<
                 0.0, -3.0,  2.0, 4.0,
                 3.0,  0.0, -1.0, 5.0,
                -2.0,  1.0,  0.0, 6.0,
                 0.0,  0.0,  0.0, 0.0).finished();

        assert(is_matrix_close(
            twist_hat(xi),
            expected));
    }

    //【SE3-02】 se3_from_rt
    {
        const Mat3 R = rot_z(kPi / 2.0);
        const Vec3 p(1.0, 2.0, 3.0);

        const Mat4 T = se3_from_rt(R, p);

        assert(is_matrix_close(
            T.block<3, 3>(0, 0),
            R));

        assert(is_matrix_close(
            T.block<3, 1>(0, 3),
            p));

        assert(is_matrix_close(
            T.row(3),
            (Eigen::RowVector4d() << 0.0, 0.0, 0.0, 1.0).finished()));
    }

    //【SE3-03】 se3_inverse
    {
        const Mat4 T = se3_from_rt(
            rot_y(0.3),
            Vec3(0.5, -0.2, 1.0));

        const Mat4 identity =
            T * se3_inverse(T);

        assert(is_matrix_close(
            identity,
            Mat4::Identity()));
    }

    //【SE3-04】 adjoint
    {
        const Mat4 T = se3_from_rt(
            rot_z(0.4),
            Vec3(1.0, 2.0, 3.0));

        const Mat6 Ad = adjoint(T);

        const Mat6 expected =
            (Mat6() <<
                T.block<3, 3>(0, 0), Mat3::Zero(),
                skew(T.block<3, 1>(0, 3)) *
                    T.block<3, 3>(0, 0),
                T.block<3, 3>(0, 0)).finished();

        assert(is_matrix_close(Ad, expected));
    }

    //【SE3-05】 adjoint_inverse
    {
        const Mat4 T = se3_from_rt(
            rot_x(0.4),
            Vec3(0.3, -0.5, 0.8));

        const Mat6 result =
            adjoint(T) * adjoint_inverse(T);

        assert(is_matrix_close(
            result,
            Mat6::Identity()));
    }

    //【SE3-06】 little_ad
    {
        Twist V;
        V << 1.0, 2.0, 3.0,
             4.0, 5.0, 6.0;

        const Mat3 omega_hat = skew(V.head<3>());
        const Mat3 v_hat = skew(V.tail<3>());

        const Mat6 expected =
            (Mat6() <<
                omega_hat, Mat3::Zero(),
                v_hat,     omega_hat).finished();

        assert(is_matrix_close(
            little_ad(V),
            expected));
    }

    //【SE3-07】 se3_exp：纯平移
    {
        Twist xi = Twist::Zero();
        xi.tail<3>() << 0.2, -0.3, 0.5;

        const Mat4 T = se3_exp(xi);

        const Mat4 expected =
            se3_from_rt(
                Mat3::Identity(),
                xi.tail<3>());

        assert(is_matrix_close(T, expected));
    }

    //【SE3-07】 se3_exp：纯旋转
    {
        Twist xi = Twist::Zero();
        xi.head<3>() << 0.0, 0.0, kPi / 2.0;

        const Mat4 T = se3_exp(xi);

        const Mat4 expected =
            se3_from_rt(
                rot_z(kPi / 2.0),
                Vec3::Zero());

        assert(is_matrix_close(T, expected));
    }

    //【SE3-07/08】 Exp-Log round trip
    {
        Twist xi;
        xi << 0.2, -0.1, 0.3,
              0.1,  0.2, -0.15;

        const Mat4 T = se3_exp(xi);
        const Twist xi_recovered = se3_log(T);
        const Mat4 T_recovered = se3_exp(xi_recovered);

        assert(is_matrix_close(
            T_recovered,
            T));
    }

    //【SE3-08】 se3_log：任意 SE(3) round trip
    {
        const Mat4 T = se3_from_rt(
            rot_x(0.2) * rot_y(-0.3),
            Vec3(0.4, -0.7, 0.9));

        const Twist xi = se3_log(T);
        const Mat4 T_recovered = se3_exp(xi);

        assert(is_matrix_close(
            T_recovered,
            T));
    }

    std::cout << "All SE(3) tests passed.\n";

    return 0;
}