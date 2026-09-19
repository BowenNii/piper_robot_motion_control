#include <cassert>
#include <cmath>
#include <iostream>

#include "piper_control/lie_group/se3.hpp"
#include "piper_control/lie_group/so3.hpp"

using namespace piper_control;

namespace
{

void expect_near(double actual, double expected, double tol = 1e-9)
{
    assert(std::abs(actual - expected) < tol);
}

void expect_vector_near(
    const Vec3& actual,
    const Vec3& expected,
    double tol = 1e-9)
{
    assert((actual - expected).norm() < tol);
}

void expect_twist_near(
    const Twist& actual,
    const Twist& expected,
    double tol = 1e-9)
{
    assert((actual - expected).norm() < tol);
}

void expect_matrix_near(
    const Mat3& actual,
    const Mat3& expected,
    double tol = 1e-9)
{
    assert((actual - expected).norm() < tol);
}

void expect_matrix4_near(
    const Mat4& actual,
    const Mat4& expected,
    double tol = 1e-9)
{
    assert((actual - expected).norm() < tol);
}

void expect_matrix6_near(
    const Mat6& actual,
    const Mat6& expected,
    double tol = 1e-9)
{
    assert((actual - expected).norm() < tol);
}

void test_se3_from_rt()
{
    const Mat3 R = rot_z(0.5);
    const Vec3 p(1.0, 2.0, 3.0);

    const Mat4 T = se3_from_rt(R, p);

    expect_matrix_near(
        T.block<3, 3>(0, 0),
        R);

    expect_vector_near(
        T.block<3, 1>(0, 3),
        p);
}

void test_se3_inverse()
{
    const Mat3 R = rot_y(0.4);
    const Vec3 p(1.0, -2.0, 0.5);

    const Mat4 T = se3_from_rt(R, p);
    const Mat4 T_inv = se3_inverse(T);

    const Mat4 I = T * T_inv;

    expect_matrix4_near(
        I,
        Mat4::Identity(),
        1e-8);
}

void test_twist_hat()
{
    Twist xi;
    xi << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

    const Mat4 Xi = twist_hat(xi);

    Mat4 expected = Mat4::Zero();

    expected.block<3, 3>(0, 0) =
        skew(xi.head<3>());

    expected.block<3, 1>(0, 3) =
        xi.tail<3>();

    expect_matrix4_near(Xi, expected);
}

void test_little_ad()
{
    Twist V1;
    V1 << 0.2, -0.4, 0.3, 1.0, 2.0, -1.0;

    Twist V2;
    V2 << -0.5, 0.1, 0.7, -2.0, 0.5, 1.5;

    const Mat6 ad = little_ad(V1);
    const Twist bracket = ad * V2;

    Twist expected;
    expected.head<3>() =
        V1.head<3>().cross(V2.head<3>());

    expected.tail<3>() =
        V1.head<3>().cross(V2.tail<3>()) +
        V1.tail<3>().cross(V2.head<3>());

    expect_twist_near(bracket, expected);
}

void test_adjoint_inverse()
{
    const Mat3 R = rot_x(0.3) * rot_y(-0.4);
    const Vec3 p(0.5, -1.2, 2.0);

    const Mat4 T = se3_from_rt(R, p);

    const Mat6 Ad = adjoint(T);
    const Mat6 Ad_inv = adjoint_inverse(T);

    expect_matrix6_near(
        Ad * Ad_inv,
        Mat6::Identity(),
        1e-8);
}

void test_se3_exp_pure_translation()
{
    Twist xi;
    xi << 0.0, 0.0, 0.0, 1.0, 2.0, 3.0;

    const Mat4 T = se3_exp(xi);

    expect_matrix_near(
        T.block<3, 3>(0, 0),
        Mat3::Identity());

    expect_vector_near(
        T.block<3, 1>(0, 3),
        Vec3(1.0, 2.0, 3.0));
}

void test_se3_exp_pure_rotation()
{
    Twist xi;
    const double theta = 0.7;

    xi << 0.0, 0.0, theta, 0.0, 0.0, 0.0;

    const Mat4 T = se3_exp(xi);

    expect_matrix_near(
        T.block<3, 3>(0, 0),
        rot_z(theta),
        1e-8);

    expect_vector_near(
        T.block<3, 1>(0, 3),
        Vec3::Zero());
}

void test_se3_exp_log_roundtrip()
{
    Twist xi;
    xi << 0.3, -0.2, 0.5, 0.8, -1.1, 0.6;

    const Mat4 T = se3_exp(xi);
    const Twist recovered = se3_log(T);
    const Mat4 T_recovered = se3_exp(recovered);

    expect_twist_near(
        recovered,
        xi,
        1e-8);

    expect_matrix4_near(
        T_recovered,
        T,
        1e-8);
}

void test_se3_log_arbitrary_transform()
{
    const Mat3 R = rot_x(0.2) * rot_y(-0.3) * rot_z(0.4);
    const Vec3 p(0.7, -0.4, 1.3);

    const Mat4 T = se3_from_rt(R, p);

    const Twist xi = se3_log(T);
    const Mat4 T_recovered = se3_exp(xi);

    expect_matrix4_near(
        T_recovered,
        T,
        1e-8);
}

}  // namespace

int main()
{
    test_se3_from_rt();
    test_se3_inverse();
    test_twist_hat();
    test_little_ad();
    test_adjoint_inverse();
    test_se3_exp_pure_translation();
    test_se3_exp_pure_rotation();
    test_se3_exp_log_roundtrip();
    test_se3_log_arbitrary_transform();

    std::cout << "All SE(3) tests passed.\n";

    return 0;
}