#include <cassert>
#include <cmath>
#include <iostream>

#include "piper_control/lie_group/so3.hpp"

using namespace piper_control;

int main()
{

    // Test 1:
    // skew -> vee
    {
        const Vec3 w(1.0, 2.0, 3.0);

        const Mat3 S = skew(w);

        const Vec3 w_recovered = vee(S);

        assert(
            (w - w_recovered).norm()
            < 1e-12);
    }

    // Test 2:
    // SO(3) exponential
    {
        constexpr double pi = 3.14159265358979323846;
        const Vec3 phi(0.0, 0.0, pi / 2.0);

        const Mat3 R = so3_exp(phi);

        const Mat3 R_expected = (Mat3() <<
            0.0, -1.0, 0.0,
            1.0,  0.0, 0.0,
            0.0,  0.0, 1.0).finished();

        assert(
            (R - R_expected).norm()
            < 1e-12);
    }

    // Test 3:
    // Exp -> Log
    {
        const Vec3 phi(
            0.3,
            -0.2,
            0.5);

        const Mat3 R =
            so3_exp(phi);

        const Vec3 phi_recovered =
            so3_log(R);

        assert(
            (phi - phi_recovered).norm()
            < 1e-10);
    }

    // Test 4:
    // R^T R = I
    {
        const Vec3 phi(
            0.4,
            -0.3,
            0.2);

        const Mat3 R = so3_exp(phi);

        const Mat3 identity_error =
            R.transpose() * R
            - Mat3::Identity();

        assert(identity_error.norm() < 1e-12);

        assert(
            std::abs(R.determinant() - 1.0)
            < 1e-12);
    }

    std::cout
        << "All SO(3) tests passed."
        << std::endl;

    return 0;
}

