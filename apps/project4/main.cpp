#include <iomanip>
#include <iostream>

#include "piper_control/common/constants.hpp"
#include "piper_control/common/types.hpp"
#include "piper_control/kinematics/forward_kinematics.hpp"

int main()
{
    using namespace piper_control;

    // 一个简单的 1-DOF POE 测试：
    // 绕空间 Z 轴旋转，M 的末端位于 X 方向 1 m。
    MatXd S(6, 1);

    S.col(0) << 0.0, 0.0, 1.0,
                0.0, 0.0, 0.0;

    VecXd q(1);
    q(0) = kPi / 2.0;

    Mat4 M = Mat4::Identity();
    M(0, 3) = 1.0;

    const Mat4 T =
        forward_poe_space(S, q, M);

    std::cout
        << std::fixed
        << std::setprecision(6)
        << "POE FK result:\n"
        << T
        << "project4\n\n";

    return 0;
}