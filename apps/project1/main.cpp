#include <iomanip>
#include <iostream>

#include "piper_control/lie_group/se3.hpp"

using namespace piper_control;

int main()
{
    Twist xi;
    xi << 0.3, -0.2, 0.5, 0.8, -1.1, 0.6;

    const Mat4 T = se3_exp(xi);
    const Twist xi_recovered = se3_log(T);

    std::cout << std::fixed << std::setprecision(6);

    std::cout << "SE(3) Exp result:\n";
    std::cout << T << "\n\n";

    std::cout << "Original twist:\n";
    std::cout << xi.transpose() << "\n\n";

    std::cout << "Recovered twist:\n";
    std::cout << xi_recovered.transpose() << "\n\n";

    std::cout << "Exp(Log(T)) error: "
              << (se3_exp(xi_recovered) - T).norm()
              << "\n";

    return 0;
}
