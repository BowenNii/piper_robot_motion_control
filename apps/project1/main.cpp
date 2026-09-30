#include <iomanip>
#include <iostream>

#include "piper_control/common/constants.hpp"
#include "piper_control/common/types.hpp"
#include "piper_control/robot_model/piper_model.hpp"

using namespace piper_control;

int main()
{
    const PiperModel piper = make_piper_model();
    const Mat4 M = piper.M;
    std::cout << M << std::endl;
    const MatXd B_list=piper.B_list;
    std::cout << B_list << std::endl;
    const MatXd S_list=piper.S_list;
    std::cout << S_list << std::endl;
    const VecXd dq=piper.dq_max;
    std::cout << dq << std::endl;
    const VecXd q_max=piper.q_max;
    std::cout << q_max << std::endl;
    const VecXd q_min=piper.q_min;
    std::cout << q_min << std::endl;

    return 0;
}