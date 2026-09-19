#pragma once

#include "piper_control/common/types.hpp"

namespace piper_control
{

MatXd jacobian_space(
    const MatXd& S_list,
    const VecXd& q);

MatXd jacobian_body(
    const MatXd& B_list,
    const VecXd& q);

Vec6 twist_from_space_jacobian(
    const MatXd& J_space,
    const VecXd& q_dot);

Vec6 twist_from_body_jacobian(
    const MatXd& J_body,
    const VecXd& q_dot);

}  // namespace piper_control