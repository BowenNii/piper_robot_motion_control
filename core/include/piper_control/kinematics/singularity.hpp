#pragma once

#include <Eigen/Dense>

#include "piper_control/common/types.hpp"

namespace piper_control
{

Eigen::VectorXd singular_values(
    const MatXd& J);

double minimum_singular_value(
    const MatXd& J);

double condition_number(
    const MatXd& J);

double manipulability(
    const MatXd& J);

MatXd pseudoinverse_svd(
    const MatXd& J,
    double tolerance = 1e-10);

MatXd dls_pseudoinverse(
    const MatXd& J,
    double lambda);

}  // namespace piper_control